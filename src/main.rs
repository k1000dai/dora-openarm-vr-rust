// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Meta Quest VR pose receiver node for `OpenArm` teleoperation.
//!
//! This dora-rs node binds a UDP socket, parses newline-delimited JSON
//! packets from a sideloaded Quest APK, converts Unity left-handed
//! controller poses into the `OpenArm` workspace, smooths them with a One
//! Euro filter, and publishes the pose, trigger, grip, joystick and button
//! state as dora-rs outputs on every `tick` input.
//!
//! This adapter is intentionally thin: all parsing/transform/filter/gating
//! logic lives in the library (`src/lib.rs` and its modules) and is tested
//! there without a running dataflow or a real headset. The UDP background
//! thread mirrors upstream `JsonUdpReceiver`: it binds (retrying on
//! failure), blocks on `recv_from` with a timeout, and drains any
//! additional queued datagrams non-blockingly before going back to sleep.
//! Matching upstream, this thread is never joined -- it is abandoned when
//! the process exits.

use std::io;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::Parser;
use dora_node_api::dora_core::config::DataId;
use dora_node_api::{DoraNode, Event, MetadataParameters, Parameter};
use dora_openarm_vr_rust::cli::Args;
use dora_openarm_vr_rust::message::QuestMessage;
use dora_openarm_vr_rust::output::{EmissionValue, build_array};
use dora_openarm_vr_rust::receiver::ReceiverState;
use dora_openarm_vr_rust::tick::{TickProcessor, output_id};

/// Bytes read per UDP datagram, matching upstream `JsonUdpReceiver`'s
/// default `buf_size=4096`.
const RECV_BUF_SIZE: usize = 4096;

/// Background UDP receiver: binds `host:port` and keeps [`ReceiverState`]
/// updated, matching upstream `JsonUdpReceiver`.
struct UdpQuestReceiver {
    state: Arc<Mutex<ReceiverState>>,
    running: Arc<AtomicBool>,
}

impl UdpQuestReceiver {
    fn start(host: String, port: u16) -> Self {
        let state = Arc::new(Mutex::new(ReceiverState::new()));
        let running = Arc::new(AtomicBool::new(true));

        let thread_state = Arc::clone(&state);
        let thread_running = Arc::clone(&running);
        std::thread::spawn(move || udp_receive_loop(&host, port, &thread_state, &thread_running));

        Self { state, running }
    }

    fn drain_recv_timestamps(&self) -> Vec<i64> {
        lock(&self.state).drain_recv_timestamps()
    }

    fn latest(&self) -> Option<QuestMessage> {
        lock(&self.state).latest().cloned()
    }

    /// Signals the background thread to stop, matching upstream
    /// `receiver.close()`. The thread itself is not joined.
    fn close(&self) {
        self.running.store(false, Ordering::SeqCst);
    }
}

fn lock(state: &Arc<Mutex<ReceiverState>>) -> std::sync::MutexGuard<'_, ReceiverState> {
    state.lock().unwrap_or_else(PoisonError::into_inner)
}

fn record(state: &Arc<Mutex<ReceiverState>>, recv_ns: i64, data: &[u8]) {
    lock(state).record_datagram(recv_ns, data);
}

/// Mirrors upstream `JsonUdpReceiver._loop`: bind (retrying every second on
/// failure), block on `recv_from` with a 1 second timeout, then drain any
/// additional queued datagrams without blocking before waiting again.
fn udp_receive_loop(
    host: &str,
    port: u16,
    state: &Arc<Mutex<ReceiverState>>,
    running: &Arc<AtomicBool>,
) {
    let address = format!("{host}:{port}");

    while running.load(Ordering::SeqCst) {
        let Ok(socket) = UdpSocket::bind(&address) else {
            if running.load(Ordering::SeqCst) {
                std::thread::sleep(Duration::from_secs(1));
            }
            continue;
        };
        if socket
            .set_read_timeout(Some(Duration::from_secs(1)))
            .is_err()
        {
            continue;
        }

        let mut buf = [0u8; RECV_BUF_SIZE];
        while running.load(Ordering::SeqCst) {
            match socket.recv_from(&mut buf) {
                Ok((len, _)) => {
                    record(state, now_ns(), &buf[..len]);
                    drain_queued_datagrams(&socket, &mut buf, state);
                }
                Err(err)
                    if matches!(
                        err.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) => {}
                Err(_) => {}
            }
        }
    }
}

/// Reads every datagram already queued on `socket` without blocking,
/// matching upstream's `while select.select([srv], [], [], 0.0)[0]:` drain
/// loop.
fn drain_queued_datagrams(socket: &UdpSocket, buf: &mut [u8], state: &Arc<Mutex<ReceiverState>>) {
    if socket.set_nonblocking(true).is_err() {
        return;
    }
    while let Ok((len, _)) = socket.recv_from(buf) {
        record(state, now_ns(), &buf[..len]);
    }
    let _ = socket.set_nonblocking(false);
    let _ = socket.set_read_timeout(Some(Duration::from_secs(1)));
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_nanos()).unwrap_or(i64::MAX))
}

fn main() -> eyre::Result<()> {
    let args = Args::parse();
    let receiver = UdpQuestReceiver::start(args.host, args.port);

    let (mut node, mut events) = DoraNode::init_from_env()?;
    node.send_output(
        DataId::from(output_id::STATUS.to_owned()),
        MetadataParameters::new(),
        build_array(EmissionValue::Status("ready")),
    )?;

    let mut tick_processor = TickProcessor::new();
    let start = Instant::now();

    while let Some(event) = events.recv() {
        let Event::Input { id, .. } = event else {
            continue;
        };
        if id.as_str() != "tick" {
            continue;
        }

        let recv_ts = receiver.drain_recv_timestamps();
        let msg = receiver.latest();
        let now = start.elapsed().as_secs_f64();
        let outcome = tick_processor.process_tick(&recv_ts, msg.as_ref(), now);

        if let Some(log) = outcome.validity_transition {
            println!("{log}");
        }

        let timestamp_ns = now_ns();
        for emission in outcome.emissions {
            let mut parameters = MetadataParameters::new();
            if emission.output_id != output_id::VR_RECEIVE_TIMES {
                parameters.insert("timestamp".to_string(), Parameter::Integer(timestamp_ns));
            }
            let array = build_array(emission.value);
            node.send_output(
                DataId::from(emission.output_id.to_owned()),
                parameters,
                array,
            )?;
        }
    }

    receiver.close();
    Ok(())
}
