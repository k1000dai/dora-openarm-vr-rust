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

//! Pure bookkeeping behind upstream `JsonUdpReceiver`.
//!
//! Upstream's background thread drains every queued datagram per wakeup,
//! parses each one, and (a) keeps only the last successfully-parsed packet
//! as `latest`, and (b) appends every successfully-parsed packet's arrival
//! time to a `maxlen=512` deque -- packets that fail to parse contribute
//! neither a `latest` update nor a timestamp. Because both effects are
//! per-successfully-parsed-packet, the observable state is identical
//! whether packets are recorded one at a time or in upstream's batches, so
//! this type models it one datagram at a time via [`ReceiverState::record_datagram`].
//! The actual socket I/O that feeds it lives in the `dora-openarm-quest-receiver`
//! binary.

use std::collections::VecDeque;

use crate::message::QuestMessage;

/// The maximum number of arrival timestamps retained, matching upstream
/// `collections.deque(maxlen=512)`.
const MAX_TIMESTAMPS: usize = 512;

/// The freshest parsed Quest packet, plus a bounded log of arrival
/// timestamps (nanoseconds) for successfully-parsed packets.
#[derive(Debug, Clone, Default)]
pub struct ReceiverState {
    latest: Option<QuestMessage>,
    recv_ts: VecDeque<i64>,
}

impl ReceiverState {
    /// Builds an empty receiver state: no latest message, no timestamps.
    #[must_use]
    pub fn new() -> Self {
        Self {
            latest: None,
            recv_ts: VecDeque::new(),
        }
    }

    /// Records one UDP datagram received at `recv_ns` (nanoseconds).
    ///
    /// If `data` parses as a Quest packet, it becomes [`Self::latest`] and
    /// `recv_ns` is appended to the timestamp log (evicting the oldest
    /// entry past [`MAX_TIMESTAMPS`]). An unparseable datagram is dropped
    /// silently, matching upstream's `except json.JSONDecodeError: return
    /// None` leaving `last_msg`/`arrivals` untouched.
    pub fn record_datagram(&mut self, recv_ns: i64, data: &[u8]) {
        let Some(message) = QuestMessage::parse(data) else {
            return;
        };
        self.recv_ts.push_back(recv_ns);
        if self.recv_ts.len() > MAX_TIMESTAMPS {
            self.recv_ts.pop_front();
        }
        self.latest = Some(message);
    }

    /// Returns the freshest successfully-parsed packet, if any, matching
    /// upstream `latest()`.
    #[must_use]
    pub fn latest(&self) -> Option<&QuestMessage> {
        self.latest.as_ref()
    }

    /// Returns and clears the recorded arrival timestamps, in arrival
    /// order, matching upstream `drain_recv_timestamps()`.
    pub fn drain_recv_timestamps(&mut self) -> Vec<i64> {
        self.recv_ts.drain(..).collect()
    }
}
