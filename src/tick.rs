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

//! Per-tick state and decision logic, matching upstream `_run`'s body for
//! each `tick` INPUT event.
//!
//! [`TickProcessor`] owns the three One Euro smoothers upstream constructs
//! once at startup (`smoother_right`, `smoother_left`, `smoother_reference`)
//! and the previous-validity state used to detect INVALID transitions. It
//! turns one tick's drained receive timestamps and latest message into the
//! ordered list of dora outputs to send. It knows nothing about dora-rs,
//! Arrow, or sockets -- the adapter in `main.rs` walks the returned
//! [`Emission`]s and sends them.
//!
//! Upstream tracks two nominally-separate "previous overall validity"
//! variables: `prev_v_overall` (gates the validity-change log line) and
//! `prev_v_reference` (gates the reference smoother's reset). Both are set
//! to the current tick's `v_overall` by the end of every tick, so they are
//! always equal at the start of the next tick; this module tracks the one
//! merged value as `prev_overall_validity`.

// The trigger/grip/joystick `as f32` narrowing matches upstream's explicit
// `pa.array(..., type=pa.float32())` casts.
#![allow(clippy::cast_possible_truncation)]

use crate::message::QuestMessage;
use crate::output::EmissionValue;
use crate::smoothing::OneEuroPoseSmoother;
use crate::transform::{self, Side};

/// Quest validity codes, matching upstream `VALID_OK`/`VALID_STALE`/`VALID_INVALID`.
const VALID_OK: i64 = 0;
const VALID_INVALID: i64 = 2;

/// Output ids, matching upstream `main.py` exactly.
pub mod output_id {
    /// `"ready"` once at startup, before the first tick.
    pub const STATUS: &str = "status";
    /// Arrival timestamps (ns) drained since the previous tick.
    pub const VR_RECEIVE_TIMES: &str = "vr_receive_times";
    /// The right controller pose plus gripper angle.
    pub const POSE_RIGHT: &str = "pose_right";
    /// The left controller pose plus gripper angle.
    pub const POSE_LEFT: &str = "pose_left";
    /// The reference pose (no gripper angle appended).
    pub const POSE_REFERENCE: &str = "pose_reference";
    /// The right index trigger value.
    pub const TRIGGER_RIGHT: &str = "trigger_right";
    /// The left index trigger value.
    pub const TRIGGER_LEFT: &str = "trigger_left";
    /// The right grip value.
    pub const GRIP_RIGHT: &str = "grip_right";
    /// The left grip value.
    pub const GRIP_LEFT: &str = "grip_left";
    /// The left joystick X axis.
    pub const JOYSTICK_X_LEFT: &str = "joystick_x_left";
    /// The left joystick Y axis.
    pub const JOYSTICK_Y_LEFT: &str = "joystick_y_left";
    /// The right joystick X axis.
    pub const JOYSTICK_X_RIGHT: &str = "joystick_x_right";
    /// The right joystick Y axis.
    pub const JOYSTICK_Y_RIGHT: &str = "joystick_y_right";
    /// The A button.
    pub const BUTTON_A: &str = "button_a";
    /// The B button.
    pub const BUTTON_B: &str = "button_b";
    /// The X button.
    pub const BUTTON_X: &str = "button_x";
    /// The Y button.
    pub const BUTTON_Y: &str = "button_y";
}

/// One dora output produced while processing a single tick.
#[derive(Debug, Clone, PartialEq)]
pub struct Emission {
    /// The dora output id.
    pub output_id: &'static str,
    /// The value to send.
    pub value: EmissionValue,
}

/// The result of processing one `tick` event: the outputs to send, in
/// upstream's exact emission order, plus an optional validity-transition
/// log line (matching upstream's `print(...)` on a `v_overall` change).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TickOutcome {
    /// Outputs to send, in order.
    pub emissions: Vec<Emission>,
    /// A human-readable validity transition message, present only when
    /// `v_overall` changed from the previous tick.
    pub validity_transition: Option<String>,
}

fn validity_name(code: i64) -> String {
    match code {
        VALID_OK => "OK".to_string(),
        1 => "STALE".to_string(),
        VALID_INVALID => "INVALID".to_string(),
        other => other.to_string(),
    }
}

fn emit(emissions: &mut Vec<Emission>, output_id: &'static str, value: EmissionValue) {
    emissions.push(Emission { output_id, value });
}

/// Per-node processing state: the three One Euro smoothers and the
/// previous-tick validity codes needed to detect INVALID transitions,
/// matching the state upstream `_run` keeps in local variables.
#[derive(Debug, Clone)]
pub struct TickProcessor {
    smoother_right: OneEuroPoseSmoother,
    smoother_left: OneEuroPoseSmoother,
    smoother_reference: OneEuroPoseSmoother,
    prev_v_right: i64,
    prev_v_left: i64,
    prev_overall_validity: i64,
}

impl Default for TickProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl TickProcessor {
    /// Builds a fresh processor with the smoother parameters upstream uses
    /// in `_run` (`min_cutoff=2.0, beta=0.04, d_cutoff=1.5`) and all
    /// previous-validity state at `VALID_OK`.
    #[must_use]
    pub fn new() -> Self {
        Self {
            smoother_right: OneEuroPoseSmoother::new(2.0, 0.04, 1.5),
            smoother_left: OneEuroPoseSmoother::new(2.0, 0.04, 1.5),
            smoother_reference: OneEuroPoseSmoother::new(2.0, 0.04, 1.5),
            prev_v_right: VALID_OK,
            prev_v_left: VALID_OK,
            prev_overall_validity: VALID_OK,
        }
    }

    /// Processes one `tick` event.
    ///
    /// `recv_ts` are the receiver's drained arrival timestamps for this
    /// tick; `msg` is the receiver's latest parsed packet, if any; `now` is
    /// a monotonic clock reading in seconds (upstream's
    /// `time.perf_counter()`), used only by the One Euro filters.
    #[must_use]
    pub fn process_tick(
        &mut self,
        recv_ts: &[i64],
        msg: Option<&QuestMessage>,
        now: f64,
    ) -> TickOutcome {
        let mut emissions = Vec::new();
        if !recv_ts.is_empty() {
            emit(
                &mut emissions,
                output_id::VR_RECEIVE_TIMES,
                EmissionValue::Int64Vec(recv_ts.to_vec()),
            );
        }

        let Some(msg) = msg else {
            return TickOutcome {
                emissions,
                validity_transition: None,
            };
        };

        let v_overall = msg.get_i64("v").unwrap_or(VALID_OK);
        let v_right = msg.get_i64("vr").unwrap_or(VALID_OK);
        let v_left = msg.get_i64("vl").unwrap_or(VALID_OK);

        // Captured before any mutation: this is both the value the
        // validity-change log line compares against, and (because upstream's
        // `prev_v_overall` and `prev_v_reference` are always equal at the
        // start of a tick -- see the module doc comment) the value that
        // gates the reference smoother's reset.
        let prev_overall = self.prev_overall_validity;

        let validity_transition = if v_overall == prev_overall {
            None
        } else {
            Some(format!(
                "[receiver] validity: {} -> {} (L={}, R={})",
                validity_name(prev_overall),
                validity_name(v_overall),
                validity_name(v_left),
                validity_name(v_right),
            ))
        };

        let outcome =
            transform::process(msg.get_pose("rf"), msg.get_pose("rc"), msg.get_pose("lc"));

        let pose_right = if v_right == VALID_INVALID {
            if self.prev_v_right != VALID_INVALID {
                self.smoother_right.reset();
            }
            None
        } else {
            self.smoother_right.smooth(now, outcome.pose_right)
        };

        let pose_left = if v_left == VALID_INVALID {
            if self.prev_v_left != VALID_INVALID {
                self.smoother_left.reset();
            }
            None
        } else {
            self.smoother_left.smooth(now, outcome.pose_left)
        };

        let pose_reference = if v_overall == VALID_INVALID {
            if prev_overall != VALID_INVALID {
                self.smoother_reference.reset();
            }
            None
        } else {
            self.smoother_reference.smooth(now, outcome.pose_reference)
        };

        self.prev_v_right = v_right;
        self.prev_v_left = v_left;
        self.prev_overall_validity = v_overall;

        if let (Some(pose), Some(rt)) = (pose_right, msg.get_f64("rt")) {
            emit_pose_with_gripper(&mut emissions, output_id::POSE_RIGHT, pose, rt, Side::Right);
        }
        if let (Some(pose), Some(lt)) = (pose_left, msg.get_f64("lt")) {
            emit_pose_with_gripper(&mut emissions, output_id::POSE_LEFT, pose, lt, Side::Left);
        }
        if let Some(pose) = pose_reference {
            emit(
                &mut emissions,
                output_id::POSE_REFERENCE,
                EmissionValue::Pose(pose.to_vec()),
            );
        }

        emit_if_present(&mut emissions, msg, "rt", output_id::TRIGGER_RIGHT);
        emit_if_present(&mut emissions, msg, "lt", output_id::TRIGGER_LEFT);
        emit_if_present(&mut emissions, msg, "rg", output_id::GRIP_RIGHT);
        emit_if_present(&mut emissions, msg, "lg", output_id::GRIP_LEFT);
        emit_if_present(&mut emissions, msg, "lsx", output_id::JOYSTICK_X_LEFT);
        emit_if_present(&mut emissions, msg, "lsy", output_id::JOYSTICK_Y_LEFT);
        emit_if_present(&mut emissions, msg, "rsx", output_id::JOYSTICK_X_RIGHT);
        emit_if_present(&mut emissions, msg, "rsy", output_id::JOYSTICK_Y_RIGHT);

        emit_bool_if_present(&mut emissions, msg, "a", output_id::BUTTON_A);
        emit_bool_if_present(&mut emissions, msg, "b", output_id::BUTTON_B);
        emit_bool_if_present(&mut emissions, msg, "x", output_id::BUTTON_X);
        emit_bool_if_present(&mut emissions, msg, "y", output_id::BUTTON_Y);

        TickOutcome {
            emissions,
            validity_transition,
        }
    }
}

fn emit_pose_with_gripper(
    emissions: &mut Vec<Emission>,
    output_id: &'static str,
    pose: [f32; 7],
    trigger: f64,
    side: Side,
) {
    let gripper = transform::map_trigger_to_gripper(trigger, side) as f32;
    let mut with_gripper = Vec::with_capacity(8);
    with_gripper.extend_from_slice(&pose);
    with_gripper.push(gripper);
    emit(emissions, output_id, EmissionValue::Pose(with_gripper));
}

fn emit_if_present(emissions: &mut Vec<Emission>, msg: &QuestMessage, key: &str, id: &'static str) {
    if let Some(v) = msg.get_f64(key) {
        emit(emissions, id, EmissionValue::Float32(v as f32));
    }
}

fn emit_bool_if_present(
    emissions: &mut Vec<Emission>,
    msg: &QuestMessage,
    key: &str,
    id: &'static str,
) {
    if let Some(v) = msg.get_bool(key) {
        emit(emissions, id, EmissionValue::Bool(v));
    }
}
