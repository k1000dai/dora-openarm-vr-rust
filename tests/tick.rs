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

//! Behavior tests for [`TickProcessor`], the per-tick body of upstream
//! `_run`: validity gating/reset, One Euro smoothing, gripper mapping, and
//! output emission order.

// Matches production's deliberate f64 -> f32 narrowing at the wire boundary.
#![allow(clippy::cast_possible_truncation)]

use dora_openarm_vr_rust::message::QuestMessage;
use dora_openarm_vr_rust::output::EmissionValue;
use dora_openarm_vr_rust::tick::{TickProcessor, output_id};
use dora_openarm_vr_rust::transform::{self, RawPose};

fn msg(json: &str) -> QuestMessage {
    QuestMessage::parse(json.as_bytes()).expect("valid JSON object")
}

fn emission_ids(outcome: &dora_openarm_vr_rust::tick::TickOutcome) -> Vec<&'static str> {
    outcome.emissions.iter().map(|e| e.output_id).collect()
}

fn find_pose<'a>(
    outcome: &'a dora_openarm_vr_rust::tick::TickOutcome,
    id: &str,
) -> Option<&'a Vec<f32>> {
    outcome.emissions.iter().find_map(|e| {
        if e.output_id == id {
            match &e.value {
                EmissionValue::Pose(p) => Some(p),
                _ => None,
            }
        } else {
            None
        }
    })
}

#[test]
fn no_recv_timestamps_and_no_message_emits_nothing() {
    let mut tick = TickProcessor::new();
    let outcome = tick.process_tick(&[], None, 0.0);
    assert!(outcome.emissions.is_empty());
    assert!(outcome.validity_transition.is_none());
}

#[test]
fn recv_timestamps_are_emitted_even_when_there_is_no_latest_message() {
    let mut tick = TickProcessor::new();
    let outcome = tick.process_tick(&[10, 20], None, 0.0);
    assert_eq!(emission_ids(&outcome), vec![output_id::VR_RECEIVE_TIMES]);
    match &outcome.emissions[0].value {
        EmissionValue::Int64Vec(v) => assert_eq!(v, &vec![10i64, 20i64]),
        other => panic!("unexpected value: {other:?}"),
    }
}

#[test]
fn empty_recv_timestamps_are_not_emitted() {
    let mut tick = TickProcessor::new();
    let message =
        msg(r#"{"rc": {"x":0.1,"y":0.2,"z":0.3,"qx":0,"qy":0,"qz":0,"qw":1}, "rt": 0.0}"#);
    let outcome = tick.process_tick(&[], Some(&message), 0.0);
    assert!(!emission_ids(&outcome).contains(&output_id::VR_RECEIVE_TIMES));
}

#[test]
fn pose_right_requires_both_the_controller_pose_and_the_trigger() {
    let mut tick = TickProcessor::new();
    let message = msg(r#"{"rc": {"x":0.1,"y":0.2,"z":0.3,"qx":0,"qy":0,"qz":0,"qw":1}}"#);
    let outcome = tick.process_tick(&[], Some(&message), 0.0);
    assert!(!emission_ids(&outcome).contains(&output_id::POSE_RIGHT));
    // The trigger itself is independent of pose validity/presence and is
    // simply absent here too.
    assert!(!emission_ids(&outcome).contains(&output_id::TRIGGER_RIGHT));
}

#[test]
fn pose_reference_does_not_require_a_trigger() {
    let mut tick = TickProcessor::new();
    let message = msg(r#"{"rf": {"x":0.1,"y":0.2,"z":0.3,"qx":0,"qy":0,"qz":0,"qw":1}}"#);
    let outcome = tick.process_tick(&[], Some(&message), 0.0);
    assert!(emission_ids(&outcome).contains(&output_id::POSE_REFERENCE));
}

#[test]
fn pose_right_carries_the_gripper_angle_as_an_eighth_element() {
    let mut tick = TickProcessor::new();
    let message =
        msg(r#"{"rc": {"x":0.5,"y":-0.2,"z":0.1,"qx":0,"qy":0,"qz":0,"qw":1}, "rt": 1.0}"#);
    let outcome = tick.process_tick(&[], Some(&message), 0.0);

    let pose = find_pose(&outcome, output_id::POSE_RIGHT).unwrap();
    assert_eq!(pose.len(), 8);
    // First sample through a fresh smoother passes through unchanged, so
    // this must equal the raw `transform::process` output plus the gripper
    // angle for a fully-closed (1.0) right trigger.
    let raw = transform::process(
        None,
        Some(RawPose {
            x: 0.5,
            y: -0.2,
            z: 0.1,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        }),
        None,
    )
    .pose_right
    .unwrap();
    for i in 0..7 {
        assert!((pose[i] - raw[i]).abs() < 1e-6, "index {i}");
    }
    let gripper = transform::map_trigger_to_gripper(1.0, transform::Side::Right) as f32;
    assert!((pose[7] - gripper).abs() < 1e-6);
}

#[test]
fn full_message_emits_all_outputs_in_upstreams_exact_order() {
    let mut tick = TickProcessor::new();
    let message = msg(r#"{
            "rf": {"x":0,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":1},
            "rc": {"x":0.1,"y":0.2,"z":0.3,"qx":0,"qy":0,"qz":0,"qw":1},
            "lc": {"x":-0.1,"y":0.15,"z":0.25,"qx":0,"qy":0,"qz":0,"qw":1},
            "rt": 0.5, "lt": 0.5, "rg": 0.1, "lg": 0.2,
            "lsx": 0.3, "lsy": -0.3, "rsx": 0.4, "rsy": -0.4,
            "a": true, "b": false, "x": true, "y": false
        }"#);
    let outcome = tick.process_tick(&[100], Some(&message), 0.0);

    assert_eq!(
        emission_ids(&outcome),
        vec![
            output_id::VR_RECEIVE_TIMES,
            output_id::POSE_RIGHT,
            output_id::POSE_LEFT,
            output_id::POSE_REFERENCE,
            output_id::TRIGGER_RIGHT,
            output_id::TRIGGER_LEFT,
            output_id::GRIP_RIGHT,
            output_id::GRIP_LEFT,
            output_id::JOYSTICK_X_LEFT,
            output_id::JOYSTICK_Y_LEFT,
            output_id::JOYSTICK_X_RIGHT,
            output_id::JOYSTICK_Y_RIGHT,
            output_id::BUTTON_A,
            output_id::BUTTON_B,
            output_id::BUTTON_X,
            output_id::BUTTON_Y,
        ]
    );
}

#[test]
fn invalid_right_validity_suppresses_pose_right_and_resets_on_recovery() {
    let mut tick = TickProcessor::new();

    let first = msg(r#"{"rc": {"x":1,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":1}, "rt": 0.0}"#);
    let outcome1 = tick.process_tick(&[], Some(&first), 0.0);
    assert!(find_pose(&outcome1, output_id::POSE_RIGHT).is_some());

    let invalid =
        msg(r#"{"rc": {"x":5,"y":5,"z":5,"qx":0,"qy":0,"qz":0,"qw":1}, "rt": 0.0, "vr": 2}"#);
    let outcome2 = tick.process_tick(&[], Some(&invalid), 1.0);
    assert!(find_pose(&outcome2, output_id::POSE_RIGHT).is_none());

    let recovered = msg(r#"{"rc": {"x":2,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":1}, "rt": 0.0}"#);
    let outcome3 = tick.process_tick(&[], Some(&recovered), 2.0);
    let pose3 = find_pose(&outcome3, output_id::POSE_RIGHT).unwrap();

    // The smoother was reset on the OK -> INVALID -> OK round trip, so the
    // first sample after recovery passes through unfiltered.
    let raw = transform::process(
        None,
        Some(RawPose {
            x: 2.0,
            y: 0.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        }),
        None,
    )
    .pose_right
    .unwrap();
    for i in 0..7 {
        assert!((pose3[i] - raw[i]).abs() < 1e-6, "index {i}");
    }
}

#[test]
fn validity_change_produces_a_transition_message_only_on_change() {
    let mut tick = TickProcessor::new();

    let ok = msg(r#"{"v": 0}"#);
    let outcome1 = tick.process_tick(&[], Some(&ok), 0.0);
    assert!(outcome1.validity_transition.is_none());

    let stale = msg(r#"{"v": 1}"#);
    let outcome2 = tick.process_tick(&[], Some(&stale), 1.0);
    assert!(outcome2.validity_transition.is_some());

    let outcome3 = tick.process_tick(&[], Some(&stale), 2.0);
    assert!(outcome3.validity_transition.is_none());
}

#[test]
fn missing_validity_fields_default_to_ok() {
    let mut tick = TickProcessor::new();
    let message =
        msg(r#"{"rc": {"x":0.1,"y":0.2,"z":0.3,"qx":0,"qy":0,"qz":0,"qw":1}, "rt": 0.0}"#);
    let outcome = tick.process_tick(&[], Some(&message), 0.0);
    // No validity fields present -> defaults to OK -> pose is not suppressed.
    assert!(find_pose(&outcome, output_id::POSE_RIGHT).is_some());
}
