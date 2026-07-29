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

//! Behavior tests for parsing a Quest UDP datagram into a [`QuestMessage`],
//! matching upstream `JsonUdpReceiver._parse_packet` and the field-presence
//! semantics `quest_receiver.py`'s `_run` relies on (`"key" in msg`).

use dora_openarm_vr_rust::message::QuestMessage;

#[test]
fn parses_a_well_formed_packet() {
    let data = br#"{"t": 1.0, "rt": 0.5, "a": true, "v": 0}"#;
    assert!(QuestMessage::parse(data).is_some());
}

#[test]
fn empty_datagram_is_not_a_message() {
    assert!(QuestMessage::parse(b"").is_none());
}

#[test]
fn whitespace_only_datagram_is_not_a_message() {
    assert!(QuestMessage::parse(b"   \n  ").is_none());
}

#[test]
fn invalid_json_is_not_a_message() {
    assert!(QuestMessage::parse(b"{not json").is_none());
}

#[test]
fn a_json_array_is_not_a_message() {
    // The wire protocol is always a JSON object; a valid-but-non-object
    // payload has no fields to read and is treated as unparseable.
    assert!(QuestMessage::parse(b"[1, 2, 3]").is_none());
}

#[test]
fn trims_surrounding_whitespace_and_trailing_newline() {
    let data = b"  {\"v\": 1}  \n";
    let msg = QuestMessage::parse(data).unwrap();
    assert_eq!(msg.get_i64("v"), Some(1));
}

#[test]
fn has_field_is_true_only_when_the_key_is_present() {
    let msg = QuestMessage::parse(br#"{"rt": 0.5, "explicit_null": null}"#).unwrap();
    assert!(msg.has_field("rt"));
    assert!(msg.has_field("explicit_null"));
    assert!(!msg.has_field("missing"));
}

#[test]
fn get_f64_reads_a_present_numeric_field() {
    let msg = QuestMessage::parse(br#"{"rt": 0.5}"#).unwrap();
    assert_eq!(msg.get_f64("rt"), Some(0.5));
    assert_eq!(msg.get_f64("missing"), None);
}

#[test]
fn get_f64_of_a_null_field_is_none() {
    let msg = QuestMessage::parse(br#"{"rt": null}"#).unwrap();
    assert_eq!(msg.get_f64("rt"), None);
}

#[test]
fn get_bool_matches_python_truthiness_for_common_json_types() {
    let msg = QuestMessage::parse(
        br#"{"t": true, "f": false, "one": 1, "zero": 0, "s": "x", "empty_s": ""}"#,
    )
    .unwrap();
    assert_eq!(msg.get_bool("t"), Some(true));
    assert_eq!(msg.get_bool("f"), Some(false));
    assert_eq!(msg.get_bool("one"), Some(true));
    assert_eq!(msg.get_bool("zero"), Some(false));
    assert_eq!(msg.get_bool("s"), Some(true));
    assert_eq!(msg.get_bool("empty_s"), Some(false));
    assert_eq!(msg.get_bool("missing"), None);
}

#[test]
fn get_i64_reads_integers_and_truncates_floats() {
    let msg = QuestMessage::parse(br#"{"v": 2, "vf": 1.9}"#).unwrap();
    assert_eq!(msg.get_i64("v"), Some(2));
    assert_eq!(msg.get_i64("vf"), Some(1));
    assert_eq!(msg.get_i64("missing"), None);
}

#[test]
fn get_pose_reads_a_full_pose_object() {
    let msg = QuestMessage::parse(
        br#"{"rc": {"x": 1.0, "y": 2.0, "z": 3.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}}"#,
    )
    .unwrap();
    let pose = msg.get_pose("rc").unwrap();
    assert!((pose.x - 1.0).abs() < f64::EPSILON);
    assert!((pose.y - 2.0).abs() < f64::EPSILON);
    assert!((pose.z - 3.0).abs() < f64::EPSILON);
    assert!((pose.qw - 1.0).abs() < f64::EPSILON);
}

#[test]
fn get_pose_of_a_missing_key_is_none() {
    let msg = QuestMessage::parse(br"{}").unwrap();
    assert!(msg.get_pose("rc").is_none());
}

#[test]
fn get_pose_of_an_explicit_null_is_none() {
    let msg = QuestMessage::parse(br#"{"rc": null}"#).unwrap();
    assert!(msg.get_pose("rc").is_none());
}

#[test]
fn get_pose_of_an_incomplete_object_is_none() {
    let msg = QuestMessage::parse(br#"{"rc": {"x": 1.0}}"#).unwrap();
    assert!(msg.get_pose("rc").is_none());
}
