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

//! Behavior tests for [`ReceiverState`], the pure bookkeeping behind
//! upstream `JsonUdpReceiver`: the freshest successfully-parsed packet, and
//! a maxlen-512 log of successful-parse arrival timestamps.

use dora_openarm_vr_rust::receiver::ReceiverState;

#[test]
fn a_fresh_receiver_has_no_latest_message_or_timestamps() {
    let state = ReceiverState::new();
    assert!(state.latest().is_none());
    let mut state = state;
    assert!(state.drain_recv_timestamps().is_empty());
}

#[test]
fn a_valid_datagram_becomes_the_latest_message_and_records_its_timestamp() {
    let mut state = ReceiverState::new();
    state.record_datagram(100, br#"{"v": 0}"#);

    assert!(state.latest().is_some());
    assert_eq!(state.drain_recv_timestamps(), vec![100]);
}

#[test]
fn an_unparseable_datagram_updates_neither_latest_nor_timestamps() {
    let mut state = ReceiverState::new();
    state.record_datagram(100, b"not json");

    assert!(state.latest().is_none());
    assert!(state.drain_recv_timestamps().is_empty());
}

#[test]
fn a_later_unparseable_datagram_does_not_overwrite_the_previous_latest() {
    let mut state = ReceiverState::new();
    state.record_datagram(100, br#"{"v": 0}"#);
    state.record_datagram(200, b"not json");

    let latest = state.latest().unwrap();
    assert_eq!(latest.get_i64("v"), Some(0));
    // Only the successful parse's timestamp is recorded.
    assert_eq!(state.drain_recv_timestamps(), vec![100]);
}

#[test]
fn latest_reflects_the_most_recently_parsed_message() {
    let mut state = ReceiverState::new();
    state.record_datagram(100, br#"{"v": 0}"#);
    state.record_datagram(200, br#"{"v": 1}"#);

    assert_eq!(state.latest().unwrap().get_i64("v"), Some(1));
}

#[test]
fn draining_timestamps_clears_them_but_not_the_latest_message() {
    let mut state = ReceiverState::new();
    state.record_datagram(100, br#"{"v": 0}"#);

    assert_eq!(state.drain_recv_timestamps(), vec![100]);
    assert!(state.drain_recv_timestamps().is_empty());
    assert!(state.latest().is_some());
}

#[test]
fn timestamps_accumulate_across_multiple_datagrams_in_arrival_order() {
    let mut state = ReceiverState::new();
    state.record_datagram(10, br#"{"v": 0}"#);
    state.record_datagram(20, br#"{"v": 0}"#);
    state.record_datagram(30, br#"{"v": 0}"#);

    assert_eq!(state.drain_recv_timestamps(), vec![10, 20, 30]);
}

#[test]
fn timestamp_log_evicts_the_oldest_entry_beyond_512() {
    let mut state = ReceiverState::new();
    for i in 0..515i64 {
        state.record_datagram(i, br#"{"v": 0}"#);
    }

    let drained = state.drain_recv_timestamps();
    assert_eq!(drained.len(), 512);
    assert_eq!(drained.first(), Some(&3));
    assert_eq!(drained.last(), Some(&514));
}
