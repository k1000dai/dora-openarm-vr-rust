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

//! Pure logic of the Meta Quest VR -> `OpenArm` teleoperation node.
//!
//! This library knows nothing about dora-rs or UDP sockets. It answers
//! "given this parsed Quest packet (and existing filter state), what should
//! the node emit?" so that the answer can be tested without a running
//! dataflow or a real headset. The dora-rs/UDP event loop that feeds it
//! lives in the upstream-compatible `dora-openarm-quest-receiver` binary
//! (`src/main.rs`).

pub mod cli;
pub mod message;
pub mod output;
pub mod quaternion;
pub mod receiver;
pub mod smoothing;
pub mod tick;
pub mod transform;
