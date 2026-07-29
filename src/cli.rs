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

//! Command-line option parsing, matching upstream `main()`'s `argparse`
//! setup exactly: `--host` (default `"0.0.0.0"`) and `--port` (default
//! `5006`). Unlike some sibling `OpenArm` nodes, upstream `quest_receiver.py`
//! reads neither option from an environment variable, so this parser
//! doesn't either.

use clap::Parser;

/// Resolved command-line options for the Quest UDP receiver.
#[derive(Parser, Debug, Clone, PartialEq, Eq)]
#[command(
    name = "dora-openarm-quest-receiver",
    about = "Meta Quest VR pose receiver (dora node)"
)]
pub struct Args {
    /// The host/address to bind the UDP socket to.
    #[arg(long, default_value = "0.0.0.0")]
    pub host: String,
    /// The UDP port to bind the socket to.
    #[arg(long, default_value_t = 5006)]
    pub port: u16,
}
