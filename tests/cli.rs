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

//! Behavior tests for command-line option parsing, matching upstream
//! `main()`'s `argparse` setup exactly: `--host` (default `0.0.0.0`) and
//! `--port` (default `5006`), no environment variable fallback.

use clap::Parser;
use dora_openarm_vr_rust::cli::Args;

#[test]
fn defaults_match_upstream() {
    let args = Args::try_parse_from(["dora-openarm-quest-receiver"]).unwrap();
    assert_eq!(args.host, "0.0.0.0");
    assert_eq!(args.port, 5006);
}

#[test]
fn host_flag_overrides_the_default() {
    let args =
        Args::try_parse_from(["dora-openarm-quest-receiver", "--host", "127.0.0.1"]).unwrap();
    assert_eq!(args.host, "127.0.0.1");
    assert_eq!(args.port, 5006);
}

#[test]
fn port_flag_overrides_the_default() {
    let args = Args::try_parse_from(["dora-openarm-quest-receiver", "--port", "9000"]).unwrap();
    assert_eq!(args.host, "0.0.0.0");
    assert_eq!(args.port, 9000);
}

#[test]
fn both_flags_together() {
    let args = Args::try_parse_from([
        "dora-openarm-quest-receiver",
        "--host",
        "192.168.1.1",
        "--port",
        "6000",
    ])
    .unwrap();
    assert_eq!(args.host, "192.168.1.1");
    assert_eq!(args.port, 6000);
}

#[test]
fn a_non_numeric_port_is_rejected() {
    let result = Args::try_parse_from(["dora-openarm-quest-receiver", "--port", "not-a-port"]);
    assert!(result.is_err());
}
