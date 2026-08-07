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
//! `main()`'s `argparse` setup exactly: `--host` (default `0.0.0.0`),
//! `--port` (default `5006`), `--max-linear-speed` (default `1.0`) and
//! `--max-angular-speed` (default `6.0`), no environment variable fallback.

use clap::Parser;
use dora_openarm_vr_rust::cli::{Args, DEFAULT_MAX_ANGULAR_SPEED, DEFAULT_MAX_LINEAR_SPEED};

#[test]
fn defaults_match_upstream() {
    let args = Args::try_parse_from(["dora-openarm-quest-receiver"]).unwrap();
    assert_eq!(args.host, "0.0.0.0");
    assert_eq!(args.port, 5006);
    assert!((args.max_linear_speed - 1.0).abs() < f64::EPSILON);
    assert!((args.max_angular_speed - 6.0).abs() < f64::EPSILON);
    assert!((DEFAULT_MAX_LINEAR_SPEED - 1.0).abs() < f64::EPSILON);
    assert!((DEFAULT_MAX_ANGULAR_SPEED - 6.0).abs() < f64::EPSILON);
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

#[test]
fn speed_limit_flags_override_the_defaults() {
    let args = Args::try_parse_from([
        "dora-openarm-quest-receiver",
        "--max-linear-speed",
        "0.25",
        "--max-angular-speed",
        "2.5",
    ])
    .unwrap();
    assert!((args.max_linear_speed - 0.25).abs() < f64::EPSILON);
    assert!((args.max_angular_speed - 2.5).abs() < f64::EPSILON);
}

#[test]
fn zero_speed_limits_are_accepted_and_disable_limiting() {
    let args = Args::try_parse_from([
        "dora-openarm-quest-receiver",
        "--max-linear-speed",
        "0",
        "--max-angular-speed",
        "0",
    ])
    .unwrap();
    assert!(args.max_linear_speed.abs() < f64::EPSILON);
    assert!(args.max_angular_speed.abs() < f64::EPSILON);
}

// Values are passed as `--flag=value` so that clap reads a leading `-` as
// part of the value rather than as another flag.
#[test]
fn negative_speed_limits_are_rejected() {
    for flag in ["--max-linear-speed", "--max-angular-speed"] {
        let arg = format!("{flag}=-1.0");
        let result = Args::try_parse_from(["dora-openarm-quest-receiver", arg.as_str()]);
        let err = result.expect_err("a negative speed limit must be rejected");
        assert!(
            err.to_string().contains("must be finite and non-negative"),
            "{flag} should report upstream's finite/non-negative message, got: {err}"
        );
    }
}

#[test]
fn non_finite_speed_limits_are_rejected() {
    for flag in ["--max-linear-speed", "--max-angular-speed"] {
        for value in ["inf", "-inf", "nan"] {
            let arg = format!("{flag}={value}");
            let result = Args::try_parse_from(["dora-openarm-quest-receiver", arg.as_str()]);
            let err = result.expect_err("a non-finite speed limit must be rejected");
            assert!(
                err.to_string().contains("must be finite and non-negative"),
                "{flag}={value} should report upstream's message, got: {err}"
            );
        }
    }
}

#[test]
fn a_non_numeric_speed_limit_is_rejected() {
    let result = Args::try_parse_from([
        "dora-openarm-quest-receiver",
        "--max-linear-speed",
        "not-a-speed",
    ]);
    assert!(result.is_err());
}
