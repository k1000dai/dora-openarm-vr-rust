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
//! setup exactly: `--host` (default `"0.0.0.0"`), `--port` (default `5006`),
//! `--max-linear-speed` (default `1.0`) and `--max-angular-speed` (default
//! `6.0`). Unlike some sibling `OpenArm` nodes, upstream `quest_receiver.py`
//! reads none of them from an environment variable, so this parser doesn't
//! either.
//!
//! Upstream validates the two speed limits after parsing
//! (`if not np.isfinite(value) or value < 0.0: parser.error(...)`); here the
//! same check runs inside a clap value parser, so the rejection happens
//! during parsing. Both exit with a usage error; only the surrounding
//! wording of the message differs.

use clap::Parser;

/// Upstream's `--max-linear-speed` default, in m/s.
pub const DEFAULT_MAX_LINEAR_SPEED: f64 = 1.0;
/// Upstream's `--max-angular-speed` default, in rad/s.
pub const DEFAULT_MAX_ANGULAR_SPEED: f64 = 6.0;

/// Resolved command-line options for the Quest UDP receiver.
#[derive(Parser, Debug, Clone, PartialEq)]
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
    /// Maximum output translation speed in m/s; 0 disables it.
    #[arg(
        long,
        default_value_t = DEFAULT_MAX_LINEAR_SPEED,
        value_parser = parse_speed_limit,
        help = "Maximum output translation speed in m/s; 0 disables it (default: 1.0)."
    )]
    pub max_linear_speed: f64,
    /// Maximum output rotation speed in rad/s; 0 disables it.
    #[arg(
        long,
        default_value_t = DEFAULT_MAX_ANGULAR_SPEED,
        value_parser = parse_speed_limit,
        help = "Maximum output rotation speed in rad/s; 0 disables it (default: 6.0)."
    )]
    pub max_angular_speed: f64,
}

/// Parses a speed limit, rejecting anything upstream's post-parse check
/// rejects: non-finite (infinity or NaN) or negative values.
fn parse_speed_limit(raw: &str) -> Result<f64, String> {
    let value: f64 = raw
        .parse()
        .map_err(|_| format!("invalid float value: '{raw}'"))?;
    if !value.is_finite() || value < 0.0 {
        return Err("must be finite and non-negative".to_string());
    }
    Ok(value)
}
