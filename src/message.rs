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

//! Parsing a Quest UDP datagram, matching upstream
//! `JsonUdpReceiver._parse_packet` and the field-presence semantics
//! `quest_receiver.py`'s `_run` relies on (`"key" in msg`).
//!
//! Upstream's wire protocol is always a JSON object (see the module
//! docstring of `quest_receiver.py`); a valid-but-non-object JSON payload
//! (e.g. a bare array or number) has no fields for `_run` to read, so it is
//! treated as unparseable here rather than modeling the `AttributeError`
//! Python's dynamic typing would raise on first field access. Likewise, a
//! field whose value doesn't coerce to the type `_run` expects (e.g. `"rt":
//! "oops"`) is treated as absent rather than reproducing an upstream
//! `TypeError`/`ValueError` crash -- real Quest packets never send
//! mistyped fields.

use crate::transform::RawPose;
use serde_json::Value;

/// A parsed Quest UDP packet: an opaque JSON object with presence-aware
/// field accessors, mirroring how upstream reads `msg: dict`.
#[derive(Debug, Clone, PartialEq)]
pub struct QuestMessage {
    fields: serde_json::Map<String, Value>,
}

impl QuestMessage {
    /// Parses one UDP datagram, matching upstream `_parse_packet`: decode
    /// as UTF-8 (invalid sequences replaced, matching Python's
    /// `errors="replace"`), strip surrounding whitespace, and reject empty
    /// or non-JSON-object payloads.
    #[must_use]
    pub fn parse(data: &[u8]) -> Option<Self> {
        let text = String::from_utf8_lossy(data);
        let trimmed = text.trim();
        if trimmed.is_empty() {
            return None;
        }
        let value: Value = serde_json::from_str(trimmed).ok()?;
        match value {
            Value::Object(fields) => Some(Self { fields }),
            _ => None,
        }
    }

    fn field(&self, key: &str) -> Option<&Value> {
        self.fields.get(key)
    }

    /// Returns whether `key` is present in the message, regardless of its
    /// value (including an explicit JSON `null`), matching Python's `key in
    /// msg`.
    #[must_use]
    pub fn has_field(&self, key: &str) -> bool {
        self.field(key).is_some()
    }

    /// Reads `key` as a floating-point value, matching upstream's
    /// `float(msg[key])` for the numeric fields it reads unconditionally
    /// once guarded by `"key" in msg`.
    #[must_use]
    pub fn get_f64(&self, key: &str) -> Option<f64> {
        self.field(key).and_then(Value::as_f64)
    }

    /// Reads `key` with Python's `bool(...)` truthiness: `false`/`0`/`""`/
    /// `null`/empty array or object are falsy, everything else is truthy.
    #[must_use]
    pub fn get_bool(&self, key: &str) -> Option<bool> {
        self.field(key).map(json_truthy)
    }

    /// Reads `key` as an integer, matching upstream's `int(msg[key])`:
    /// integral JSON numbers pass through, floating-point numbers truncate
    /// toward zero.
    #[must_use]
    #[allow(
        clippy::cast_possible_truncation,
        reason = "deliberately mirrors Python's int() truncation-toward-zero for float inputs"
    )]
    pub fn get_i64(&self, key: &str) -> Option<i64> {
        self.field(key).and_then(|value| {
            value
                .as_i64()
                .or_else(|| value.as_f64().map(|f| f.trunc() as i64))
        })
    }

    /// Reads `key` as a controller/reference pose object, matching
    /// upstream's `msg.get(key)`: absent and explicit-`null` both yield
    /// [`None`].
    #[must_use]
    pub fn get_pose(&self, key: &str) -> Option<RawPose> {
        let object = self.field(key)?.as_object()?;
        Some(RawPose {
            x: object.get("x")?.as_f64()?,
            y: object.get("y")?.as_f64()?,
            z: object.get("z")?.as_f64()?,
            qx: object.get("qx")?.as_f64()?,
            qy: object.get("qy")?.as_f64()?,
            qz: object.get("qz")?.as_f64()?,
            qw: object.get("qw")?.as_f64()?,
        })
    }
}

fn json_truthy(value: &Value) -> bool {
    match value {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64().is_some_and(|f| f != 0.0),
        Value::String(s) => !s.is_empty(),
        Value::Array(a) => !a.is_empty(),
        Value::Object(o) => !o.is_empty(),
    }
}
