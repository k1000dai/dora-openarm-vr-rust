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

//! Arrow array builders for dora outputs, matching upstream `main.py`'s
//! `pa.array(...)` payloads exactly:
//!
//! - `status`: `Utf8Array` len 1 (`"ready"`).
//! - `vr_receive_times`: `Int64Array`, one element per packet drained this
//!   tick.
//! - `button_{a,b,x,y}`: `BooleanArray` len 1.
//! - `trigger_{side}`, `grip_{side}`, `joystick_{x,y}_{side}`:
//!   `Float32Array` len 1.
//! - `pose_{right,left,reference}`: `StructArray` len 1 with one field,
//!   `pose: List<Float32>` (length 8 for `right`/`left` with the gripper
//!   angle appended, length 7 for `reference`), matching upstream's
//!   `pa.struct({"pose": pa.list_(pa.float32())})`.

use arrow::array::{
    ArrayRef, BooleanArray, Float32Array, Int64Array, ListArray, StringArray, StructArray,
};
use arrow::buffer::OffsetBuffer;
use arrow::datatypes::{DataType, Field, Fields};
use std::sync::Arc;

/// The value of one dora output, independent of how it gets encoded as
/// Arrow.
#[derive(Debug, Clone, PartialEq)]
pub enum EmissionValue {
    /// A status string (only ever `"ready"`).
    Status(&'static str),
    /// The `vr_receive_times` payload: one arrival timestamp (ns) per
    /// packet drained this tick.
    Int64Vec(Vec<i64>),
    /// A button state.
    Bool(bool),
    /// A trigger, grip, or joystick axis value.
    Float32(f32),
    /// A pose (`[x, y, z, qw, qx, qy, qz]`, optionally with a trailing
    /// gripper angle).
    Pose(Vec<f32>),
}

/// Builds the Arrow array for one [`EmissionValue`].
#[must_use]
pub fn build_array(value: EmissionValue) -> ArrayRef {
    match value {
        EmissionValue::Status(text) => Arc::new(StringArray::from(vec![text])),
        EmissionValue::Int64Vec(values) => Arc::new(Int64Array::from(values)),
        EmissionValue::Bool(v) => Arc::new(BooleanArray::from(vec![v])),
        EmissionValue::Float32(v) => Arc::new(Float32Array::from(vec![v])),
        EmissionValue::Pose(values) => pose_struct(values),
    }
}

fn pose_struct(values: Vec<f32>) -> ArrayRef {
    let len = values.len();
    let item_field = Arc::new(Field::new("item", DataType::Float32, true));
    let values: ArrayRef = Arc::new(Float32Array::from(values));
    let offsets = OffsetBuffer::from_lengths([len]);
    let pose_list = ListArray::new(item_field.clone(), offsets, values, None);

    let pose_field = Arc::new(Field::new("pose", DataType::List(item_field), true));
    Arc::new(StructArray::new(
        Fields::from(vec![pose_field]),
        vec![Arc::new(pose_list)],
        None,
    ))
}
