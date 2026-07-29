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

//! Behavior tests for the Arrow array builders, matching upstream
//! `main.py`'s `pa.array(...)` payloads exactly.

use arrow::array::{Array, BooleanArray, Float32Array, Int64Array, ListArray, StringArray};
use dora_openarm_vr_rust::output::{EmissionValue, build_array};

#[test]
fn status_ready_is_a_length_one_utf8_array() {
    let array = build_array(EmissionValue::Status("ready"));
    let strings = array.as_any().downcast_ref::<StringArray>().unwrap();
    assert_eq!(strings.len(), 1);
    assert_eq!(strings.value(0), "ready");
}

#[test]
fn vr_receive_times_is_an_int64_array_of_arbitrary_length() {
    let array = build_array(EmissionValue::Int64Vec(vec![100, 200, 300]));
    let ints = array.as_any().downcast_ref::<Int64Array>().unwrap();
    assert_eq!(ints.values(), &[100, 200, 300]);
}

#[test]
fn float32_scalar_is_a_length_one_float32_array() {
    let array = build_array(EmissionValue::Float32(0.75));
    let floats = array.as_any().downcast_ref::<Float32Array>().unwrap();
    assert_eq!(floats.len(), 1);
    assert!((floats.value(0) - 0.75).abs() < f32::EPSILON);
}

#[test]
fn bool_scalar_is_a_length_one_boolean_array() {
    let array = build_array(EmissionValue::Bool(true));
    let bools = array.as_any().downcast_ref::<BooleanArray>().unwrap();
    assert_eq!(bools.len(), 1);
    assert!(bools.value(0));
}

#[test]
fn pose_is_a_length_one_struct_with_one_pose_field_holding_a_float32_list() {
    let pose = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
    let array = build_array(EmissionValue::Pose(pose.clone()));
    let structs = array
        .as_any()
        .downcast_ref::<arrow::array::StructArray>()
        .unwrap();
    assert_eq!(structs.len(), 1);
    assert_eq!(structs.fields().len(), 1);
    assert_eq!(structs.fields()[0].name(), "pose");

    let list = structs
        .column(0)
        .as_any()
        .downcast_ref::<ListArray>()
        .unwrap();
    let values = list
        .value(0)
        .as_any()
        .downcast_ref::<Float32Array>()
        .unwrap()
        .values()
        .to_vec();
    assert_eq!(values, pose);
}

#[test]
fn pose_supports_the_seven_element_reference_layout() {
    let pose = vec![0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0];
    let array = build_array(EmissionValue::Pose(pose.clone()));
    let structs = array
        .as_any()
        .downcast_ref::<arrow::array::StructArray>()
        .unwrap();
    let list = structs
        .column(0)
        .as_any()
        .downcast_ref::<ListArray>()
        .unwrap();
    assert_eq!(list.value_length(0), 7);
}
