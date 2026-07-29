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

//! Behavior tests for scipy-compatible quaternion math.

use dora_openarm_vr_rust::quaternion::Quaternion;

const EPS: f64 = 1e-12;

fn assert_close4(a: [f64; 4], b: [f64; 4]) {
    for i in 0..4 {
        assert!(
            (a[i] - b[i]).abs() < EPS,
            "component {i}: {} vs {} (a={a:?}, b={b:?})",
            a[i],
            b[i]
        );
    }
}

fn assert_close3(a: [f64; 3], b: [f64; 3]) {
    for i in 0..3 {
        assert!(
            (a[i] - b[i]).abs() < EPS,
            "component {i}: {} vs {} (a={a:?}, b={b:?})",
            a[i],
            b[i]
        );
    }
}

#[test]
fn from_xyzw_round_trips_through_to_xyzw() {
    let q = Quaternion::from_xyzw([0.1, 0.2, 0.3, 0.927_361_849_549_570_4]);
    assert_close4(q.to_xyzw(), [0.1, 0.2, 0.3, 0.927_361_849_549_570_4]);
}

#[test]
fn identity_quaternion_rotates_vector_to_itself() {
    let identity = Quaternion::from_xyzw([0.0, 0.0, 0.0, 1.0]);
    assert_close3(identity.rotate_vector([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
}

#[test]
fn normalize_scales_a_non_unit_quaternion_to_unit_length() {
    let q = Quaternion::from_xyzw([0.0, 0.0, 0.0, 2.0]).normalize();
    assert_close4(q.to_xyzw(), [0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn ninety_degree_z_rotation_maps_x_axis_to_y_axis() {
    let q = Quaternion::from_z_rotation_degrees(90.0);
    assert_close3(q.rotate_vector([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
}

#[test]
fn conjugate_of_a_z_rotation_undoes_it() {
    let q = Quaternion::from_z_rotation_degrees(37.0);
    let v = [1.0, 2.0, 3.0];
    let rotated = q.rotate_vector(v);
    let back = q.conjugate().rotate_vector(rotated);
    assert_close3(back, v);
}

#[test]
fn hamilton_product_composes_two_z_rotations_by_summing_angles() {
    let a = Quaternion::from_z_rotation_degrees(30.0);
    let b = Quaternion::from_z_rotation_degrees(60.0);
    let composed = a.hamilton_product(b);
    let expected = Quaternion::from_z_rotation_degrees(90.0);
    assert_close4(composed.to_xyzw(), expected.to_xyzw());
}

#[test]
fn hamilton_product_applies_right_operand_first() {
    // p = r1 * r2 => p.apply(v) == r1.apply(r2.apply(v))
    let r1 = Quaternion::from_z_rotation_degrees(90.0);
    let r2 = Quaternion::from_xyzw([1.0, 0.0, 0.0, 0.0]); // 180 deg about X
    let v = [1.0, 0.0, 0.0];
    let composed = r1.hamilton_product(r2);
    let direct = r1.rotate_vector(r2.rotate_vector(v));
    assert_close3(composed.rotate_vector(v), direct);
}
