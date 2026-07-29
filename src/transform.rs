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

//! Meta Quest (Unity, left-handed) pose -> `OpenArm` workspace transform.
//!
//! Ports `parse_lh_to_rh`, `pose_to_array`, `QuestPoseProcessor.process` and
//! `_map_trigger_to_gripper` from upstream `quest_receiver.py`. All rotation
//! math runs in `f64`, matching scipy's internal quaternion representation
//! and upstream's own `dtype=np.float64` constants; values are cast to
//! `f32` only at [`pose_to_array`], upstream's final `dtype=np.float32`
//! step.

// Every `as f32` in this module is the deliberate final narrowing step of
// that dtype ladder, matching upstream's explicit `dtype=np.float32` casts.
#![allow(clippy::cast_possible_truncation)]

use crate::quaternion::Quaternion;

/// The quaternion (scalar-last) for `_FRAME_ROT =
/// [[0,0,-1],[-1,0,0],[0,1,0]]`.
///
/// Hardcoded rather than derived via a generic matrix-to-quaternion
/// conversion: `scipy.spatial.transform.Rotation.from_matrix(...).as_quat()`
/// gives exactly `[0.5, -0.5, -0.5, 0.5]` for this matrix (verified against
/// scipy 1.17.1 / numpy 2.4.6, the versions upstream's `pyproject.toml`
/// requires), and every component is exactly representable, so using the
/// literal avoids any risk of a from-matrix algorithm picking the opposite
/// sign (`-q` represents the same rotation but would flip every composed
/// output's sign).
const FRAME_ROT_QUAT: Quaternion = Quaternion {
    x: 0.5,
    y: -0.5,
    z: -0.5,
    w: 0.5,
};

/// Neutral hand position relative to the `arm_origin` site (chest level),
/// matching upstream's `FRAME_OFFSET_NECK = np.array([-0.085, 0, -0.14],
/// dtype=np.float64)`. Unlike the frame rotation constant shared with the
/// `WebXR` port, this stays `f64` throughout: upstream never routes it
/// through a `float32` array before adding it to the rotated position.
const FRAME_OFFSET_NECK: [f64; 3] = [-0.085, 0.0, -0.14];

/// The reference pose substituted when no `rf` packet field is present,
/// matching upstream `_IDENTITY_REF`.
const IDENTITY_REF: RawPose = RawPose {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    qw: 1.0,
};

/// Which controller/arm side a value belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// The right controller / right arm.
    Right,
    /// The left controller / left arm.
    Left,
}

/// A Quest controller/reference pose as received over UDP, before any unit
/// conversion. Fields keep the upstream JSON key names; position is in
/// meters, orientation is a Unity left-handed quaternion (scalar-last).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawPose {
    /// X position in meters.
    pub x: f64,
    /// Y position in meters.
    pub y: f64,
    /// Z position in meters.
    pub z: f64,
    /// Quaternion X component.
    pub qx: f64,
    /// Quaternion Y component.
    pub qy: f64,
    /// Quaternion Z component.
    pub qz: f64,
    /// Quaternion W (scalar) component.
    pub qw: f64,
}

/// Converts a Unity left-handed pose into a right-handed `(position,
/// rotation)` pair, matching upstream `parse_lh_to_rh`.
///
/// Flip: `z -> -z`, `qx -> -qx`, `qy -> -qy`. The resulting quaternion is
/// normalized, matching `Rotation.from_quat`'s default behavior.
#[must_use]
pub fn parse_lh_to_rh(pose: &RawPose) -> ([f64; 3], Quaternion) {
    let position = [pose.x, pose.y, -pose.z];
    let rotation = Quaternion::from_xyzw([-pose.qx, -pose.qy, pose.qz, pose.qw]).normalize();
    (position, rotation)
}

/// Packs a rectified `(position, rotation)` pair into the wire pose layout
/// `[x, y, z, qw, qx, qy, qz]`, matching upstream `pose_to_array`
/// (orientation scalar-first, cast to `f32`).
#[must_use]
pub fn pose_to_array(position: [f64; 3], rotation: Quaternion) -> [f32; 7] {
    let [qx, qy, qz, qw] = rotation.to_xyzw();
    [
        position[0] as f32,
        position[1] as f32,
        position[2] as f32,
        qw as f32,
        qx as f32,
        qy as f32,
        qz as f32,
    ]
}

/// Maps a trigger value to a calibrated gripper angle in radians, matching
/// upstream `_map_trigger_to_gripper`. The trigger is clipped to `[0, 1]`
/// before interpolating between the side's open/closed angles (right:
/// `-45 deg -> 10 deg`; left: `45 deg -> -10 deg`).
#[must_use]
pub fn map_trigger_to_gripper(trigger: f64, side: Side) -> f64 {
    let trigger = trigger.clamp(0.0, 1.0);
    let (open_deg, closed_deg) = match side {
        Side::Right => (-45.0, 10.0),
        Side::Left => (45.0, -10.0),
    };
    (open_deg + trigger * (closed_deg - open_deg)).to_radians()
}

/// The result of rectifying a Quest packet's controller/reference poses
/// against its reference pose, matching upstream `QuestPoseProcessor.process`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProcessOutcome {
    /// The right controller pose, rectified into the `arm_origin` frame.
    pub pose_right: Option<[f32; 7]>,
    /// The left controller pose, rectified into the `arm_origin` frame.
    pub pose_left: Option<[f32; 7]>,
    /// The reference pose, converted to right-handed coordinates but not
    /// rectified against itself.
    pub pose_reference: Option<[f32; 7]>,
}

/// Rectifies the right/left controller poses relative to `reference` and
/// maps them into the robot's workspace frame, matching upstream
/// `QuestPoseProcessor.process`.
///
/// When `reference` is absent, an identity reference is used for
/// rectifying `right`/`left`, but `pose_reference` itself stays [`None`]
/// (upstream only emits it when the packet actually carried an `rf` field).
#[must_use]
pub fn process(
    reference: Option<RawPose>,
    right: Option<RawPose>,
    left: Option<RawPose>,
) -> ProcessOutcome {
    let (p_ref, r_ref) = parse_lh_to_rh(&reference.unwrap_or(IDENTITY_REF));
    let r_ref_inv = r_ref.conjugate();
    let r_fix = Quaternion::from_z_rotation_degrees(90.0);

    let rectify = |raw: RawPose| -> [f32; 7] {
        let (p, r) = parse_lh_to_rh(&raw);
        let relative_position =
            r_ref_inv.rotate_vector([p[0] - p_ref[0], p[1] - p_ref[1], p[2] - p_ref[2]]);
        let relative_rotation = r_ref_inv.hamilton_product(r);
        let rotated = FRAME_ROT_QUAT.rotate_vector(relative_position);
        let p_out = [
            rotated[0] + FRAME_OFFSET_NECK[0],
            rotated[1] + FRAME_OFFSET_NECK[1],
            rotated[2] + FRAME_OFFSET_NECK[2],
        ];
        let r_out = FRAME_ROT_QUAT
            .hamilton_product(relative_rotation)
            .hamilton_product(r_fix);
        pose_to_array(p_out, r_out)
    };

    ProcessOutcome {
        pose_right: right.map(rectify),
        pose_left: left.map(rectify),
        pose_reference: reference.map(|_| pose_to_array(p_ref, r_ref)),
    }
}
