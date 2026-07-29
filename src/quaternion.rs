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

//! Quaternion math matching `scipy.spatial.transform.Rotation` conventions.
//!
//! Quaternions are stored scalar-last (`x, y, z, w`), the same layout scipy
//! uses for `from_quat`/`as_quat`. Composition follows scipy's `Rotation.*`
//! operator: for `p = r1 * r2`, `p.apply(v) == r1.apply(r2.apply(v))`. All
//! arithmetic is `f64`, matching scipy's internal representation regardless
//! of the input array dtype.

/// A quaternion in scalar-last `(x, y, z, w)` order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// The `x` (i) component of the vector part.
    pub x: f64,
    /// The `y` (j) component of the vector part.
    pub y: f64,
    /// The `z` (k) component of the vector part.
    pub z: f64,
    /// The scalar (real) component.
    pub w: f64,
}

impl Quaternion {
    /// Builds a quaternion from scalar-last components.
    #[must_use]
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    /// Builds a quaternion from a scalar-last `[x, y, z, w]` array, matching
    /// `scipy.spatial.transform.Rotation.from_quat`'s input layout.
    #[must_use]
    pub fn from_xyzw(xyzw: [f64; 4]) -> Self {
        Self::new(xyzw[0], xyzw[1], xyzw[2], xyzw[3])
    }

    /// Returns the scalar-last `[x, y, z, w]` array, matching
    /// `scipy.spatial.transform.Rotation.as_quat`'s output layout.
    #[must_use]
    pub fn to_xyzw(self) -> [f64; 4] {
        [self.x, self.y, self.z, self.w]
    }

    /// Returns a unit quaternion pointing in the same direction as `self`.
    ///
    /// Matches `Rotation.from_quat`'s default `normalize=True` behavior.
    #[must_use]
    pub fn normalize(self) -> Self {
        let norm = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        Self::new(self.x / norm, self.y / norm, self.z / norm, self.w / norm)
    }

    /// Returns the conjugate `(-x, -y, -z, w)`, the inverse for unit
    /// quaternions. Matches `Rotation.inv()`.
    #[must_use]
    pub fn conjugate(self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.w)
    }

    /// The Hamilton product `self ⊗ other`.
    ///
    /// For rotations, this represents composing `self` after `other`:
    /// `self.hamilton_product(other)` corresponds to scipy's `self_rot * other_rot`.
    #[must_use]
    pub fn hamilton_product(self, other: Self) -> Self {
        let (x1, y1, z1, w1) = (self.x, self.y, self.z, self.w);
        let (x2, y2, z2, w2) = (other.x, other.y, other.z, other.w);
        Self::new(
            w1 * x2 + w2 * x1 + (y1 * z2 - z1 * y2),
            w1 * y2 + w2 * y1 + (z1 * x2 - x1 * z2),
            w1 * z2 + w2 * z1 + (x1 * y2 - y1 * x2),
            w1 * w2 - (x1 * x2 + y1 * y2 + z1 * z2),
        )
    }

    /// Rotates the vector `v` by this quaternion, assuming `self` is a unit
    /// quaternion. Matches `Rotation.apply(v)`.
    #[must_use]
    pub fn rotate_vector(self, v: [f64; 3]) -> [f64; 3] {
        let v_quat = Self::new(v[0], v[1], v[2], 0.0);
        let rotated = self
            .hamilton_product(v_quat)
            .hamilton_product(self.conjugate());
        [rotated.x, rotated.y, rotated.z]
    }

    /// Builds the quaternion for a right-handed rotation of `degrees` about
    /// the Z axis, matching `Rotation.from_euler("z", degrees, degrees=True)`.
    #[must_use]
    pub fn from_z_rotation_degrees(degrees: f64) -> Self {
        let half_angle = degrees.to_radians() / 2.0;
        Self::new(0.0, 0.0, half_angle.sin(), half_angle.cos())
    }
}
