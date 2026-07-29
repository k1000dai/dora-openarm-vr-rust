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

//! One Euro Filter based pose smoother, matching upstream `smoothing.py`.
//!
//! All state and arithmetic use `f64`. Upstream's `self.dp_prev =
//! np.zeros(3)` defaults to a `float64` numpy array, and mixing that into
//! the filter recurrence upgrades position, quaternion and velocity state
//! to `float64` from the second `smooth()` call onward under numpy 2.x's
//! type promotion rules, even though the input/output poses are `float32`.

// The `as f32` narrowing at the end of `smooth()` is the deliberate final
// step of that dtype ladder, matching upstream's explicit
// `dtype=np.float32` cast in its own return statement.
#![allow(clippy::cast_possible_truncation)]

/// Spherical linear interpolation between two quaternions (scalar-last
/// `[x, y, z, w]`), matching upstream `_slerp_quat`.
#[must_use]
pub fn slerp_quat(q1: [f64; 4], q2: [f64; 4], alpha: f64) -> [f64; 4] {
    let dot_raw = dot4(q1, q2);
    let (q2, dot) = if dot_raw < 0.0 {
        (neg4(q2), -dot_raw)
    } else {
        (q2, dot_raw)
    };

    if dot > 0.9995 {
        let res = [
            q1[0] + alpha * (q2[0] - q1[0]),
            q1[1] + alpha * (q2[1] - q1[1]),
            q1[2] + alpha * (q2[2] - q1[2]),
            q1[3] + alpha * (q2[3] - q1[3]),
        ];
        let norm = (res[0] * res[0] + res[1] * res[1] + res[2] * res[2] + res[3] * res[3]).sqrt();
        return [res[0] / norm, res[1] / norm, res[2] / norm, res[3] / norm];
    }

    let theta_0 = dot.acos();
    let sin_theta_0 = theta_0.sin();
    let theta = theta_0 * alpha;
    let sin_theta = theta.sin();

    let s0 = theta.cos() - dot * sin_theta / sin_theta_0;
    let s1 = sin_theta / sin_theta_0;
    [
        s0 * q1[0] + s1 * q2[0],
        s0 * q1[1] + s1 * q2[1],
        s0 * q1[2] + s1 * q2[2],
        s0 * q1[3] + s1 * q2[3],
    ]
}

fn dot4(a: [f64; 4], b: [f64; 4]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn neg4(a: [f64; 4]) -> [f64; 4] {
    [-a[0], -a[1], -a[2], -a[3]]
}

fn get_alpha(dt: f64, cutoff: f64) -> f64 {
    let tau = 1.0 / (2.0 * std::f64::consts::PI * cutoff);
    1.0 / (1.0 + tau / dt)
}

/// One Euro Filter applied to position (adaptive cutoff) and rotation
/// (SLERP), matching upstream `OneEuroPoseSmoother`.
///
/// A pose is `[x, y, z, qx, qy, qz, qw]`: position in meters, quaternion
/// scalar-last.
#[derive(Debug, Clone)]
pub struct OneEuroPoseSmoother {
    min_cutoff: f64,
    beta: f64,
    d_cutoff: f64,
    p_prev: Option<[f64; 3]>,
    q_prev: Option<[f64; 4]>,
    dp_prev: [f64; 3],
    t_prev: Option<f64>,
}

impl OneEuroPoseSmoother {
    /// Builds a smoother with the given minimum cutoff frequency, speed
    /// coefficient, and derivative cutoff frequency.
    #[must_use]
    pub fn new(min_cutoff: f64, beta: f64, d_cutoff: f64) -> Self {
        Self {
            min_cutoff,
            beta,
            d_cutoff,
            p_prev: None,
            q_prev: None,
            dp_prev: [0.0; 3],
            t_prev: None,
        }
    }

    /// Clears filter state so the next sample is treated as a fresh start.
    ///
    /// Call this on an INVALID -> valid transition of the tracked pose.
    pub fn reset(&mut self) {
        self.p_prev = None;
        self.q_prev = None;
        self.dp_prev = [0.0; 3];
        self.t_prev = None;
    }

    /// Smooths `target_pose` sampled at time `t` (seconds, monotonic
    /// clock), or passes `None` through unchanged, matching upstream
    /// `smooth(self, t, target_pose)`.
    ///
    /// The first sample (or any sample where `t` doesn't advance past the
    /// previous one) passes through unchanged.
    #[must_use]
    pub fn smooth(&mut self, t: f64, target_pose: Option<[f32; 7]>) -> Option<[f32; 7]> {
        let target_pose = target_pose?;

        let t_p = [
            f64::from(target_pose[0]),
            f64::from(target_pose[1]),
            f64::from(target_pose[2]),
        ];
        let t_q = [
            f64::from(target_pose[3]),
            f64::from(target_pose[4]),
            f64::from(target_pose[5]),
            f64::from(target_pose[6]),
        ];

        let (Some(t_prev), Some(p_prev)) = (self.t_prev, self.p_prev) else {
            self.p_prev = Some(t_p);
            self.q_prev = Some(t_q);
            self.t_prev = Some(t);
            return Some(target_pose);
        };

        let dt = t - t_prev;
        if dt <= 0.0 {
            return Some(target_pose);
        }

        let dp_raw = [
            (t_p[0] - p_prev[0]) / dt,
            (t_p[1] - p_prev[1]) / dt,
            (t_p[2] - p_prev[2]) / dt,
        ];
        let alpha_d = get_alpha(dt, self.d_cutoff);
        let dp_filtered = [
            alpha_d * dp_raw[0] + (1.0 - alpha_d) * self.dp_prev[0],
            alpha_d * dp_raw[1] + (1.0 - alpha_d) * self.dp_prev[1],
            alpha_d * dp_raw[2] + (1.0 - alpha_d) * self.dp_prev[2],
        ];

        let speed = (dp_filtered[0] * dp_filtered[0]
            + dp_filtered[1] * dp_filtered[1]
            + dp_filtered[2] * dp_filtered[2])
            .sqrt();
        let cutoff_p = self.min_cutoff + self.beta * speed;

        let alpha_p = get_alpha(dt, cutoff_p);
        let p_hat = [
            p_prev[0] + alpha_p * (t_p[0] - p_prev[0]),
            p_prev[1] + alpha_p * (t_p[1] - p_prev[1]),
            p_prev[2] + alpha_p * (t_p[2] - p_prev[2]),
        ];
        let q_hat = slerp_quat(self.q_prev.unwrap_or(t_q), t_q, alpha_p);

        self.p_prev = Some(p_hat);
        self.q_prev = Some(q_hat);
        self.dp_prev = dp_filtered;
        self.t_prev = Some(t);

        Some([
            p_hat[0] as f32,
            p_hat[1] as f32,
            p_hat[2] as f32,
            q_hat[0] as f32,
            q_hat[1] as f32,
            q_hat[2] as f32,
            q_hat[3] as f32,
        ])
    }
}
