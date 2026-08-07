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
//!
//! On top of the filter itself, the smoother optionally caps how fast its
//! output pose may translate and rotate (upstream's `max_linear_speed` /
//! `max_angular_speed`), and can be [suspended](OneEuroPoseSmoother::suspend)
//! -- holding the last filtered pose while clearing its motion state --
//! instead of [reset](OneEuroPoseSmoother::reset) while a pose is INVALID.

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

/// The factor that keeps a per-sample `step` within `maximum`, matching
/// upstream `_limit_scale`.
///
/// A non-positive `maximum` (the caller's limit is `0.0`, or `dt` is
/// non-positive) disables limiting, as does a non-positive `step`.
fn limit_scale(step: f64, maximum: f64) -> f64 {
    if maximum > 0.0 && step > 0.0 {
        (maximum / step).min(1.0)
    } else {
        1.0
    }
}

fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
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
    max_linear_speed: f64,
    max_angular_speed: f64,
    p_prev: Option<[f64; 3]>,
    q_prev: Option<[f64; 4]>,
    dp_prev: [f64; 3],
    t_prev: Option<f64>,
}

impl OneEuroPoseSmoother {
    /// Builds a smoother with the given minimum cutoff frequency, speed
    /// coefficient, and derivative cutoff frequency, and no output speed
    /// limiting -- upstream's `max_linear_speed=0.0, max_angular_speed=0.0`
    /// constructor defaults, which is how upstream `_run` builds
    /// `smoother_reference`.
    #[must_use]
    pub fn new(min_cutoff: f64, beta: f64, d_cutoff: f64) -> Self {
        Self::with_speed_limits(min_cutoff, beta, d_cutoff, 0.0, 0.0)
    }

    /// Builds a smoother that additionally caps how fast its *output* pose
    /// may move: `max_linear_speed` in m/s and `max_angular_speed` in rad/s,
    /// with `0.0` disabling that limit.
    ///
    /// This is how upstream `_run` builds `smoother_right`/`smoother_left`
    /// from the `--max-linear-speed`/`--max-angular-speed` options.
    #[must_use]
    pub fn with_speed_limits(
        min_cutoff: f64,
        beta: f64,
        d_cutoff: f64,
        max_linear_speed: f64,
        max_angular_speed: f64,
    ) -> Self {
        Self {
            min_cutoff,
            beta,
            d_cutoff,
            max_linear_speed,
            max_angular_speed,
            p_prev: None,
            q_prev: None,
            dp_prev: [0.0; 3],
            t_prev: None,
        }
    }

    /// Clears all filter state, so the next sample is treated as a fresh
    /// start and passes through unfiltered.
    pub fn reset(&mut self) {
        self.p_prev = None;
        self.q_prev = None;
        self.dp_prev = [0.0; 3];
        self.t_prev = None;
    }

    /// Pauses updates while preserving the last filtered pose, matching
    /// upstream `suspend`.
    ///
    /// The retained position/orientation stay put and only the velocity
    /// state is cleared, so the next accepted sample resumes filtering from
    /// the last output (over `t - t_prev`) instead of jumping straight to
    /// the new target. Call this on every tick a pose is INVALID.
    pub fn suspend(&mut self, t: f64) {
        self.dp_prev = [0.0; 3];
        self.t_prev = Some(t);
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

        let speed = norm3(dp_filtered);
        let cutoff_p = self.min_cutoff + self.beta * speed;

        let alpha_p = get_alpha(dt, cutoff_p);
        let q_prev = self.q_prev.unwrap_or(t_q);

        // The position/rotation steps the unlimited filter would take, each
        // scaled down so the output moves no faster than the configured
        // limit over this `dt`.
        let step = [
            alpha_p * (t_p[0] - p_prev[0]),
            alpha_p * (t_p[1] - p_prev[1]),
            alpha_p * (t_p[2] - p_prev[2]),
        ];
        let linear_scale = limit_scale(norm3(step), self.max_linear_speed * dt);
        let p_hat = [
            p_prev[0] + step[0] * linear_scale,
            p_prev[1] + step[1] * linear_scale,
            p_prev[2] + step[2] * linear_scale,
        ];

        let q_error_angle = 2.0 * dot4(q_prev, t_q).abs().clamp(0.0, 1.0).acos();
        let rotation_alpha =
            alpha_p * limit_scale(alpha_p * q_error_angle, self.max_angular_speed * dt);
        let q_hat = slerp_quat(q_prev, t_q, rotation_alpha);

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
