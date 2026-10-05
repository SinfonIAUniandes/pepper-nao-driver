//! Geometric primitives.

/// 2D point or vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

impl Vector2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn norm(self) -> f32 {
        self.x.hypot(self.y)
    }
}

/// 3D point or vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

/// Rotation quaternion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    /// Rotation described by roll (x), pitch (y) and yaw (z) angles.
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Self {
        let (sr, cr) = (roll * 0.5).sin_cos();
        let (sp, cp) = (pitch * 0.5).sin_cos();
        let (sy, cy) = (yaw * 0.5).sin_cos();
        Self {
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
            w: cr * cp * cy + sr * sp * sy,
        }
    }

    /// Rotation of `angle` radians about `axis`.
    pub fn from_axis_angle(axis: Vector3, angle: f32) -> Self {
        let (half_sin, half_cos) = (angle * 0.5).sin_cos();
        Self {
            x: axis.x * half_sin,
            y: axis.y * half_sin,
            z: axis.z * half_sin,
            w: half_cos,
        }
    }

    /// Hamilton product; `self` applied after `other`.
    pub fn mul(self, other: Self) -> Self {
        Self {
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        }
    }

    pub fn rotate(self, v: Vector3) -> Vector3 {
        let qv = Vector3::new(self.x, self.y, self.z);
        let uv = cross(qv, v);
        let uuv = cross(qv, uv);
        Vector3::new(
            v.x + 2.0 * (self.w * uv.x + uuv.x),
            v.y + 2.0 * (self.w * uv.y + uuv.y),
            v.z + 2.0 * (self.w * uv.z + uuv.z),
        )
    }
}

fn cross(a: Vector3, b: Vector3) -> Vector3 {
    Vector3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

/// Planar pose: position plus heading.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    pub position: Vector2,
    pub theta: f32,
}

impl Pose2 {
    pub const fn new(x: f32, y: f32, theta: f32) -> Self {
        Self {
            position: Vector2::new(x, y),
            theta,
        }
    }

    pub fn inverse(self) -> Self {
        let (sin, cos) = (-self.theta).sin_cos();
        Self {
            position: Vector2::new(
                -(self.position.x * cos - self.position.y * sin),
                -(self.position.y * cos + self.position.x * sin),
            ),
            theta: -self.theta,
        }
    }

    /// Composition `self ∘ other`.
    pub fn compose(self, other: Self) -> Self {
        let (sin, cos) = self.theta.sin_cos();
        Self {
            position: Vector2::new(
                self.position.x + cos * other.position.x - sin * other.position.y,
                self.position.y + sin * other.position.x + cos * other.position.y,
            ),
            theta: self.theta + other.theta,
        }
    }
}

/// Spatial pose: position plus orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose3 {
    pub position: Vector3,
    pub orientation: Quaternion,
}

impl Default for Pose3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Pose3 {
    pub const IDENTITY: Self = Self {
        position: Vector3::new(0.0, 0.0, 0.0),
        orientation: Quaternion::IDENTITY,
    };

    /// Composition `self ∘ other`.
    pub fn compose(self, other: Self) -> Self {
        Self {
            position: Vector3::new(
                self.position.x + self.orientation.rotate(other.position).x,
                self.position.y + self.orientation.rotate(other.position).y,
                self.position.z + self.orientation.rotate(other.position).z,
            ),
            orientation: self.orientation.mul(other.orientation),
        }
    }

    pub fn inverse(self) -> Self {
        let inv_orientation = Quaternion {
            x: -self.orientation.x,
            y: -self.orientation.y,
            z: -self.orientation.z,
            w: self.orientation.w,
        };
        Self {
            position: inv_orientation.rotate(self.position * -1.0),
            orientation: inv_orientation,
        }
    }
}

impl std::ops::Mul<f32> for Vector3 {
    type Output = Vector3;

    fn mul(self, rhs: f32) -> Vector3 {
        Vector3::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

/// Velocity of the base: `vx`, `vy` (m/s) and `wz` (rad/s).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Twist {
    pub vx: f32,
    pub vy: f32,
    pub wz: f32,
}

/// Named rigid transform between two frames, as published on `tf`.
#[derive(Clone, Debug, PartialEq)]
pub struct Transform {
    pub parent: String,
    pub child: String,
    pub translation: Vector3,
    pub rotation: Quaternion,
}

impl Transform {
    /// The pose of `child` in `parent`.
    pub fn pose(&self) -> Pose3 {
        Pose3 {
            position: self.translation,
            orientation: self.rotation,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    fn assert_close(a: f32, b: f32) {
        assert!((a - b).abs() < EPS, "{a} != {b}");
    }

    #[test]
    fn euler_yaw_is_a_rotation_about_z() {
        let q = Quaternion::from_euler(0.0, 0.0, std::f32::consts::FRAC_PI_2);
        let v = q.rotate(Vector3::new(1.0, 0.0, 0.0));
        assert_close(v.x, 0.0);
        assert_close(v.y, 1.0);
        assert_close(v.z, 0.0);
    }

    #[test]
    fn axis_angle_matches_euler() {
        let q = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), 0.7);
        let e = Quaternion::from_euler(0.0, 0.0, 0.7);
        assert_close(q.x, e.x);
        assert_close(q.y, e.y);
        assert_close(q.z, e.z);
        assert_close(q.w, e.w);
    }

    #[test]
    fn pose2_inverse_composes_to_identity() {
        let pose = Pose2::new(1.0, -2.0, 0.6);
        let round_trip = pose.compose(pose.inverse());
        assert_close(round_trip.position.x, 0.0);
        assert_close(round_trip.position.y, 0.0);
        assert_close(round_trip.theta, 0.0);
    }

    #[test]
    fn pose2_compose_translates_in_parent_frame() {
        let result = Pose2::new(1.0, 0.0, std::f32::consts::FRAC_PI_2).compose(Pose2::new(1.0, 0.0, 0.0));
        assert_close(result.position.x, 1.0);
        assert_close(result.position.y, 1.0);
    }

    #[test]
    fn pose3_compose_matches_manual_rotation() {
        let half_turn = Pose3 {
            position: Vector3::new(1.0, 0.0, 0.0),
            orientation: Quaternion::from_euler(0.0, 0.0, std::f32::consts::PI),
        };
        let child = Pose3 {
            position: Vector3::new(1.0, 0.0, 0.0),
            orientation: Quaternion::IDENTITY,
        };
        let composed = half_turn.compose(child);
        assert_close(composed.position.x, 0.0);
        assert_close(composed.position.y, 0.0);

        let round_trip = composed.compose(composed.inverse());
        assert_close(round_trip.position.x, 0.0);
        assert_close(round_trip.position.y, 0.0);
        assert_close(round_trip.orientation.w, 1.0);
    }
}
