//! Sensor and perception messages.

use super::{Pose3, Timestamp, Twist, Vector3};

/// Range measurement of a single sonar.
#[derive(Clone, Debug, PartialEq)]
pub struct Range {
    pub stamp: Timestamp,
    pub frame: String,
    pub field_of_view: f32,
    pub min: f32,
    pub max: f32,
    pub value: f32,
}

/// A planar laser scan.
#[derive(Clone, Debug, PartialEq)]
pub struct LaserScan {
    pub stamp: Timestamp,
    pub frame: String,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub scan_time: f32,
    pub time_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    /// Distance per beam; `-1` marks a hole.
    pub ranges: Vec<f32>,
}

/// Odometry sample of the robot base.
#[derive(Clone, Debug, PartialEq)]
pub struct Odometry {
    pub stamp: Timestamp,
    pub frame: String,
    pub child_frame: String,
    pub pose: Pose3,
    pub twist: Twist,
}

/// Sequence of world points produced by the external planner.
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    pub stamp: Timestamp,
    pub frame: String,
    pub points: Vec<Vector3>,
}

/// Color space of an image, matching the NAOqi encoding ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ColorSpace(pub i32);

impl ColorSpace {
    pub const RGB: Self = Self(11);
    pub const RAW_DEPTH: Self = Self(23);
}

/// Camera calibration for one resolution, loaded from `share/camera_info`.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraInfo {
    pub frame: String,
    pub width: u32,
    pub height: u32,
    pub distortion_model: String,
    pub d: Vec<f32>,
    pub k: [f32; 9],
    pub r: [f32; 9],
    pub p: [f32; 12],
    pub binning_x: u32,
    pub binning_y: u32,
}

/// One image delivered by ALVideoDevice.
#[derive(Clone, Debug, PartialEq)]
pub struct ImageFrame {
    pub stamp: Timestamp,
    pub frame: String,
    pub width: u32,
    pub height: u32,
    pub color_space: ColorSpace,
    pub data: Vec<u8>,
    pub camera_info: Option<CameraInfo>,
}

/// Interleaved microphone samples.
#[derive(Clone, Debug, PartialEq)]
pub struct AudioBuffer {
    pub stamp: Timestamp,
    pub frequency: u32,
    pub channel_map: Vec<u8>,
    pub data: Vec<i16>,
}

/// Direction of a localized sound source.
#[derive(Clone, Debug, PartialEq)]
pub struct SoundBearing {
    pub stamp: Timestamp,
    pub azimuth: f32,
    pub elevation: f32,
    pub confidence: f32,
    pub energy: f32,
    /// Head pose in torso frame, as reported by the event.
    pub head_in_torso: Vec<f32>,
    /// Head pose in robot frame, as reported by the event.
    pub head_in_robot: Vec<f32>,
}

/// A detected face cropped from a camera image.
#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    pub stamp: Timestamp,
    pub image: ImageFrame,
}

/// Faces detected in one camera frame.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceSet {
    pub stamp: Timestamp,
    pub camera: String,
    pub faces: Vec<Face>,
}

/// Touch sensor identifiers, spelled as in the driver API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TouchId {
    BumperRight,
    BumperLeft,
    BumperBack,
    HeadFront,
    HeadMiddle,
    HeadRear,
    HandRightBack,
    HandRightLeft,
    HandRightRight,
    HandLeftBack,
    HandLeftLeft,
    HandLeftRight,
}

impl TouchId {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::BumperRight => "bumper_right",
            Self::BumperLeft => "bumper_left",
            Self::BumperBack => "bumper_back",
            Self::HeadFront => "head_front",
            Self::HeadMiddle => "head_middle",
            Self::HeadRear => "head_rear",
            Self::HandRightBack => "hand_right_back",
            Self::HandRightLeft => "hand_right_left",
            Self::HandRightRight => "hand_right_right",
            Self::HandLeftBack => "hand_left_back",
            Self::HandLeftLeft => "hand_left_left",
            Self::HandLeftRight => "hand_left_right",
        }
    }
}

/// Touch sensor reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Touch {
    pub id: TouchId,
    pub pressed: bool,
}
