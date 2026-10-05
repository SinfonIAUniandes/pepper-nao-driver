//! Messages flowing between the driver and the transport.

use super::{
    AnimationCommand, AudioBuffer, FaceSet, FreeZoneRequest, ImageFrame, JointCommand, LaserScan,
    LedCommand, Odometry, Path, Pose2, Pose3, Range, SoundBearing, SpecialSetting, SpeechCommand,
    Touch, Transform, Twist,
};

/// Domain message travelling over the transport, in both directions.
///
/// Output capabilities publish these, input capabilities consume them. Which
/// topics are used is documented on [`crate::transport::Transport`].
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    // Outputs
    /// `tf`: frame updates, one per child frame.
    Transforms(Vec<Transform>),
    /// `odom`, `merged_laser` poses.
    Odometry(Odometry),
    /// `laser`, `depth_to_laser`, `merged_laser`.
    LaserScan(LaserScan),
    /// `sonar`, one message per head.
    Range(Range),
    /// `front_camera`, `bottom_camera`, `depth_camera`.
    Image(ImageFrame),
    /// `front_camera_face_detector`, `bottom_camera_face_detector`.
    Faces(FaceSet),
    /// `mic`.
    Audio(AudioBuffer),
    /// `miclocalization`.
    SoundBearing(SoundBearing),
    /// `navigation_path`.
    Path(Path),
    /// `pose-pub`.
    Pose(Pose3),
    /// `navigation_result`.
    Text(String),
    /// `free_zone` result.
    Pose2(Pose2),
    /// `touch`.
    Touch(Touch),

    // Inputs
    /// `cmd_vel`.
    CmdVel(Twist),
    /// `moveto`.
    MoveTo(Pose2),
    /// `set_angles`.
    SetAngles(JointCommand),
    /// `animation`.
    Animation(AnimationCommand),
    /// `leds`.
    Leds(LedCommand),
    /// `special_settings`.
    SpecialSetting(SpecialSetting),
    /// `speech`.
    Speech(SpeechCommand),
    /// `navigation_goal`.
    NavigationGoal(Pose2),
    /// `pose-set`.
    PoseSet(Pose2),
    /// `free_zone`.
    FreeZone(FreeZoneRequest),
}
