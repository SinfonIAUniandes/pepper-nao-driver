//! Control RPC payloads, one entry per tool group.

use super::{CameraConfig, CameraId, CameraParams, DepthToLaserParams, MicConfig, SpeechParams};

/// Control request, as delivered by the transport RPC channel.
#[derive(Clone, Debug, PartialEq)]
pub enum ControlRequest {
    Navigation(NavigationCommand),
    Vision(VisionTools),
    Audio(AudioCommand),
    Motion(MotionCommand),
    Misc(MiscCommand),
    SpeechRecognition(SpeechRecognitionRequest),
}

/// Control response; the call always succeeds on the channel and reports
/// failures in `result`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ControlResponse {
    pub result: String,
    /// Payload of `get_parameters` / `get_speech_params` commands.
    pub params: Option<ControlParams>,
}

impl ControlResponse {
    pub fn ok() -> Self {
        Self {
            result: "ok".to_owned(),
            params: None,
        }
    }

    pub fn error(result: impl std::fmt::Display) -> Self {
        Self {
            result: result.to_string(),
            params: None,
        }
    }
}

/// Parameter payloads returned by control commands.
#[derive(Clone, Debug, PartialEq)]
pub enum ControlParams {
    Camera(CameraParams),
    Speech(SpeechParams),
}

/// Commands of `navigation_tools`.
#[derive(Clone, Debug, PartialEq)]
pub enum NavigationCommand {
    EnableMapper,
    DisableMapper,
    EnableNavigate,
    DisableNavigate,
    EnableAll,
    DisableAll,
    Custom(NavigationCustom),
}

/// `navigation_tools custom`: independent enable flag per functionality.
#[derive(Clone, Debug, PartialEq)]
pub struct NavigationCustom {
    pub tf: FrequencySetting,
    pub odom: FrequencySetting,
    pub laser: FrequencySetting,
    pub path: FrequencySetting,
    pub pose_pub: FrequencySetting,
    pub cmd_vel: CmdVelSetting,
    pub depth_to_laser: DepthToLaserSetting,
    pub moveto: bool,
    pub goal: bool,
    pub pose_set: bool,
    pub result: bool,
    pub free_zone: bool,
}

/// Enable a periodic capability at `frequency`, or disable it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrequencySetting {
    pub enable: bool,
    pub frequency: f32,
}

/// Enable `cmd_vel` with its watchdog timeout, or disable it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CmdVelSetting {
    pub enable: bool,
    /// Watchdog timeout in seconds; `<= 0` disables the watchdog.
    pub security_timer: f32,
}

/// Enable `depth_to_laser` with its parameters, or disable it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthToLaserSetting {
    pub enable: bool,
    pub params: DepthToLaserParams,
}

/// Commands of `vision_tools`.
#[derive(Clone, Debug, PartialEq)]
pub struct VisionTools {
    pub camera: CameraTarget,
    pub command: VisionCommand,
}

/// Vision targets: the three cameras and the two face detectors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraTarget {
    FrontCamera,
    BottomCamera,
    DepthCamera,
    FrontCameraFaceDetector,
    BottomCameraFaceDetector,
}

impl CameraTarget {
    /// The camera feeding this target, if it is a camera.
    pub fn camera(self) -> Option<CameraId> {
        match self {
            Self::FrontCamera | Self::FrontCameraFaceDetector => Some(CameraId::Front),
            Self::BottomCamera | Self::BottomCameraFaceDetector => Some(CameraId::Bottom),
            Self::DepthCamera => Some(CameraId::Depth),
        }
    }

    pub fn is_face_detector(self) -> bool {
        matches!(
            self,
            Self::FrontCameraFaceDetector | Self::BottomCameraFaceDetector
        )
    }
}

/// Commands of `vision_tools`, per target.
#[derive(Clone, Debug, PartialEq)]
pub enum VisionCommand {
    Enable,
    Disable,
    Custom(CameraConfig, CameraParams),
    SetParameters(CameraParams),
    /// Reads the parameters flagged in the given value.
    GetParameters(CameraParams),
}

/// Commands of `audio_tools`.
#[derive(Clone, Debug, PartialEq)]
pub enum AudioCommand {
    Enable,
    Disable,
    EnableMic,
    DisableMic,
    Custom(MicConfig),
    EnableTts,
    DisableTts,
    EnableLocalization,
    DisableLocalization,
    GetSpeechParams,
    SetSpeechParams(SpeechParams),
    ResetSpeechParams,
}

/// Commands of `motion_tools`.
#[derive(Clone, Debug, PartialEq)]
pub enum MotionCommand {
    EnableAll,
    DisableAll,
    Custom {
        animation: Switch,
        set_angles: Switch,
    },
}

/// Commands of `misc_tools`.
#[derive(Clone, Debug, PartialEq)]
pub enum MiscCommand {
    EnableAll,
    DisableAll,
    Custom {
        leds: Switch,
        sonars: Switch,
        touch: Switch,
    },
}

/// Tri-state per-field setting of `custom` commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Switch {
    Enable,
    Disable,
    /// Leave the current state unchanged.
    Keep,
}

/// Request of the `speech_recognition` RPC.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeechRecognitionRequest {
    pub words: Vec<String>,
    pub threshold: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_response_reports_errors_in_the_result_string() {
        assert_eq!(ControlResponse::ok().result, "ok");
        assert!(ControlResponse::error("boom").result.contains("boom"));
    }

    #[test]
    fn face_detectors_stay_bound_to_their_camera() {
        assert_eq!(
            CameraTarget::FrontCameraFaceDetector.camera(),
            Some(CameraId::Front)
        );
        assert!(CameraTarget::BottomCameraFaceDetector.is_face_detector());
        assert!(!CameraTarget::DepthCamera.is_face_detector());
    }
}
