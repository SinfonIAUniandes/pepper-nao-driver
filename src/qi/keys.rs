//! ALMemory key names: laser and sonar sensors, external module keys and events.

/// Laser segment XY keys, in the order the scan conversion expects them:
/// right segments 01..15, front segments 01..15, left segments 01..15, each
/// interleaving X and Y.
pub fn laser_keys() -> Vec<String> {
    let mut keys = Vec::with_capacity(90);
    for side in ["Right", "Front", "Left"] {
        for segment in 1..=15 {
            for axis in ['X', 'Y'] {
                keys.push(format!(
                    "Device/SubDeviceList/Platform/LaserSensor/{side}/Horizontal/Seg{segment:02}/{axis}/Sensor/Value"
                ));
            }
        }
    }
    keys
}

/// Front sonar range key.
pub const SONAR_FRONT: &str = "Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value";
/// Rear sonar range key.
pub const SONAR_BACK: &str = "Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value";

/// Path computed by the external planner: floats in groups of three.
pub const PLANNER_PATH: &str = "NAOqiPlanner/Path";
/// Pose reported by the external localizer: three floats.
pub const LOCALIZER_ROBOT_POSE: &str = "NAOqiLocalizer/RobotPose";
/// Ranges computed by the external depth-to-laser module.
pub const DEPTH2LASER_RANGES: &str = "NAOqiDepth2Laser/Ranges";
pub const DEPTH2LASER_MIN_ANGLE: &str = "NAOqiDepth2Laser/MinAngle";
pub const DEPTH2LASER_MAX_ANGLE: &str = "NAOqiDepth2Laser/MaxAngle";
pub const DEPTH2LASER_NUM_RANGES: &str = "NAOqiDepth2Laser/NumRanges";
pub const DEPTH2LASER_MAX_RANGE: &str = "NAOqiDepth2Laser/MaxRange";

/// Goal raised to the external planner: `[x, y, theta]`.
pub const PLANNER_GOAL: &str = "NAOqiPlanner/Goal";
/// Pose raised to the external localizer: `[x, y, theta]`.
pub const LOCALIZER_SET_POSE: &str = "NAOqiLocalizer/SetPose";

/// Planner outcome event.
pub const PLANNER_RESULT: &str = "NAOqiPlanner/Result";
/// Face detection event.
pub const FACE_DETECTED: &str = "FaceDetected";
/// Word spotting event.
pub const WORD_RECOGNIZED: &str = "WordRecognized";
/// Sound localization event.
pub const SOUND_LOCATED: &str = "ALSoundLocalization/SoundLocated";

/// Touch events, mapped to their domain ids.
pub const TOUCH_EVENTS: [(&str, crate::domain::TouchId); 12] = [
    ("RightBumperPressed", crate::domain::TouchId::BumperRight),
    ("LeftBumperPressed", crate::domain::TouchId::BumperLeft),
    ("BackBumperPressed", crate::domain::TouchId::BumperBack),
    ("FrontTactilTouched", crate::domain::TouchId::HeadFront),
    ("MiddleTactilTouched", crate::domain::TouchId::HeadMiddle),
    ("RearTactilTouched", crate::domain::TouchId::HeadRear),
    ("HandRightBackTouched", crate::domain::TouchId::HandRightBack),
    ("HandRightLeftTouched", crate::domain::TouchId::HandRightLeft),
    ("HandRightRightTouched", crate::domain::TouchId::HandRightRight),
    ("HandLeftBackTouched", crate::domain::TouchId::HandLeftBack),
    ("HandLeftLeftTouched", crate::domain::TouchId::HandLeftLeft),
    ("HandLeftRightTouched", crate::domain::TouchId::HandLeftRight),
];

/// Service name under which a callback object for `key` is registered.
pub fn callback_service_name(key: &str) -> String {
    format!("ROS-Driver{key}")
}

/// Service name of the microphone callback object.
pub const AUDIO_CALLBACK_SERVICE: &str = "ROS-Driver-Audio";

/// Client name used for ALSpeechRecognition subscriptions.
pub const SPEECH_RECOGNITION_CLIENT: &str = "ROSDriverAudioWordRecognized";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn laser_keys_follow_the_documented_pattern() {
        let keys = laser_keys();
        assert_eq!(keys.len(), 90);
        assert_eq!(
            keys[0],
            "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value"
        );
        assert_eq!(
            keys[29],
            "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/Y/Sensor/Value"
        );
        assert_eq!(
            keys[30],
            "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value"
        );
        assert_eq!(
            keys[89],
            "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/Y/Sensor/Value"
        );
    }

    #[test]
    fn touch_events_map_to_unique_ids() {
        let mut ids: Vec<_> = TOUCH_EVENTS.iter().map(|(_, id)| *id).collect();
        ids.sort_by_key(|id| id.as_str());
        ids.dedup();
        assert_eq!(ids.len(), TOUCH_EVENTS.len());
    }
}
