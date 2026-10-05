//! Commands the driver accepts from the bus.

/// Supported speech languages. Anything else triggers the canned apology.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Language {
    English,
    Spanish,
}

/// Text the robot should say.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeechCommand {
    pub language: Language,
    pub text: String,
    /// Say through ALAnimatedSpeech (with gestures) instead of plain text-to-speech.
    pub animated: bool,
}

/// Animation package installed on the robot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnimationFamily {
    /// `animations/Stand/<name>`
    Animations,
    /// `animations_sinfonia/animations/<name>`
    AnimationsSinfonia,
}

/// Animation to play through ALBehaviorManager.
#[derive(Clone, Debug, PartialEq)]
pub struct AnimationCommand {
    pub family: AnimationFamily,
    pub name: String,
}

/// Joints to move, three parallel arrays of equal length.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct JointCommand {
    pub names: Vec<String>,
    pub angles: Vec<f32>,
    pub fraction_max_speed: Vec<f32>,
}

impl JointCommand {
    pub fn len(&self) -> usize {
        self.names.len()
    }

    pub fn is_empty(&self) -> bool {
        self.names.is_empty()
    }
}

/// RGB LED command; color channels are 0–255.
#[derive(Clone, Debug, PartialEq)]
pub struct LedCommand {
    pub name: String,
    pub red: u8,
    pub green: u8,
    pub blue: u8,
    /// Fade duration in seconds.
    pub duration: f32,
}

/// Robot-wide settings, always enabled at boot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpecialSetting {
    /// `true` puts the robot at rest, `false` wakes it up.
    Rest(bool),
    ExternalCollisionProtection(bool),
    SecurityDistance(f32),
    Awareness(bool),
}

/// Request for the free-zone walk: find a circle of `desired_radius`
/// reachable with at most `displacement` meters of travel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FreeZoneRequest {
    pub desired_radius: f32,
    pub displacement: f32,
}
