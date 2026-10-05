// Copyright 2026 SinfonIA Uniandes <sinfonia@uniandes.edu.co>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Commands the driver accepts from the bus.

/// Supported speech languages.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Language {
    English,
    Spanish,
}

/// Text the robot should say.
#[derive(Clone, Debug, PartialEq)]
pub struct SpeechCommand {
    /// `None` when the requested language is unsupported; the robot then says
    /// a canned apology in its current language instead of `text`.
    pub language: Option<Language>,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joint_command_tracks_its_length() {
        let empty = JointCommand::default();
        assert!(empty.is_empty());
        let command = JointCommand {
            names: vec!["HeadYaw".to_owned()],
            angles: vec![0.1],
            fraction_max_speed: vec![0.5],
        };
        assert_eq!(command.len(), 1);
        assert!(!command.is_empty());
    }
}
