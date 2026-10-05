//! Configuration and parameter types with their validation rules.

use super::ColorSpace;

/// Camera sources, matching the ALVideoDevice source ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CameraId {
    Front,
    Bottom,
    Depth,
}

impl CameraId {
    /// NAOqi camera source index.
    pub fn source(self) -> i32 {
        match self {
            Self::Front => 0,
            Self::Bottom => 1,
            Self::Depth => 2,
        }
    }

    pub fn is_depth(self) -> bool {
        self == Self::Depth
    }
}

/// NAOqi resolution codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Resolution(pub u8);

impl Resolution {
    pub const QQVGA: Self = Self(0); // 160x120
    pub const QVGA: Self = Self(1); // 320x240
    pub const VGA: Self = Self(2); // 640x480
    pub const FOUR_VGA: Self = Self(3); // 1280x960
    pub const SIXTEEN_VGA: Self = Self(4); // 2560x1920
    pub const QQQVGA: Self = Self(7); // 80x60
    pub const QQQQVGA: Self = Self(8); // 40x30

    /// Key of this resolution inside the camera info JSON files.
    pub fn camera_info_key(self) -> &'static str {
        match self {
            Self::QQVGA => "kQQVGA",
            Self::QVGA => "kQVGA",
            Self::VGA => "kVGA",
            Self::FOUR_VGA => "k4VGA",
            Self::SIXTEEN_VGA => "k16VGA",
            Self::QQQVGA => "kQQQVGA",
            Self::QQQQVGA => "kQQQQVGA",
            _ => "kUnsupported",
        }
    }
}

/// Streaming setup of one camera.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraConfig {
    pub resolution: Resolution,
    pub color_space: ColorSpace,
    pub fps: u8,
    /// Embed a JPEG copy of the image when publishing.
    pub compress: bool,
    pub jpeg_quality: u8,
}

impl CameraConfig {
    pub const DEFAULT_FPS: u8 = 10;

    /// Default setup of a color camera: QVGA RGB at 10 Hz.
    pub fn color_default() -> Self {
        Self {
            resolution: Resolution::QVGA,
            color_space: ColorSpace::RGB,
            fps: Self::DEFAULT_FPS,
            compress: false,
            jpeg_quality: 97,
        }
    }

    /// Default setup of the depth camera: QVGA raw depth at 10 Hz.
    pub fn depth_default() -> Self {
        Self {
            resolution: Resolution::QVGA,
            color_space: ColorSpace::RAW_DEPTH,
            fps: Self::DEFAULT_FPS,
            compress: false,
            jpeg_quality: 97,
        }
    }

    /// Rejects configurations NAOqi would refuse, per camera family.
    pub fn validate(self, camera: CameraId) -> crate::Result<()> {
        let invalid = |detail: &str| crate::Error::invalid("camera config", detail);
        let res_ok = matches!(
            self.resolution,
            Resolution::QQVGA
                | Resolution::QVGA
                | Resolution::VGA
                | Resolution::FOUR_VGA
                | Resolution::SIXTEEN_VGA
                | Resolution::QQQVGA
                | Resolution::QQQQVGA
        );
        if !res_ok {
            return Err(invalid(&format!("unknown resolution {}", self.resolution.0)));
        }
        if camera.is_depth() {
            let res_ok = matches!(
                self.resolution,
                Resolution::QQVGA | Resolution::QVGA | Resolution::QQQVGA | Resolution::QQQQVGA
            );
            if !res_ok {
                return Err(invalid("depth camera supports resolutions 0, 1, 7 and 8"));
            }
            if !(1..=20).contains(&self.fps) {
                return Err(invalid("depth camera fps must be within 1..=20"));
            }
            if ![0, 11, 17, 19, 21, 23].contains(&self.color_space.0) {
                return Err(invalid("unsupported depth color space"));
            }
        } else {
            if !(1..=30).contains(&self.fps) {
                return Err(invalid("fps must be within 1..=30"));
            }
            if matches!(
                self.resolution,
                Resolution::FOUR_VGA | Resolution::SIXTEEN_VGA
            ) && self.fps != 1
            {
                return Err(invalid("4VGA and 16VGA only run at 1 fps"));
            }
            if self.color_space.0 > 16 {
                return Err(invalid("color space must be <= 16 for RGB cameras"));
            }
        }
        if !(1..=100).contains(&self.jpeg_quality) {
            return Err(invalid("jpeg quality must be within 1..=100"));
        }
        Ok(())
    }
}

/// NAOqi camera parameter ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraParam {
    Brightness,
    Contrast,
    Saturation,
    Hue,
    Gain,
    HorizontalFlip,
    VerticalFlip,
    AutoExposure,
    AutoWhiteBalance,
    AutoGain,
    Resolution,
    Fps,
    Exposure,
    SetDefault,
    BlcBlue,
    BlcRed,
    BlcGreen,
    AverageLuminance,
    AutoFocus,
}

impl CameraParam {
    pub fn id(self) -> i32 {
        match self {
            Self::Brightness => 0,
            Self::Contrast => 1,
            Self::Saturation => 2,
            Self::Hue => 3,
            Self::Gain => 6,
            Self::HorizontalFlip => 7,
            Self::VerticalFlip => 8,
            Self::AutoExposure => 11,
            Self::AutoWhiteBalance => 12,
            Self::AutoGain => 13,
            Self::Resolution => 14,
            Self::Fps => 15,
            Self::Exposure => 17,
            Self::SetDefault => 19,
            Self::BlcBlue => 29,
            Self::BlcRed => 30,
            Self::BlcGreen => 31,
            Self::AverageLuminance => 39,
            Self::AutoFocus => 40,
        }
    }
}

/// Camera register values. `None` means "leave alone" when setting and
/// "not requested" when reading.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CameraParams {
    pub brightness: Option<i32>,
    pub contrast: Option<i32>,
    pub saturation: Option<i32>,
    pub hue: Option<i32>,
    pub gain: Option<i32>,
    pub exposure: Option<i32>,
    pub horizontal_flip: Option<bool>,
    pub vertical_flip: Option<bool>,
    pub auto_exposure: Option<bool>,
    pub auto_white_balance: Option<bool>,
    pub auto_gain: Option<bool>,
    pub auto_focus: Option<bool>,
    pub average_luminance: Option<i32>,
    pub blc_blue: Option<i32>,
    pub blc_red: Option<i32>,
    pub blc_green: Option<i32>,
}

impl CameraParams {
    /// The set parameters as `(id, value)` pairs, ready for ALVideoDevice.
    pub fn as_pairs(self) -> Vec<(CameraParam, i32)> {
        let pairs = [
            (CameraParam::Brightness, self.brightness),
            (CameraParam::Contrast, self.contrast),
            (CameraParam::Saturation, self.saturation),
            (CameraParam::Hue, self.hue),
            (CameraParam::Gain, self.gain),
            (CameraParam::Exposure, self.exposure),
            (CameraParam::HorizontalFlip, self.horizontal_flip.map(i32::from)),
            (CameraParam::VerticalFlip, self.vertical_flip.map(i32::from)),
            (CameraParam::AutoExposure, self.auto_exposure.map(i32::from)),
            (
                CameraParam::AutoWhiteBalance,
                self.auto_white_balance.map(i32::from),
            ),
            (CameraParam::AutoGain, self.auto_gain.map(i32::from)),
            (CameraParam::AutoFocus, self.auto_focus.map(i32::from)),
            (CameraParam::AverageLuminance, self.average_luminance),
            (CameraParam::BlcBlue, self.blc_blue),
            (CameraParam::BlcRed, self.blc_red),
            (CameraParam::BlcGreen, self.blc_green),
        ];
        pairs
            .into_iter()
            .filter_map(|(param, value)| value.map(|v| (param, v)))
            .collect()
    }
}

/// Text-to-speech voice parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpeechParams {
    /// `0` disables the effect, otherwise within `1.0..=4.0`.
    pub pitch_shift: f32,
    /// `0` disables the effect, otherwise within `1.0..=4.0`.
    pub double_voice: f32,
    pub double_voice_level: f32,
    pub double_voice_time_shift: f32,
    pub speed: f32,
}

impl SpeechParams {
    /// Defaults per language: pitch 1.17 / speed 100 for English,
    /// pitch 1.25 / speed 90 for Spanish.
    pub fn defaults(language: super::Language) -> Self {
        let (pitch_shift, speed) = match language {
            super::Language::English => (1.17, 100.0),
            super::Language::Spanish => (1.25, 90.0),
        };
        Self {
            pitch_shift,
            double_voice: 0.0,
            double_voice_level: 0.0,
            double_voice_time_shift: 0.0,
            speed,
        }
    }

    /// Enforces the ALTextToSpeech parameter bounds.
    pub fn validate(self) -> crate::Result<()> {
        let invalid = |detail: &str| crate::Error::invalid("speech params", detail);
        let effect = |v: f32| v == 0.0 || (1.0..=4.0).contains(&v);
        if !effect(self.pitch_shift) {
            return Err(invalid("pitch_shift must be 0 or within 1.0..=4.0"));
        }
        if !effect(self.double_voice) {
            return Err(invalid("double_voice must be 0 or within 1.0..=4.0"));
        }
        if !(0.0..=4.0).contains(&self.double_voice_level) {
            return Err(invalid("double_voice_level must be within 0..=4"));
        }
        if !(0.0..=0.5).contains(&self.double_voice_time_shift) {
            return Err(invalid("double_voice_time_shift must be within 0..=0.5"));
        }
        if !(50.0..=400.0).contains(&self.speed) {
            return Err(invalid("speed must be within 50..=400"));
        }
        Ok(())
    }
}

/// Microphone setup.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MicConfig {
    /// 16000 or 48000 Hz.
    pub frequency: u16,
    /// `0` all, `1` left, `2` right, `3` front, `4` rear.
    pub channels: u8,
}

impl MicConfig {
    pub const DEFAULT: Self = Self {
        frequency: 48_000,
        channels: 0,
    };

    pub fn validate(self) -> crate::Result<()> {
        if ![16_000, 48_000].contains(&self.frequency) {
            return Err(crate::Error::invalid(
                "mic config",
                "frequency must be 16000 or 48000",
            ));
        }
        if self.channels > 4 {
            return Err(crate::Error::invalid(
                "mic config",
                "channels must be within 0..=4",
            ));
        }
        Ok(())
    }
}

/// Depth-to-laser setup, forwarded by the `custom` control command.
///
/// The driver does not compute ranges from the depth image; the external
/// `NAOqiDepth2Laser` module does. `scan_time` and `range_min` only shape the
/// published scan; `resolution`, `range_max` and `scan_height` are accepted for
/// API compatibility.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthToLaserParams {
    pub resolution: u8,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub scan_height: f32,
}

impl Default for DepthToLaserParams {
    fn default() -> Self {
        Self {
            resolution: 1,
            scan_time: 0.01,
            range_min: 0.1,
            range_max: 5.0,
            scan_height: 0.05,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn color_camera_rejects_high_color_space_and_odd_fps() {
        let config = CameraConfig {
            color_space: ColorSpace(23),
            ..CameraConfig::color_default()
        };
        assert!(config.validate(CameraId::Front).is_err());

        let config = CameraConfig {
            resolution: Resolution::FOUR_VGA,
            fps: 10,
            ..CameraConfig::color_default()
        };
        assert!(config.validate(CameraId::Front).is_err());

        let config = CameraConfig {
            resolution: Resolution::FOUR_VGA,
            fps: 1,
            ..CameraConfig::color_default()
        };
        assert!(config.validate(CameraId::Front).is_ok());
    }

    #[test]
    fn depth_camera_accepts_raw_depth_only() {
        assert!(CameraConfig::depth_default().validate(CameraId::Depth).is_ok());
        let config = CameraConfig {
            color_space: ColorSpace::RGB,
            ..CameraConfig::depth_default()
        };
        assert!(config.validate(CameraId::Depth).is_ok());
        let config = CameraConfig {
            fps: 30,
            ..CameraConfig::depth_default()
        };
        assert!(config.validate(CameraId::Depth).is_err());
        let config = CameraConfig {
            resolution: Resolution::VGA,
            ..CameraConfig::depth_default()
        };
        assert!(config.validate(CameraId::Depth).is_err());
    }

    #[test]
    fn camera_params_serialize_only_the_set_fields() {
        let params = CameraParams {
            brightness: Some(10),
            gain: Some(32),
            ..Default::default()
        };
        let pairs = params.as_pairs();
        assert_eq!(
            pairs,
            vec![(CameraParam::Brightness, 10), (CameraParam::Gain, 32)]
        );
    }

    #[test]
    fn speech_params_bounds() {
        assert!(SpeechParams::defaults(super::super::Language::English)
            .validate()
            .is_ok());
        let bad = SpeechParams {
            pitch_shift: 0.5,
            ..SpeechParams::defaults(super::super::Language::Spanish)
        };
        assert!(bad.validate().is_err());
        let bad = SpeechParams {
            speed: 450.0,
            ..SpeechParams::defaults(super::super::Language::Spanish)
        };
        assert!(bad.validate().is_err());
    }

    #[test]
    fn mic_config_bounds() {
        assert!(MicConfig::DEFAULT.validate().is_ok());
        assert!(MicConfig {
            frequency: 22_050,
            ..MicConfig::DEFAULT
        }
        .validate()
        .is_err());
        assert!(MicConfig {
            channels: 5,
            ..MicConfig::DEFAULT
        }
        .validate()
        .is_err());
    }
}
