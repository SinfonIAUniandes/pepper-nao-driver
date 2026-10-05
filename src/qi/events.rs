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

//! Decoding of ALMemory event payloads into domain values.

use super::value::{as_bytes, as_f32, as_f32s, as_i32, as_text, plain};
use crate::domain::{SoundBearing, Timestamp, Touch, TouchId};
use qi::value::Value;

/// Word with its recognition confidence.
#[derive(Clone, Debug, PartialEq)]
pub struct WordRecognized {
    pub words: Vec<(String, f32)>,
}

impl WordRecognized {
    /// The most confident word above `threshold`, if any.
    pub fn best_above(&self, threshold: f32) -> Option<&str> {
        self.words
            .iter()
            .filter(|(_, confidence)| *confidence > threshold)
            .max_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(word, _)| word.as_str())
    }
}

/// Face detection event: bounding boxes in angular coordinates plus camera pose.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceEvent {
    /// Event stamp as reported: seconds and microseconds.
    pub stamp: [f32; 2],
    /// Per face: `[id, x, y, width, height]` in angular coordinates.
    pub faces: Vec<Vec<f32>>,
    pub camera_pose_in_torso: Vec<f32>,
    pub camera_pose_in_robot: Vec<f32>,
    pub camera_id: i32,
}

/// Audio chunk pushed by ALAudioDevice via `processRemote`.
#[derive(Clone, Debug, PartialEq)]
pub struct RemoteAudio {
    pub channels: i32,
    pub samples_per_channel: i32,
    /// Interleaved 16-bit samples.
    pub data: Vec<i16>,
}

/// Maps a touch event key to its domain id.
pub fn touch_id(key: &str) -> Option<TouchId> {
    super::keys::TOUCH_EVENTS
        .iter()
        .find(|(name, _)| *name == key)
        .map(|(_, id)| *id)
}

/// Decodes a touch event payload: a pressed flag as float.
pub fn decode_touch(key: &str, value: &Value<'_>) -> Option<Touch> {
    Some(Touch {
        id: touch_id(key)?,
        pressed: as_f32(value)? != 0.0,
    })
}

/// Decodes the flat `[word, confidence, ...]` list of `WordRecognized`.
pub fn decode_word_recognized(value: &Value<'_>) -> Option<WordRecognized> {
    let elements = match plain(value.clone()) {
        Value::List(elements) | Value::Tuple(elements) => elements,
        _ => return None,
    };
    let mut words = Vec::new();
    for pair in elements.chunks_exact(2) {
        words.push((strip_word_wrap(&as_text(&pair[0])?), as_f32(&pair[1])?));
    }
    Some(WordRecognized { words })
}

/// Strips the `<...>` word-spotting wrap: six characters on each side.
fn strip_word_wrap(word: &str) -> String {
    if word.len() > 12 {
        word.chars().skip(6).take(word.len() - 12).collect()
    } else {
        word.to_owned()
    }
}

/// Decodes `ALSoundLocalization/SoundLocated`.
pub fn decode_sound_located(value: &Value<'_>) -> Option<SoundBearing> {
    let elements = list_elements(value)?;
    let bearing = as_f32s(&elements[1])?;
    Some(SoundBearing {
        stamp: Timestamp::default(),
        azimuth: *bearing.first()?,
        elevation: *bearing.get(1)?,
        confidence: *bearing.get(2)?,
        energy: *bearing.get(3)?,
        head_in_torso: as_f32s(&elements[2])?,
        head_in_robot: as_f32s(&elements[3])?,
    })
}

/// Decodes the `FaceDetected` event.
pub fn decode_face_event(value: &Value<'_>) -> Option<FaceEvent> {
    let elements = list_elements(value)?;
    let stamp = as_f32s(&elements[0])?;
    let faces = match plain(elements[1].clone()) {
        Value::List(faces) | Value::Tuple(faces) => faces
            .iter()
            .map(|face| {
                let face = super::value::plain(face.clone());
                match face {
                    Value::List(parts) | Value::Tuple(parts) => as_f32s(&parts.first()?.clone()),
                    _ => None,
                }
            })
            .collect::<Option<Vec<_>>>()?,
        _ => return None,
    };
    Some(FaceEvent {
        stamp: [*stamp.first()?, *stamp.get(1)?],
        faces,
        camera_pose_in_torso: as_f32s(&elements[2])?,
        camera_pose_in_robot: as_f32s(&elements[3])?,
        camera_id: as_i32(&elements[4])?,
    })
}

/// Decodes `NAOqiPlanner/Result`: a plain string.
pub fn decode_result(value: &Value<'_>) -> Option<String> {
    as_text(value)
}

/// Decodes the `processRemote` arguments of the microphone client.
pub fn decode_audio(
    channels: i32,
    samples_per_channel: i32,
    buffer: &Value<'_>,
) -> Option<RemoteAudio> {
    let raw = as_bytes(buffer)?;
    let data: Vec<i16> = raw
        .chunks_exact(2)
        .map(|bytes| i16::from_le_bytes([bytes[0], bytes[1]]))
        .collect();
    Some(RemoteAudio {
        channels,
        samples_per_channel,
        data,
    })
}

fn list_elements(value: &Value<'_>) -> Option<Vec<Value<'static>>> {
    match plain(value.clone()) {
        Value::List(elements) | Value::Tuple(elements) => Some(elements),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use qi::value::IntoValue;

    #[test]
    fn touch_decoding_maps_keys_and_flags() {
        let touch = decode_touch("RightBumperPressed", &1.0f32.into_value()).expect("touch");
        assert_eq!(touch.id, TouchId::BumperRight);
        assert!(touch.pressed);
        assert!(
            !decode_touch("RightBumperPressed", &0.0f32.into_value())
                .expect("touch")
                .pressed
        );
        assert!(decode_touch("NoSuchKey", &1.0f32.into_value()).is_none());
    }

    #[test]
    fn word_recognized_strips_the_word_spotting_wrap() {
        let value = vec![
            "<...> robot <...>".to_owned().into_value(),
            0.8f32.into_value(),
            "<...> stop <...>".to_owned().into_value(),
            0.4f32.into_value(),
        ]
        .into_value();
        let decoded = decode_word_recognized(&value).expect("words");
        assert_eq!(
            decoded.words,
            vec![("robot".to_owned(), 0.8), ("stop".to_owned(), 0.4)]
        );
        assert_eq!(decoded.best_above(0.5), Some("robot"));
        assert_eq!(decoded.best_above(0.9), None);
    }

    #[test]
    fn short_words_are_kept_as_is() {
        assert_eq!(strip_word_wrap("robot"), "robot");
    }

    #[test]
    fn sound_located_reads_the_three_sections() {
        let value = vec![
            0.0f32.into_value(),
            vec![0.1f32, 0.2, 0.3, 0.4].into_value(),
            vec![1.0f32, 2.0].into_value(),
            vec![3.0f32, 4.0, 5.0].into_value(),
        ]
        .into_value();
        let decoded = decode_sound_located(&value).expect("bearing");
        assert_eq!(decoded.azimuth, 0.1);
        assert_eq!(decoded.energy, 0.4);
        assert_eq!(decoded.head_in_torso, vec![1.0, 2.0]);
        assert_eq!(decoded.head_in_robot, vec![3.0, 4.0, 5.0]);
    }

    #[test]
    fn face_event_reads_shapes_and_camera_id() {
        let face = vec![
            vec![0.0f32, 0.1, 0.2, 0.3, 0.4].into_value(),
            0.0f32.into_value(),
            0.0f32.into_value(),
        ]
        .into_value();
        let value = vec![
            vec![100.0f32, 200.0].into_value(),
            vec![face].into_value(),
            vec![1.0f32, 2.0].into_value(),
            vec![3.0f32].into_value(),
            0.0f32.into_value(),
        ]
        .into_value();
        let decoded = decode_face_event(&value).expect("faces");
        assert_eq!(decoded.stamp, [100.0, 200.0]);
        assert_eq!(decoded.faces.len(), 1);
        assert_eq!(decoded.faces[0], vec![0.0, 0.1, 0.2, 0.3, 0.4]);
        assert_eq!(decoded.camera_id, 0);
    }

    #[test]
    fn audio_decoding_interleaves_16_bit_samples() {
        let buffer = vec![0x01u8, 0x02, 0xff, 0x7f, 0x00, 0x80];
        let decoded = decode_audio(2, 3, &buffer.clone().into_value()).expect("audio");
        assert_eq!(decoded.data, vec![0x0201, 0x7fff, 0x8000u16 as i16]);
    }
}
