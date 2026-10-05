//! Text-to-speech and animated speech.

use super::{
    Capability, CapabilityId, Configuration, ConfigurationResult, Context, message_handler,
};
use crate::Result;
use crate::domain::{Language, Message, SpeechCommand, SpeechParams};
use crate::qi::Robot;
use crate::transport::Subscription;
use async_trait::async_trait;
use std::sync::{Arc, Mutex};

/// Said instead of the requested text when the language is unsupported.
const APOLOGY_ENGLISH: &str = "I am sorry, I dont know that language";
const APOLOGY_SPANISH: &str = "Lo siento, no se hablar en ese idioma";

/// Says what the bus asks, in the language the robot currently speaks.
pub struct Speech {
    language: Arc<Mutex<Language>>,
    subscription: Mutex<Option<Subscription>>,
}

impl Speech {
    pub fn new() -> Self {
        Self {
            language: Arc::new(Mutex::new(Language::English)),
            subscription: Mutex::new(None),
        }
    }

    /// The language of the voice currently set.
    pub fn language(&self) -> Language {
        *self.language.lock().unwrap_or_else(|err| err.into_inner())
    }
}

impl Default for Speech {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Capability for Speech {
    fn id(&self) -> CapabilityId {
        CapabilityId::Speech
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let robot = ctx.robot.clone();
        let language = Arc::clone(&self.language);
        let subscription = ctx.transport.subscribe(
            self.id().as_str(),
            message_handler(
                |message| match message {
                    Message::Speech(command) => Some(command.clone()),
                    _ => None,
                },
                move |command| {
                    let robot = Arc::clone(&robot);
                    let language = Arc::clone(&language);
                    Box::pin(async move { say(&robot, &language, command).await })
                },
            ),
        );
        *self
            .subscription
            .lock()
            .unwrap_or_else(|err| err.into_inner()) = Some(subscription);
        Ok(())
    }

    async fn disable(&self, _ctx: &Context) -> Result<()> {
        self.subscription
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .take();
        Ok(())
    }

    async fn configure(
        &self,
        ctx: &Context,
        configuration: Configuration,
    ) -> Result<ConfigurationResult> {
        match configuration {
            Configuration::Speech(params) => {
                params.validate()?;
                apply_params(&ctx.robot, params).await?;
                Ok(ConfigurationResult::None)
            }
            Configuration::ReadSpeechParams => Ok(ConfigurationResult::SpeechParams(
                read_params(&ctx.robot).await?,
            )),
            Configuration::ResetSpeechParams => {
                let params = SpeechParams::defaults(self.language());
                apply_params(&ctx.robot, params).await?;
                Ok(ConfigurationResult::None)
            }
            _ => Ok(ConfigurationResult::None),
        }
    }
}

async fn say(robot: &Robot, language: &Mutex<Language>, command: SpeechCommand) -> Result<()> {
    let Some(requested) = command.language else {
        let apology = match *language.lock().unwrap_or_else(|err| err.into_inner()) {
            Language::English => APOLOGY_ENGLISH,
            Language::Spanish => APOLOGY_SPANISH,
        };
        return robot.text_to_speech.say(apology).await;
    };
    if requested != *language.lock().unwrap_or_else(|err| err.into_inner()) {
        robot
            .text_to_speech
            .set_language(language_name(requested))
            .await?;
        *language.lock().unwrap_or_else(|err| err.into_inner()) = requested;
    }
    if command.animated {
        robot.animated_speech.say(&command.text).await
    } else {
        robot.text_to_speech.say(&command.text).await
    }
}

fn language_name(language: Language) -> &'static str {
    match language {
        Language::English => "English",
        Language::Spanish => "Spanish",
    }
}

async fn apply_params(robot: &Robot, params: SpeechParams) -> Result<()> {
    let tts = &robot.text_to_speech;
    tts.set_parameter("pitchShift", params.pitch_shift).await?;
    tts.set_parameter("doubleVoice", params.double_voice)
        .await?;
    tts.set_parameter("doubleVoiceLevel", params.double_voice_level)
        .await?;
    tts.set_parameter("doubleVoiceTimeShift", params.double_voice_time_shift)
        .await?;
    tts.set_parameter("speed", params.speed).await
}

async fn read_params(robot: &Robot) -> Result<SpeechParams> {
    let tts = &robot.text_to_speech;
    Ok(SpeechParams {
        pitch_shift: tts.get_parameter("pitchShift").await?,
        double_voice: tts.get_parameter("doubleVoice").await?,
        double_voice_level: tts.get_parameter("doubleVoiceLevel").await?,
        double_voice_time_shift: tts.get_parameter("doubleVoiceTimeShift").await?,
        speed: tts.get_parameter("speed").await?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use qi::value::IntoValue;
    use std::time::Duration;

    fn command(language: Option<Language>, text: &str, animated: bool) -> SpeechCommand {
        SpeechCommand {
            language,
            text: text.to_owned(),
            animated,
        }
    }

    async fn say_now(harness: &Harness, command: SpeechCommand) {
        harness.transport.inject("speech", Message::Speech(command));
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    fn texts(harness: &Harness) -> Vec<qi::value::Value<'static>> {
        harness.fakes.service("ALTextToSpeech").calls_to("say")
    }

    fn set_languages(harness: &Harness) -> Vec<qi::value::Value<'static>> {
        harness
            .fakes
            .service("ALTextToSpeech")
            .calls_to("setLanguage")
    }

    #[tokio::test]
    async fn commands_switch_language_only_when_it_changes() {
        let harness = Harness::default();
        let speech = Speech::new();
        speech.enable(&harness.ctx).await.expect("enable");

        say_now(&harness, command(Some(Language::Spanish), "hola", false)).await;
        say_now(&harness, command(Some(Language::Spanish), "adios", false)).await;
        say_now(&harness, command(Some(Language::English), "hello", true)).await;

        assert_eq!(
            set_languages(&harness),
            vec![
                "Spanish".to_owned().into_value(),
                "English".to_owned().into_value()
            ]
        );
        assert_eq!(
            harness.fakes.service("ALAnimatedSpeech").calls_to("say"),
            vec!["hello".to_owned().into_value()]
        );
        assert_eq!(
            texts(&harness),
            vec![
                "hola".to_owned().into_value(),
                "adios".to_owned().into_value()
            ]
        );
    }

    #[tokio::test]
    async fn unsupported_language_says_the_apology_in_the_current_language() {
        let harness = Harness::default();
        let speech = Speech::new();
        speech.enable(&harness.ctx).await.expect("enable");

        say_now(&harness, command(None, "ignored", false)).await;
        say_now(&harness, command(Some(Language::Spanish), "hola", false)).await;
        say_now(&harness, command(None, "ignored", true)).await;

        assert_eq!(
            texts(&harness),
            vec![
                APOLOGY_ENGLISH.to_owned().into_value(),
                "hola".to_owned().into_value(),
                APOLOGY_SPANISH.to_owned().into_value(),
            ]
        );
        // The apology never touches the voice language.
        assert_eq!(
            set_languages(&harness),
            vec!["Spanish".to_owned().into_value()]
        );
    }

    #[tokio::test]
    async fn set_speech_params_applies_all_five_parameters() {
        let harness = Harness::default();
        let speech = Speech::new();
        let params = SpeechParams {
            pitch_shift: 1.5,
            double_voice: 0.0,
            double_voice_level: 2.0,
            double_voice_time_shift: 0.1,
            speed: 120.0,
        };
        speech
            .configure(&harness.ctx, Configuration::Speech(params))
            .await
            .expect("configure");

        assert_eq!(
            harness
                .fakes
                .service("ALTextToSpeech")
                .calls_to("setParameter"),
            vec![
                ("pitchShift".to_owned(), 1.5f32).into_value(),
                ("doubleVoice".to_owned(), 0.0f32).into_value(),
                ("doubleVoiceLevel".to_owned(), 2.0f32).into_value(),
                ("doubleVoiceTimeShift".to_owned(), 0.1f32).into_value(),
                ("speed".to_owned(), 120.0f32).into_value(),
            ]
        );
    }

    #[tokio::test]
    async fn read_speech_params_reports_the_voice_settings() {
        let harness = Harness::default();
        let tts = harness.fakes.service("ALTextToSpeech");
        for (name, value) in [
            ("pitchShift", 1.2),
            ("doubleVoice", 0.0),
            ("doubleVoiceLevel", 0.0),
            ("doubleVoiceTimeShift", 0.0),
            ("speed", 150.0),
        ] {
            tts.script_for("getParameter", name, (value as f32).into_value());
        }
        let speech = Speech::new();
        let result = speech
            .configure(&harness.ctx, Configuration::ReadSpeechParams)
            .await
            .expect("configure");

        assert_eq!(
            result,
            ConfigurationResult::SpeechParams(SpeechParams {
                pitch_shift: 1.2,
                double_voice: 0.0,
                double_voice_level: 0.0,
                double_voice_time_shift: 0.0,
                speed: 150.0,
            })
        );
    }

    #[tokio::test]
    async fn reset_speech_params_applies_the_defaults_of_the_current_language() {
        let harness = Harness::default();
        let speech = Speech::new();
        speech.enable(&harness.ctx).await.expect("enable");
        say_now(&harness, command(Some(Language::Spanish), "hola", false)).await;
        speech
            .configure(&harness.ctx, Configuration::ResetSpeechParams)
            .await
            .expect("configure");

        let params = SpeechParams::defaults(Language::Spanish);
        assert_eq!(
            harness
                .fakes
                .service("ALTextToSpeech")
                .calls_to("setParameter"),
            vec![
                ("pitchShift".to_owned(), params.pitch_shift).into_value(),
                ("doubleVoice".to_owned(), params.double_voice).into_value(),
                ("doubleVoiceLevel".to_owned(), params.double_voice_level).into_value(),
                (
                    "doubleVoiceTimeShift".to_owned(),
                    params.double_voice_time_shift
                )
                    .into_value(),
                ("speed".to_owned(), params.speed).into_value(),
            ]
        );
    }

    #[tokio::test]
    async fn invalid_params_are_rejected() {
        let harness = Harness::default();
        let speech = Speech::new();
        let bad = Configuration::Speech(SpeechParams {
            pitch_shift: 0.5,
            ..SpeechParams::defaults(Language::English)
        });
        assert!(speech.configure(&harness.ctx, bad).await.is_err());
    }

    #[tokio::test]
    async fn disable_stops_the_input() {
        let harness = Harness::default();
        let speech = Speech::new();
        speech.enable(&harness.ctx).await.expect("enable");
        speech.disable(&harness.ctx).await.expect("disable");
        say_now(&harness, command(Some(Language::Spanish), "hola", false)).await;
        assert!(texts(&harness).is_empty());
    }
}
