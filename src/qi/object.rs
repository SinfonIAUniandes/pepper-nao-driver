//! The QI object the driver serves to the robot.
//!
//! It answers the `robot_toolkit` handshake and receives the ALMemory event
//! callbacks, decoding payloads and handing them to the registered sinks.

use super::events::{self, FaceEvent, RemoteAudio, WordRecognized};
use super::value::Raw;
use crate::domain::{Touch, SoundBearing};
use async_trait::async_trait;
use qi::object::{MemberIdent, MetaMethod, MetaObject, Object, ACTION_START_ID};
use qi::value::{FromValue, IntoValue, Type, Value};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// One callback target of the served object.
#[derive(Clone)]
pub struct Slot<T>(Arc<Mutex<Option<Arc<dyn Fn(T) + Send + Sync>>>>);

impl<T> Default for Slot<T> {
    fn default() -> Self {
        Self(Arc::default())
    }
}

impl<T> Slot<T> {
    /// Installs the handler; replaces any previous one.
    pub fn set(&self, handler: impl Fn(T) + Send + Sync + 'static) {
        *self.0.lock().unwrap_or_else(|err| err.into_inner()) = Some(Arc::new(handler));
    }

    /// Removes the handler.
    pub fn clear(&self) {
        *self.0.lock().unwrap_or_else(|err| err.into_inner()) = None;
    }

    fn dispatch(&self, value: T) {
        let handler = self.0.lock().unwrap_or_else(|err| err.into_inner()).clone();
        if let Some(handler) = handler {
            handler(value);
        }
    }
}

/// State, sinks and handshake methods of the served `robot_toolkit` object.
#[derive(Default)]
pub struct Toolkit {
    instance_prefix: String,
    publish_enabled: AtomicBool,
    attached_transport: Mutex<Option<String>>,
    pub touch: Slot<Touch>,
    pub word_recognized: Slot<WordRecognized>,
    pub sound_located: Slot<SoundBearing>,
    pub face_detected: Slot<FaceEvent>,
    pub result: Slot<String>,
    pub audio: Slot<RemoteAudio>,
}

impl Toolkit {
    pub fn new(instance_prefix: impl Into<String>) -> Self {
        Self {
            instance_prefix: instance_prefix.into(),
            ..Default::default()
        }
    }

    pub fn instance_prefix(&self) -> &str {
        &self.instance_prefix
    }

    /// Whether `startPublishing` was called: periodic publishing stays off
    /// until the transport stack is up.
    pub fn publish_enabled(&self) -> bool {
        self.publish_enabled.load(Ordering::Relaxed)
    }

    /// Records the attached transport; returns an error string on bad input.
    pub fn attach_transport(&self, name: &str) -> String {
        if name.is_empty() {
            return "error: transport name must not be empty".to_owned();
        }
        *self
            .attached_transport
            .lock()
            .unwrap_or_else(|err| err.into_inner()) = Some(name.to_owned());
        "ok".to_owned()
    }

    pub fn attached_transport(&self) -> Option<String> {
        self.attached_transport
            .lock()
            .unwrap_or_else(|err| err.into_inner())
            .clone()
    }
}

/// Parameter type of the event callbacks: event key, dynamic payload and the
/// subscriber identifier.
const CALLBACK_PARAMS: [Option<Type>; 3] = [Some(Type::String), None, Some(Type::String)];

fn build_meta() -> MetaObject {
    let mut builder = MetaObject::builder();
    let mut uid = ACTION_START_ID.0;
    let mut add = |name: &str, params: &[Option<Type>], returns: Option<Type>| {
        let mut method = MetaMethod::builder(uid);
        uid += 1;
        method.set_name(name);
        for (index, ty) in params.iter().enumerate() {
            method.parameter(index).set_type(ty.clone());
        }
        method.return_value().set_type(returns);
        builder.add_method(method.build());
    };
    add("_whoWillWin", &[], Some(Type::String));
    add("attach-transport", &[Some(Type::String)], Some(Type::String));
    add("startPublishing", &[], Some(Type::Unit));
    add("touchCallback", &CALLBACK_PARAMS, Some(Type::Unit));
    add("wordRecognizedCallback", &CALLBACK_PARAMS, Some(Type::Unit));
    add("soundLocatedCallback", &CALLBACK_PARAMS, Some(Type::Unit));
    add("faceDetectedCallback", &CALLBACK_PARAMS, Some(Type::Unit));
    add("onResultCallback", &CALLBACK_PARAMS, Some(Type::Unit));
    add(
        "processRemote",
        &[Some(Type::Int32), Some(Type::Int32), None, None],
        Some(Type::Unit),
    );
    builder.build()
}

static META: std::sync::LazyLock<MetaObject> = std::sync::LazyLock::new(build_meta);

#[async_trait]
impl Object for Toolkit {
    fn meta(&self) -> &MetaObject {
        &META
    }

    async fn meta_call(&self, ident: MemberIdent, args: Value<'_>) -> qi::Result<Value<'static>> {
        let name = match &ident {
            MemberIdent::Name(name) => name.clone(),
            MemberIdent::Id(id) => META
                .methods
                .values()
                .find(|method| method.uid == *id)
                .map(|method| method.name.clone())
                .ok_or(qi::Error::MethodNotFound(ident.clone()))?,
        };
        match name.as_str() {
            "_whoWillWin" => Ok(self.instance_prefix.clone().into_value().into_owned()),
            "attach-transport" => {
                let transport: String = decode_arg(args, "attach-transport")?;
                Ok(self.attach_transport(&transport).into_value().into_owned())
            }
            "startPublishing" => {
                self.publish_enabled.store(true, Ordering::Relaxed);
                Ok(Value::Unit)
            }
            "touchCallback" => self.dispatch_event(args, |key, value| {
                if let Some(touch) = events::decode_touch(&key, &value) {
                    self.touch.dispatch(touch);
                }
            }),
            "wordRecognizedCallback" => self.dispatch_event(args, |key, value| {
                let _ = key;
                if let Some(words) = events::decode_word_recognized(&value) {
                    self.word_recognized.dispatch(words);
                }
            }),
            "soundLocatedCallback" => self.dispatch_event(args, |key, value| {
                let _ = key;
                if let Some(bearing) = events::decode_sound_located(&value) {
                    self.sound_located.dispatch(bearing);
                }
            }),
            "faceDetectedCallback" => self.dispatch_event(args, |key, value| {
                let _ = key;
                if let Some(faces) = events::decode_face_event(&value) {
                    self.face_detected.dispatch(faces);
                }
            }),
            "onResultCallback" => self.dispatch_event(args, |key, value| {
                let _ = key;
                if let Some(result) = events::decode_result(&value) {
                    self.result.dispatch(result);
                }
            }),
            "processRemote" => {
                let (channels, samples, _timestamp, buffer): (i32, i32, Raw, Raw) = decode_args(args)?;
                if let Some(audio) = events::decode_audio(channels, samples, &buffer.into_inner()) {
                    self.audio.dispatch(audio);
                }
                Ok(Value::Unit)
            }
            _ => Err(qi::Error::MethodNotFound(ident)),
        }
    }

    async fn meta_post(&self, ident: MemberIdent, args: Value<'_>) {
        let _ = self.meta_call(ident, args).await;
    }

    async fn meta_event(&self, _ident: MemberIdent, _value: Value<'_>) {}
}

impl Toolkit {
    /// Decodes the `(key, payload, subscriber)` triple of an event callback and
    /// runs `handle`; malformed payloads are dropped.
    fn dispatch_event(
        &self,
        args: Value<'_>,
        handle: impl FnOnce(String, Value<'static>),
    ) -> qi::Result<Value<'static>> {
        if let Ok((key, value, _subscriber)) = decode_args::<(String, Raw, String)>(args.clone()) {
            handle(key, value.into_inner());
        }
        Ok(Value::Unit)
    }
}

fn decode_arg<T: for<'a> FromValue<'a>>(args: Value<'_>, context: &'static str) -> qi::Result<T> {
    args.cast_into()
        .map_err(|_| qi::Error::Other(format!("{context}: malformed arguments").into()))
}

fn decode_args<T: for<'a> FromValue<'a>>(args: Value<'_>) -> qi::Result<T> {
    args.cast_into()
        .map_err(|err| qi::Error::Other(qi::BoxError::from(err)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn callback_args(key: &str, payload: Value<'static>) -> Value<'static> {
        (key.to_owned(), Raw::new(payload), "subscriber".to_owned()).into_value()
    }

    #[tokio::test]
    async fn handshake_reports_prefix_and_enables_publishing() {
        let toolkit = Toolkit::new("pepper-1");
        let prefix: String = toolkit
            .meta_call("_whoWillWin".into(), Value::Unit)
            .await
            .expect("call")
            .cast_into()
            .expect("string");
        assert_eq!(prefix, "pepper-1");

        assert!(!toolkit.publish_enabled());
        toolkit
            .meta_call("startPublishing".into(), Value::Unit)
            .await
            .expect("call");
        assert!(toolkit.publish_enabled());

        let result: String = toolkit
            .meta_call("attach-transport".into(), "ros2".to_owned().into_value())
            .await
            .expect("call")
            .cast_into()
            .expect("string");
        assert_eq!(result, "ok");
        assert_eq!(toolkit.attached_transport().as_deref(), Some("ros2"));

        let result: String = toolkit
            .meta_call("attach-transport".into(), "".to_owned().into_value())
            .await
            .expect("call")
            .cast_into()
            .expect("string");
        assert!(result.starts_with("error"));
    }

    #[tokio::test]
    async fn touch_events_reach_their_slot() {
        let toolkit = Toolkit::default();
        let received = Arc::new(Mutex::new(Vec::new()));
        let slot = Arc::clone(&received);
        toolkit
            .touch
            .set(move |touch| slot.lock().unwrap().push(touch));

        toolkit
            .meta_call(
                "touchCallback".into(),
                callback_args("FrontTactilTouched", 1.0f32.into_value()),
            )
            .await
            .expect("call");
        let touches = received.lock().unwrap();
        assert_eq!(touches.len(), 1);
        assert_eq!(touches[0].id, crate::domain::TouchId::HeadFront);
    }

    #[tokio::test]
    async fn malformed_event_payloads_are_dropped() {
        let toolkit = Toolkit::default();
        let fired = Arc::new(AtomicBool::new(false));
        let slot = Arc::clone(&fired);
        toolkit
            .word_recognized
            .set(move |_| slot.store(true, Ordering::Relaxed));
        toolkit
            .meta_call(
                "wordRecognizedCallback".into(),
                callback_args("WordRecognized", Value::Unit),
            )
            .await
            .expect("call");
        assert!(!fired.load(Ordering::Relaxed));
    }

    #[tokio::test]
    async fn unknown_methods_are_rejected() {
        let toolkit = Toolkit::default();
        assert!(toolkit
            .meta_call("nope".into(), Value::Unit)
            .await
            .is_err());
    }
}
