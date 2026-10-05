//! Touch sensor events.

use super::{Capability, CapabilityId, Context};
use crate::Result;
use crate::domain::Message;
use crate::qi::keys;
use async_trait::async_trait;

/// Publishes bumper, head and hand touch readings.
pub struct Touch;

#[async_trait]
impl Capability for Touch {
    fn id(&self) -> CapabilityId {
        CapabilityId::Touch
    }

    async fn enable(&self, ctx: &Context) -> Result<()> {
        let transport = ctx.transport.clone();
        let topic = self.id().as_str();
        ctx.toolkit.touch().set(move |touch| {
            if let Err(err) = transport.publish(topic, Message::Touch(touch)) {
                tracing::warn!(error = %err, "touch publish failed");
            }
        });
        for (key, _) in keys::TOUCH_EVENTS {
            if let Err(err) = ctx
                .robot
                .memory
                .subscribe_to_event(key, &keys::callback_service_name(key), "touchCallback")
                .await
            {
                self.disable(ctx).await.ok();
                return Err(err);
            }
        }
        Ok(())
    }

    async fn disable(&self, ctx: &Context) -> Result<()> {
        ctx.toolkit.touch().clear();
        let mut failure = None;
        for (key, _) in keys::TOUCH_EVENTS {
            if let Err(err) = ctx
                .robot
                .memory
                .unsubscribe_to_event(key, &keys::callback_service_name(key))
                .await
                && failure.is_none()
            {
                failure = Some(err);
            }
        }
        match failure {
            Some(err) => Err(err),
            None => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capabilities::Capability;
    use crate::capabilities::support::Harness;
    use crate::domain::{Touch as TouchMessage, TouchId};
    use crate::qi::value::Raw;
    use qi::object::{MemberIdent, Object};
    use qi::value::IntoValue;

    fn touch_event(key: &str, pressed: f32) -> qi::Value<'static> {
        (
            key.to_owned(),
            Raw::new(pressed.into_value()),
            "subscriber".to_owned(),
        )
            .into_value()
    }

    #[tokio::test]
    async fn enable_subscribes_every_touch_event() {
        let harness = Harness::default();
        Touch.enable(&harness.ctx).await.expect("enable");

        let subscriptions = harness
            .fakes
            .service("ALMemory")
            .calls_to("subscribeToEvent");
        assert_eq!(subscriptions.len(), keys::TOUCH_EVENTS.len());
        assert!(
            subscriptions.contains(
                &(
                    "FrontTactilTouched".to_owned(),
                    keys::callback_service_name("FrontTactilTouched"),
                    "touchCallback".to_owned()
                )
                    .into_value()
            )
        );
    }

    #[tokio::test]
    async fn events_are_published_on_the_touch_topic() {
        let harness = Harness::default();
        Touch.enable(&harness.ctx).await.expect("enable");

        harness
            .ctx
            .toolkit
            .meta_call(
                MemberIdent::from("touchCallback"),
                touch_event("FrontTactilTouched", 1.0),
            )
            .await
            .expect("event");

        let published = harness.transport.published_on("touch");
        assert_eq!(published.len(), 1);
        let Message::Touch(touch) = &published[0] else {
            panic!("expected a touch message");
        };
        assert_eq!(
            *touch,
            TouchMessage {
                id: TouchId::HeadFront,
                pressed: true
            }
        );
    }

    #[tokio::test]
    async fn disable_unsubscribes_every_event_and_clears_the_slot() {
        let harness = Harness::default();
        Touch.enable(&harness.ctx).await.expect("enable");
        Touch.disable(&harness.ctx).await.expect("disable");

        let memory = harness.fakes.service("ALMemory");
        assert_eq!(
            memory.calls_to("unsubscribeToEvent").len(),
            keys::TOUCH_EVENTS.len()
        );

        harness
            .ctx
            .toolkit
            .meta_call(
                MemberIdent::from("touchCallback"),
                touch_event("FrontTactilTouched", 1.0),
            )
            .await
            .expect("event");
        assert!(harness.transport.published_on("touch").is_empty());
    }

    #[test]
    fn touch_events_cover_the_documented_sensors() {
        let ids: Vec<TouchId> = keys::TOUCH_EVENTS.iter().map(|(_, id)| *id).collect();
        assert_eq!(ids.len(), 12);
        assert!(ids.contains(&TouchId::BumperRight));
        assert!(ids.contains(&TouchId::HandLeftLeft));
    }
}
