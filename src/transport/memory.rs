//! In-memory transport.
//!
//! Records published messages, delivers injected inputs to subscribers and
//! dispatches control RPCs to registered handlers. It backs the test suite and
//! serves as the reference for real transport adapters.

use super::{MessageHandler, RpcHandler, Subscription, Timer, Transport};
use crate::domain::{ControlRequest, ControlResponse, Message, Timestamp};
use futures::stream;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default)]
struct Inner {
    published: Vec<(String, Message)>,
    handlers: HashMap<String, Vec<(u64, MessageHandler)>>,
    rpcs: HashMap<String, (u64, RpcHandler)>,
    consumer_overrides: HashMap<String, bool>,
    next_id: u64,
}

/// Test double and reference implementation of [`Transport`].
#[derive(Clone, Default)]
pub struct MemoryTransport {
    inner: Arc<Mutex<Inner>>,
}

impl MemoryTransport {
    pub fn new() -> Self {
        Self::default()
    }

    /// Everything published so far, in order.
    pub fn published(&self) -> Vec<(String, Message)> {
        self.lock().published.clone()
    }

    /// Messages published on `topic`, in order.
    pub fn published_on(&self, topic: &str) -> Vec<Message> {
        self.lock()
            .published
            .iter()
            .filter(|(name, _)| name == topic)
            .map(|(_, message)| message.clone())
            .collect()
    }

    pub fn clear_published(&self) {
        self.lock().published.clear();
    }

    /// Feeds a bus input to the subscribers of `topic`.
    pub fn inject(&self, topic: &str, message: Message) {
        let handlers: Vec<MessageHandler> = self
            .lock()
            .handlers
            .get(topic)
            .map(|entries| entries.iter().map(|(_, handler)| handler.clone()).collect())
            .unwrap_or_default();
        for handler in handlers {
            handler(message.clone());
        }
    }

    /// Invokes a control RPC handler; `None` when nobody serves `name`.
    pub async fn call_rpc(&self, name: &str, request: ControlRequest) -> Option<ControlResponse> {
        let rpc = self
            .lock()
            .rpcs
            .get(name)
            .map(|(_, handler)| handler.clone())?;
        Some(rpc(request).await)
    }

    /// Overrides the consumer report of `topic` (defaults to `true`).
    pub fn set_has_consumers(&self, topic: &str, has_consumers: bool) {
        self.lock()
            .consumer_overrides
            .insert(topic.to_owned(), has_consumers);
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        // A poisoned mutex means a panicking handler; carrying on is safe here.
        self.inner.lock().unwrap_or_else(|err| err.into_inner())
    }
}

impl Transport for MemoryTransport {
    fn publish(&self, topic: &str, message: Message) -> crate::Result<()> {
        self.lock().published.push((topic.to_owned(), message));
        Ok(())
    }

    fn has_consumers(&self, topic: &str) -> bool {
        self.lock()
            .consumer_overrides
            .get(topic)
            .copied()
            .unwrap_or(true)
    }

    fn subscribe(&self, topic: &str, handler: MessageHandler) -> Subscription {
        let mut inner = self.lock();
        let id = inner.next_id;
        inner.next_id += 1;
        inner
            .handlers
            .entry(topic.to_owned())
            .or_default()
            .push((id, handler));
        let (state, topic) = (Arc::clone(&self.inner), topic.to_owned());
        Subscription::on_cancel(move || {
            if let Some(entries) = state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .handlers
                .get_mut(&topic)
            {
                entries.retain(|(entry_id, _)| *entry_id != id);
            }
        })
    }

    fn register_rpc(&self, name: &str, handler: RpcHandler) -> Subscription {
        let mut inner = self.lock();
        let id = inner.next_id;
        inner.next_id += 1;
        inner.rpcs.insert(name.to_owned(), (id, handler));
        let (state, name) = (Arc::clone(&self.inner), name.to_owned());
        Subscription::on_cancel(move || {
            state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .rpcs
                .remove(&name);
        })
    }

    fn timer(&self, hz: f32) -> Timer {
        if hz <= 0.0 {
            return Box::pin(stream::empty());
        }
        let period = Duration::from_secs_f64(1.0 / f64::from(hz));
        let interval = tokio::time::interval(period);
        Box::pin(stream::unfold(interval, |mut interval| async move {
            interval.tick().await;
            Some(((), interval))
        }))
    }

    fn now(&self) -> Timestamp {
        std::time::SystemTime::now().into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{MiscCommand, Twist};

    #[test]
    fn subscription_teardown_removes_the_handler() {
        let transport = MemoryTransport::new();
        let received = Arc::new(Mutex::new(Vec::new()));
        let slot = Arc::clone(&received);
        let subscription = transport.subscribe(
            "cmd_vel",
            Arc::new(move |message| slot.lock().unwrap().push(message)),
        );

        transport.inject("cmd_vel", Message::CmdVel(Twist::default()));
        assert_eq!(received.lock().unwrap().len(), 1);

        drop(subscription);
        transport.inject("cmd_vel", Message::CmdVel(Twist::default()));
        assert_eq!(received.lock().unwrap().len(), 1);
    }

    #[test]
    fn consumer_report_can_be_overridden() {
        let transport = MemoryTransport::new();
        assert!(transport.has_consumers("tf"));
        transport.set_has_consumers("tf", false);
        assert!(!transport.has_consumers("tf"));
    }

    #[tokio::test]
    async fn rpc_dispatch_and_teardown() {
        let transport = MemoryTransport::new();
        let subscription = transport.register_rpc(
            "misc_tools",
            Arc::new(|_request| Box::pin(async { ControlResponse::ok() })),
        );
        let response = transport
            .call_rpc("misc_tools", ControlRequest::Misc(MiscCommand::EnableAll))
            .await;
        assert_eq!(response, Some(ControlResponse::ok()));

        drop(subscription);
        assert!(
            transport
                .call_rpc("misc_tools", ControlRequest::Misc(MiscCommand::EnableAll))
                .await
                .is_none()
        );
    }

    #[tokio::test(start_paused = true)]
    async fn timer_ticks_repeatedly() {
        let transport = MemoryTransport::new();
        let mut timer = transport.timer(10.0);
        use futures::StreamExt;
        for _ in 0..3 {
            assert!(timer.next().await.is_some());
        }
    }
}
