//! The adapter seam: the driver talks to a [`Transport`], never to a bus.
//!
//! Topic names are the capability ids (see [`crate::capabilities::CapabilityId`]);
//! control RPCs are named after their tool group (`navigation_tools`, ...).

use crate::domain::{ControlRequest, ControlResponse, Message, Timestamp};
use futures::future::BoxFuture;
use futures::Stream;
use std::pin::Pin;
use std::sync::Arc;

/// Handler receiving bus inputs for one topic.
pub type MessageHandler = Arc<dyn Fn(Message) + Send + Sync>;

/// Handler serving one control RPC.
pub type RpcHandler = Arc<dyn Fn(ControlRequest) -> BoxFuture<'static, ControlResponse> + Send + Sync>;

/// Stream of periodic ticks.
pub type Timer = Pin<Box<dyn Stream<Item = ()> + Send>>;

/// Removes a subscription or RPC registration when dropped.
#[derive(Default)]
pub struct Subscription {
    cancel: Option<Box<dyn FnOnce() + Send>>,
}

impl Subscription {
    /// A handle with no teardown, useful for registrations that live as long as
    /// the process does.
    pub fn permanent() -> Self {
        Self::default()
    }

    pub(crate) fn on_cancel(cancel: impl FnOnce() + Send + 'static) -> Self {
        Self {
            cancel: Some(Box::new(cancel)),
        }
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        if let Some(cancel) = self.cancel.take() {
            cancel();
        }
    }
}

/// The bus abstraction the driver is written against.
///
/// Adapters translate domain messages to and from their bus. Implementations
/// must be cheaply shareable; the driver holds them behind an [`Arc`].
pub trait Transport: Send + Sync {
    /// Publishes a domain message on `topic`.
    fn publish(&self, topic: &str, message: Message) -> crate::Result<()>;

    /// Whether anything on the bus consumes `topic`. Sensor and TF work is
    /// skipped while this is `false`.
    fn has_consumers(&self, topic: &str) -> bool;

    /// Registers `handler` for bus inputs on `topic`.
    fn subscribe(&self, topic: &str, handler: MessageHandler) -> Subscription;

    /// Registers `handler` for the control RPC `name`.
    fn register_rpc(&self, name: &str, handler: RpcHandler) -> Subscription;

    /// A stream of ticks at `hz`.
    fn timer(&self, hz: f32) -> Timer;

    /// The transport clock, used to stamp messages.
    fn now(&self) -> Timestamp;
}

pub mod memory;

pub use memory::MemoryTransport;
