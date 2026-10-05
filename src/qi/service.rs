//! Port to a remote QI service.

use super::value::adapt_args;
use crate::Result;
use async_trait::async_trait;
use qi::object::{MemberIdent, Object, ObjectExt};
use qi::value::Value;

/// A callable QI service, e.g. one of the AL* modules of the robot.
///
/// The typed proxies in [`crate::qi::services`] build their arguments as plain
/// values; implementations encode them for the wire.
#[async_trait]
pub trait Service: Send + Sync {
    /// Calls `method` and decodes its return value.
    async fn call(&self, method: &str, args: Value<'static>) -> Result<Value<'static>>;

    /// Fires `method` without waiting for a reply ("async" in NAOqi terms).
    async fn post(&self, method: &str, args: Value<'static>) -> Result<()>;
}

/// Adapter exposing a service handle resolved through a [`qi::Node`].
pub struct ObjectService<O> {
    client: O,
}

impl<O> ObjectService<O>
where
    O: Object + Sync,
{
    pub fn new(client: O) -> Self {
        Self { client }
    }

    /// Encodes `args` as the remote method's signature demands.
    fn encode(&self, method: &str, args: Value<'static>) -> Value<'static> {
        let ident = MemberIdent::from(method);
        match self
            .client
            .meta()
            .method(&ident)
            .and_then(|method| method.parameters_signature.to_type())
        {
            Some(parameters) => adapt_args(args, parameters),
            // Unknown method: pass through, the call below reports the error.
            None => args,
        }
    }
}

#[async_trait]
impl<O> Service for ObjectService<O>
where
    O: Object + Clone + Send + Sync,
{
    async fn call(&self, method: &str, args: Value<'static>) -> Result<Value<'static>> {
        let args = self.encode(method, args);
        Ok(ObjectExt::call::<Value<'static>, _, _>(&self.client, method, args).await?)
    }

    async fn post(&self, method: &str, args: Value<'static>) -> Result<()> {
        let args = self.encode(method, args);
        // Fire-and-forget: failures are logged by the client, no result exists.
        self.client.meta_post(MemberIdent::from(method), args).await;
        Ok(())
    }
}
