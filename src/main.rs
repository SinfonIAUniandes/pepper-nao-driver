//! Driver binary: connects to Pepper and runs the capability set.

use al_robot_driver_rs::capabilities;
use al_robot_driver_rs::driver::{self, Driver, Options};
use al_robot_driver_rs::transport::{Transport, memory::MemoryTransport};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
use clap::Parser;
use std::path::PathBuf;
use std::sync::Arc;

#[derive(Debug, Parser)]
#[command(version, about)]
struct Args {
    /// Address of the robot's QI space.
    #[arg(long, default_value = "tcp://127.0.0.1:9559")]
    qi_address: qi::Address,

    /// Instance prefix reported by `_whoWillWin`.
    #[arg(long)]
    instance_prefix: String,

    /// Also stream `odom -> base_link` on `tf`.
    #[arg(long)]
    publish_odom: bool,

    /// Base directory holding the `share/` assets.
    #[arg(long)]
    assets: Option<PathBuf>,
}

/// The bus adapter plugs in here; the driver only sees the trait.
fn transport() -> Arc<dyn Transport> {
    Arc::new(MemoryTransport::new())
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()),
        )
        .init();

    let args = Args::parse();
    let options = Options {
        qi_address: args.qi_address,
        instance_prefix: args.instance_prefix,
        publish_odom: args.publish_odom,
        assets_base: args.assets,
    };

    let connected = driver::connect(&options, transport()).await?;
    let driver = Driver::new(connected.ctx, capabilities::default_capabilities());
    driver.start().await?;
    tracing::info!("driver ready");

    tokio::signal::ctrl_c().await?;
    tracing::info!("shutting down");
    driver.shutdown().await?;
    Ok(())
}
