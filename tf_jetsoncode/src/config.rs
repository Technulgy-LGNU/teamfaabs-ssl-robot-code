use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fs;
use std::net::Ipv4Addr;
use std::path::Path;

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
  pub robot_id: u8,
  pub cp_config: CrashPilotConfig,
  pub onboard_vision_socket_path: String,
  pub teensy: TeensyConfig,
}
impl Default for Config {
  fn default() -> Self {
    Self {
      robot_id: 1,
      cp_config: CrashPilotConfig::default(),
      onboard_vision_socket_path: "/tmp/ov_socket".to_string(),
      teensy: Default::default(),
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct CrashPilotConfig {
  pub host: Ipv4Addr,
  pub port: u16,
  pub port_outgoing: u16,
}
impl Default for CrashPilotConfig {
  fn default() -> Self {
    Self {
      host: Ipv4Addr::new(10, 0, 64, 221),
      port: 1024,
      port_outgoing: 2048,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct TeensyConfig {
  pub vid: u16,
  pub pid: u16,
}
impl Default for TeensyConfig {
  fn default() -> Self {
    Self {
      vid: 0x16C0,
      pid: 0x0486,
    }
  }
}

pub fn load_or_create_config(path: &str) -> Result<Config, Box<dyn Error>> {
  if !Path::new(path).exists() {
    let default_config = Config::default();

    let toml_string = toml::to_string_pretty(&default_config)?;
    fs::write(path, toml_string)?;

    return Ok(default_config);
  }

  let content = fs::read_to_string(path)?;
  let config: Config = toml::from_str(&content)?;

  Ok(config)
}
