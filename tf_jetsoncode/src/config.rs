use std::error::Error;
use std::fs;
use serde::{Deserialize, Serialize};
use std::net::Ipv4Addr;
use std::path::Path;

#[derive(Debug, Deserialize, Serialize, Default)]
pub struct Config {
  pub robot_id: u8,
  pub cp_config: CrashPilotConfig,
}



#[derive(Debug, Deserialize, Serialize)]
pub struct CrashPilotConfig {
  host: Ipv4Addr,
  port: u16,
}
impl Default for CrashPilotConfig {
  fn default() -> Self {
    Self {
      host: Ipv4Addr::new(10, 0, 64, 221),
      port: 8192,
    }
  }
}

pub fn load_or_create_config(path: &str, ) -> Result<Config, Box<dyn Error>> {
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

