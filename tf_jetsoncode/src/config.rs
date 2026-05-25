use std::error::Error;
use std::fs;
use serde::{Deserialize, Serialize};
use std::net::Ipv4Addr;
use std::path::Path;

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
  pub robot_id: u8,
  pub robot_team: String,
  pub cp_config: CrashPilotConfig,
  pub onboard_vision_socket_path: String,
  pub teensy: TeensyConfig,
  pub field: FieldConfig,
}
impl Default for Config {
  fn default() -> Self {
    Self {
      robot_id: 1,
      robot_team: "yellow".to_string(),
      cp_config: CrashPilotConfig::default(),
      onboard_vision_socket_path: "/tmp/ov_socket".to_string(),
      teensy: Default::default(),
      field: Default::default(),
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
      port_outgoing: 4096,
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
      pid: 0x0480,
    }
  }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct FieldConfig {
  // General field config, everything in mm
  width: u32,
  height: u32,
  runoff_width: u32,
  penalty_area_width: u32,
  penalty_area_height: u32,
  goal_width: u32,
}
impl Default for FieldConfig {
  fn default() -> Self {
    // Default values for DIV-B
    Self {
      width: 9000,
      height: 6000,
      runoff_width: 300,
      penalty_area_width: 2000,
      penalty_area_height: 1000,
      goal_width: 1600,
    }
  }
}

impl FieldConfig {
  #[inline]
  pub fn width_mm(&self) -> f32 {
    self.width as f32
  }

  #[inline]
  pub fn height_mm(&self) -> f32 {
    self.height as f32
  }

  #[inline]
  pub fn runoff_width_mm(&self) -> f32 {
    self.runoff_width as f32
  }

  #[inline]
  pub fn penalty_area_width_mm(&self) -> f32 {
    self.penalty_area_width as f32
  }

  #[inline]
  pub fn penalty_area_height_mm(&self) -> f32 {
    self.penalty_area_height as f32
  }

  #[inline]
  pub fn goal_width_mm(&self) -> f32 {
    self.goal_width as f32
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

