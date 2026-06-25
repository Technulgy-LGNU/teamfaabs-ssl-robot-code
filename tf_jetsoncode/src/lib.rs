use crate::communication::communication_receiver;
use crate::communication::send_cp::send_cp;
pub use crate::communication::{Events, TeensyRecMSG, TeensySendMsg, send_flags};
pub use crate::config::Config;
use crate::robot_logic::helpers::{
  allow_own_penalty_area, ball_avoidance_margin_mm, outside_field,
};
use crate::robot_logic::orca::{
  NavIntent, Orca, OrcaParams, OrcaRequest, WorldSnapshot, nav_command_to_teensy,
};
use crate::robot_logic::vec::Vec2f;
use crate::utils::{CommunicationChannels, PacketBuffer};
pub use core_dump::proto::{
  CpBall, CpCommand, CpRobot, CpState, CpTrackedRobot, CpVector2, RobotCp,
};
use std::time::Duration;
use tracing::info;

mod communication;
mod config;
mod robot_logic;
mod utils;

// Constants
const TEENSY_SEND_MSG_SIZE: usize = 17;
const TEENSY_RECEIVE_MSG_SIZE: usize = 7;
const DEFAULT_ACCEL_MM_S2: u32 = 4_000;
const DEFAULT_DECEL_MM_S2: u32 = 6_000;

pub struct Robot<C = CommunicationChannels> {
  config: Config,
  params: OrcaParams,
  orca: Orca,
  was_goalie: bool,
  packets: PacketBuffer,
  comm: C,
}

impl Robot {
  pub async fn default() -> Self {
    // Get config
    let config = match config::load_or_create_config("config.toml") {
      Ok(config) => config,
      Err(e) => panic!("{}", e),
    };

    // Get communication channels
    let communication = match communication_receiver(&config) {
      Ok(communication) => communication,
      Err(e) => panic!("{}", e),
    };

    let rx = communication.events;
    let tx = communication.teensy;

    // Udp Socket to send data back to the CrashPilot
    let udp_socket =
      match tokio::net::UdpSocket::bind(format!("0.0.0.0:{}", config.cp_config.port_outgoing + 2))
        .await
      {
        Ok(s) => s,
        Err(e) => {
          panic!("Failed to create udp socket for sending cp data: {}", e);
        }
      };

    let comm = CommunicationChannels { rx, tx, udp_socket };

    Self::from_parts(config, comm)
  }

  pub async fn recv(&mut self) {
    // Drain the latest state from each channel
    let events = {
      let lock = self.comm.rx.read().await;

      lock.clone().take()
    };

    self.interpret(events);
  }

  pub async fn run(&mut self) {
    let mut tick = tokio::time::interval(Duration::from_millis(2)); // 500 Hz
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    loop {
      tick.tick().await;

      self.step().await;
    }
  }

  pub async fn step(&mut self) {
    self.recv().await;
    self.update();
    self.send().await;
  }

  pub async fn send(&mut self) {
    let buf = self.packets.robot_msg.encode();
    self.comm.tx.publish(buf).await;

    // At the end of the loop, send cp update data
    let cp_update_data = self.cp_packet();

    send_cp(&self.config, &self.comm.udp_socket, cp_update_data).await;
  }
}

impl<C: Default> Robot<C> {
  pub fn new(config: Config) -> Self {
    Self::from_parts(config, C::default())
  }
}

impl<C> Robot<C> {
  pub fn from_parts(config: Config, comm: C) -> Self {
    let params = OrcaParams {
      time_horizon_ms: 200,
      safety_margin_mm: 30,
      default_robot_radius_mm: 90,
      time_step_ms: 2,
      responsibility: 0.8,
      max_accel_mm_s2: DEFAULT_ACCEL_MM_S2,
      max_decel_mm_s2: DEFAULT_DECEL_MM_S2,
    };

    let orca = Orca::new(params);

    Self {
      config,
      orca,
      params,
      was_goalie: false,
      packets: PacketBuffer::new(),
      comm,
    }
  }

  pub fn interpret(&mut self, events: Events) {
    if let Some(packet) = events.cp {
      self.packets.cp_data = packet;
    }
    if let Some(packet) = events.vis {
      self.packets.vision_data = packet;
    }
    if let Some(packet) = events.teensy {
      self.packets.teensy_data = packet;
    }

    // Checks if the config robot_id is the same as the one send by the crashpilot
    if self.packets.cp_data != Default::default() {
      assert_eq!(self.config.robot_id, self.packets.cp_data.robot_id as u8);
    }

    // Self
    if !self.packets.cp_data.infos.team_color {
      self.packets.robot_self = *self
        .packets
        .cp_data
        .robots_yellow
        .iter()
        .find(|r| r.robot_id == self.config.robot_id as u32)
        .unwrap_or(&self.packets.robot_self);
    } else if self.packets.cp_data.infos.team_color {
      self.packets.robot_self = *self
        .packets
        .cp_data
        .robots_blue
        .iter()
        .find(|r| r.robot_id == self.config.robot_id as u32)
        .unwrap_or(&self.packets.robot_self);
    } else {
      panic!("Unknown team: {}", self.packets.cp_data.infos.team_color);
    }
  }

  pub fn cp_packet(&self) -> RobotCp {
    RobotCp {
      robot_id: self.config.robot_id as u32,
      battery_voltage: Some(self.packets.teensy_data.batt_level as u32),
      current: Some(self.packets.teensy_data.current as u32),
      kicker_ready: self.packets.teensy_data.kick_ready() && self.packets.teensy_data.chip_ready(),
      has_ball: self.packets.teensy_data.has_ball(),
      has_error: if self.packets.teensy_data.error() {
        Some(true)
      } else {
        None
      },
      acting: Some(true),
      last_rec_packet: Some(self.packets.cp_data.packet_id),
      timestamp: self.packets.cp_data.timestamp,
    }
  }

  pub fn step_with_data(&mut self, events: Events) -> (TeensySendMsg, RobotCp) {
    self.interpret(events);
    self.update();

    let cp_update_data = self.cp_packet();
    (self.packets.robot_msg, cp_update_data)
  }

  pub fn update(&mut self) {
    // Orca
    let world = WorldSnapshot::from_cp(
      &self.packets.cp_data,
      &self.packets.robot_self,
      self.params.default_robot_radius_mm,
      ball_avoidance_margin_mm(&self.packets.cp_data, self.packets.robot_self),
      allow_own_penalty_area(&self.packets.cp_data),
    );

    // Buttons
    // React to button presses
    for i in 0..15 {
      if self.packets.teensy_data.button(i) {
        println!("Button {} pressed", i);
      }
    }

    // Clear all flags
    self.packets.robot_msg.clear_all_flags();

    // Correctly produce right self_orient, for calibration
    let mut orient = self.packets.robot_self.orientation % 360;
    while orient.is_negative() {
      orient += 360;
    }
    self.packets.robot_self.orientation = orient;
    self.packets.robot_msg.self_orient = self.packets.robot_self.orientation as u16;
    // Game Logic
    match CpState::try_from(self.packets.cp_data.cmd.state).unwrap_or(CpState::StateUnspecified) {
      CpState::StateUnspecified => {
        info!("UNKNOWN");
        self.packets.robot_msg.set_flag(send_flags::ERROR);
      }
      CpState::StateHalt => {
        // Robot is not allowed to move
        let nav_command = self.orca.step(OrcaRequest {
          world,
          intent: NavIntent::Stop,
        });
        nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
      }
      CpState::StateStop => {
        // Robot is allowed to move with a max speed of
        // 1,5m/s (1500mm/s) & stay away from ball 500mm
        if self.was_goalie {
          self.command(&world, true);
        } else {
          self.goalie(&world);
        }

        self.packets.robot_msg.self_orient = orient as u16;
        self.packets.robot_msg.orient =
          self.packets.cp_data.cmd.orientation.unwrap_or_default() as u16;
      }
      CpState::StateFree => {
        // Free to listen to commands
        self.was_goalie = false;

        self.command(&world, false);
      }
      CpState::StateGoalie => {
        // Goalie, move into penalty area and protect the goal
        self.was_goalie = true;
        self.goalie(&world);
      }
      CpState::StateSubstitute => {
        // Substitute
        // HALT
        let nav_command = self.orca.step(OrcaRequest {
          world,
          intent: NavIntent::Stop,
        });
        nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
      }
    }

    // Led's
    // Depending on different states, set the led's on the mainboard

    // After logic, send new robot command
    self.packets.robot_msg.state = self.packets.cp_data.cmd.state as u8;
    self.packets.robot_msg.vel_x = self.packets.robot_self.vel.unwrap_or_default().x as i16;
    self.packets.robot_msg.vel_y = self.packets.robot_self.vel.unwrap_or_default().y as i16;

    self.packets.robot_msg.dir = self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as u16;
    self.packets.robot_msg.orient = self.packets.cp_data.cmd.orientation.unwrap_or_default() as u16;
    self.packets.robot_msg.speed = self.packets.cp_data.cmd.speed.unwrap_or_default() as u16;

    // Do last check, if robot is out of field, if yes, stop && checks if the robot is visibly in the vision
    if outside_field(
      &self.packets.cp_data.infos,
      Vec2f::new_from_cp(self.packets.robot_self.pos),
    ) || self.packets.robot_self.visibility <= 20
    {
      self.packets.robot_msg.speed = 0;
    }
  }
}
