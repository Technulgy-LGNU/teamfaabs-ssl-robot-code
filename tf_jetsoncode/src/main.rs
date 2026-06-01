use crate::communication::send_cp::send_cp;
use crate::communication::{communication_receiver, send_flags};
use crate::proto::{CpState, RobotCp};
use crate::robot_logic::command;
use crate::robot_logic::goalie::goalie;
use crate::robot_logic::helpers::{
  Vec2f, allow_own_penalty_area, ball_avoidance_margin_mm, inside_field,
};
use crate::robot_logic::orca::{OrcaHandle, OrcaParams, WorldSnapshot};
use std::time::Duration;
use tracing::info;

mod communication;
mod config;
mod proto;
mod robot_logic;

// Constants
const TEENSY_SEND_MSG_SIZE: usize = 17;
const TEENSY_RECEIVE_MSG_SIZE: usize = 8;
const DEFAULT_ACCEL_MM_S2: u32 = 4_000;
const DEFAULT_DECEL_MM_S2: u32 = 6_000;

#[tokio::main]
async fn main() {
  // Start tracing
  tracing_subscriber::fmt().with_ansi(true).init();

  // Get config
  let config = match config::load_or_create_config("config.toml") {
    Ok(config) => config,
    Err(e) => panic!("{}", e),
  };

  // Get communication channels
  let communication = match communication_receiver(&config).await {
    Ok(communication) => communication,
    Err(e) => panic!("{}", e),
  };

  let rx = communication.events;
  let tx = communication.teensy;

  // Udp Socket to send data back to the CrashPilot
  let upd_socket =
    match tokio::net::UdpSocket::bind(format!("0.0.0.0:{}", config.cp_config.port_outgoing + 2))
      .await
    {
      Ok(s) => s,
      Err(e) => {
        panic!("Failed to create udp socket for sending cp data: {}", e);
      }
    };

  // Orca Params & Handlers
  let params = OrcaParams {
    time_horizon_ms: 500,
    safety_margin_mm: 30,
    default_robot_radius_mm: 90,
    time_step_ms: 1,
    responsibility: 2.0,
    max_accel_mm_s2: DEFAULT_ACCEL_MM_S2,
    max_decel_mm_s2: DEFAULT_DECEL_MM_S2,
    run_blocking: true,
  };
  let orca = OrcaHandle::spawn(params);

  // Starting robot
  info!("Starting robot ...");
  // Data Packets
  let mut cp_data: proto::CpRobot = Default::default();
  let mut vision_data: communication::VisionMsg = Default::default();
  let mut teensy_data: communication::TeensyRecMSG = Default::default();
  let mut robot_msg: communication::TeensySendMsg = Default::default();
  let mut robot_self: proto::CpTrackedRobot = Default::default();

  // The rest of the code should not depend on
  // late packets, so we use tokio::time::tick to
  // have predictably program time

  let mut tick = tokio::time::interval(Duration::from_millis(4)); // ~240 Hz
  tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

  loop {
    tick.tick().await;

    // Drain the latest state from each channel
    let (cp, vis, teensy) = {
      let mut lock = rx.lock().await;
      (lock.0.take(), lock.1.take(), lock.2.take())
    };

    if let Some(packet) = cp {
      cp_data = packet;
    }
    if let Some(packet) = vis {
      vision_data = packet;
    }
    if let Some(packet) = teensy {
      teensy_data = packet;
    }

    // Self
    if config.robot_team.as_str() == "yellow" {
      robot_self = *cp_data
        .robots_yellow
        .iter()
        .find(|r| r.robot_id == config.robot_id as u32)
        .unwrap_or(&robot_self);
    } else if config.robot_team.as_str() == "blue" {
      robot_self = *cp_data
        .robots_blue
        .iter()
        .find(|r| r.robot_id == config.robot_id as u32)
        .unwrap_or(&robot_self);
    } else {
      panic!("Unknown team: {}", config.robot_team);
    }

    // Orca
    let world = WorldSnapshot::from_cp(
      &config,
      &cp_data,
      &robot_self,
      params.default_robot_radius_mm,
      ball_avoidance_margin_mm(&cp_data),
      allow_own_penalty_area(&cp_data),
    );

    // Buttons
    // React to button presses
    for i in 0..15 {
      if teensy_data.button(i) {
        println!("Button {} pressed", i);
      }
    }

    // Clear all flags
    robot_msg.clear_all_flags();

    // Correctly produce right self_orient, for calibration
    let mut orient = robot_self.orientation % 360;
    while orient.is_negative() {
      orient += 360;
    }
    robot_self.orientation = orient;
    robot_msg.self_orient = robot_self.orientation as u16;
    // Game Logic
    match cp_data.cmd.state {
      0 => {
        info!("UNKNOWN");
        robot_msg.set_flag(send_flags::ERROR);
      }
      1 => {
        // Robot is not allowed to move
        robot_msg.speed = 0;
      }
      2 => {
        // Robot is allowed to move with a max speed of
        // 1,5m/s (1500mm/s) & stay away from ball 500mm
        robot_msg = command(
          &cp_data,
          &vision_data,
          &orca,
          &world,
          robot_msg,
          true,
          robot_self,
        );

        robot_msg.self_orient = orient as u16;
        robot_msg.orient = cp_data.cmd.orientation.unwrap_or_default() as u16;
      }
      3 => {
        // Free to listen to commands
        robot_msg = command(
          &cp_data,
          &vision_data,
          &orca,
          &world,
          robot_msg,
          false,
          robot_self,
        )
      }
      4 => {
        // Goalie, move into penalty area and protect the goal
        robot_msg = goalie(
          &config,
          &cp_data,
          &robot_self,
          &vision_data,
          &orca,
          &world,
          robot_msg,
        );
      }
      5 => {
        // Substitute
        // HALT
        robot_msg.speed = 0;
      }
      _ => {}
    }

    // Led's
    // Depending on different states, set the led's on the mainboard

    // After logic, send new robot command
    robot_msg.state = cp_data.cmd.state as u8;
    robot_msg.vel_x = robot_self.vel.unwrap_or_default().x as i16;
    robot_msg.vel_y = robot_self.vel.unwrap_or_default().y as i16;

    // Do last check, if robot is out of field, if yes, stop
    if inside_field(&config, Vec2f::new_from_cp(robot_self.pos)) || robot_self.visibility <= 20 {
      robot_msg.speed = 0;
    }

    let buf = robot_msg.encode();
    tx.publish(buf).await;

    // At the end of the loop, send cp update data
    let cp_update_data: RobotCp = RobotCp {
      robot_id: config.robot_id as u32,
      battery_voltage: Some(teensy_data.batt_level as u32),
      current: Some(teensy_data.current as u32),
      kicker_ready: teensy_data.kick_ready() && teensy_data.chip_ready(),
      has_ball: teensy_data.has_ball(),
      has_error: if teensy_data.error() {
        Some(true)
      } else {
        None
      },
      acting: Some(true),
      last_rec_packet: Some(cp_data.packet_id),
    };
    send_cp(&config, &upd_socket, cp_update_data).await;
  }
}
