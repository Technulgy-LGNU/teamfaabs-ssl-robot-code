use crate::communication::{communication_receiver, send_flags};
use crate::communication::send_cp::send_cp;
use crate::proto::RobotCp;
use crate::robot_logic::command;
use crate::robot_logic::goalie::goalie;
use std::time::Duration;
use tracing::info;

mod communication;
mod config;
mod proto;
mod robot_logic;

// Constants
const TEENSY_SEND_MSG_SIZE: usize = 17;
const TEENSY_RECEIVE_MSG_SIZE: usize = 6;
const DEFAULT_ACCEL_MM_S2: f32 = 2_800.0;
const DEFAULT_DECEL_MM_S2: f32 = 3_800.0;

#[tokio::main]
async fn main() {
  // Start tracing
  tracing_subscriber::fmt()
    .with_ansi(true)
    .init();

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
    match tokio::net::UdpSocket::bind(format!("0.0.0.0:{}", config.cp_config.port_outgoing+2))
      .await
    {
      Ok(s) => s,
      Err(e) => {
        panic!("Failed to create udp socket for sending cp data: {}", e);
      }
    };

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
        .unwrap_or( &robot_self );
    } else if config.robot_team.as_str() == "blue" {
      robot_self = *cp_data
        .robots_blue
        .iter()
        .find(|r| r.robot_id == config.robot_id as u32)
        .unwrap_or( &robot_self );
    } else {
      panic!("Unknown team: {}", config.robot_team);
    }

    // Buttons
    // React to button presses
    for i in 0..15 {
      if teensy_data.button(i) {
        println!("Button {} pressed", i);
      }
    }

    info!("\x1b[32m=================\x1b[0m");
    info!("Incoming CP_Data: {:?}", cp_data);
    info!("\x1b[32m=================\x1b[0m");

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
        robot_msg = command(&config, &cp_data, &vision_data, robot_msg, true, robot_self).await;

        let mut orient = robot_self.orientation % 360;
        while orient.is_negative() {
          orient += 360;
        }
        robot_msg.self_orient = orient as u16;
        robot_msg.orient = cp_data.cmd.orientation.unwrap_or_default() as u16;
      }
      3 => {
        // Free to listen to commands
        robot_msg = command(&config, &cp_data, &vision_data, robot_msg, false, robot_self).await;

        let mut orient = robot_self.orientation % 360;
        while orient.is_negative() {
          orient += 360;
        }
        robot_msg.self_orient = orient as u16;

        // Check if self_orient != cp_data.cmd.orientation, if so, gradually rotate the robot, dependent on the distance
        // to the end position, if command == pos, if not, just rotate
        if cp_data.cmd.task == 1 {
          // Check if orientation matches
          if orient != cp_data.cmd.orientation.unwrap_or_default() as i32 {
            // Distance to point

          }
        } else {
          robot_msg.orient = cp_data.cmd.orientation.unwrap_or_default() as u16;
        }
      }
      4 => {
        // Goalie, move into penalty area and protect the goal
        robot_msg = goalie(&config, &cp_data, &robot_self, &vision_data, robot_msg);
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


    // Print data for testing
    info!("Direction: {:?}", robot_msg.dir);
    info!("Speed: {:?}", robot_msg.speed);
    info!("Orientation: {:?}", robot_msg.orient);
    info!("Self Dir: {:?}", robot_msg.self_orient);

    // Print Self velocity in mm/s
    //info!("Self Velocity: {:?}", ((robot_self.vel.unwrap_or_default().x*robot_self.vel.unwrap_or_default().x+robot_self.vel.unwrap_or_default().y*robot_self.vel.unwrap_or_default().y) as f32).sqrt());


    let buf = robot_msg.encode();
    tx.publish(buf).await;

    // At the end of the loop, send cp update data
    let cp_update_data: RobotCp = RobotCp {
      robot_id: config.robot_id as u32,
      battery_voltage: Some(teensy_data.batt_level as f32),
      kicker_ready: teensy_data.kick_ready() && teensy_data.chip_ready(),
      has_ball: teensy_data.has_ball(),
      error_msg: if teensy_data.error() {
        Some("Teensy reported an error".to_string())
      } else {
        None
      },
      acting: Some(true),
      last_rec_packet: Some(cp_data.packet_id),
    };
    send_cp(&config, &upd_socket, cp_update_data).await;
  }
}
