use std::time::Duration;
use crate::communication::communication_receiver;
use crate::communication::send_cp::send_cp;
use crate::proto::RobotCp;

mod proto;
mod communication;
mod config;

#[tokio::main]
async fn main() {
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
  let upd_socket = match tokio::net::UdpSocket::bind(format!("0.0.0.0:{}", config.cp_config.port_outgoing)).await {
    Ok(s) => s,
    Err(e) => {
      panic!("Failed to create udp socket for sending cp data: {}", e);
    }
  };


  // Starting robot
  println!("Starting robot ...");
  // Data Packets
  let mut cp_data: proto::CpRobot = Default::default();
  let mut vision_data: communication::VisionMsg = Default::default();
  let mut teensy_data: communication::TeensyRecMSG = Default::default();

  // The rest of the code should not depend on
  // late packets, so we use tokio::time::tick to
  // have predictably program time

  let mut tick = tokio::time::interval(Duration::from_millis(8)); // ~120 Hz
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


    // At the end of the loop, send cp update data
    let cp_update_data: RobotCp = RobotCp {
      robot_id: config.robot_id as u32,
      battery_voltage: Some(teensy_data.batt_volt),
      kicker_ready: teensy_data.kick_ready,
      has_ball: teensy_data.has_ball,
      error_msg: None,
      acting: None,
      last_rec_packet: Some(cp_data.packet_id),
    };
    send_cp(&config, &upd_socket, cp_update_data).await;
  }
}
