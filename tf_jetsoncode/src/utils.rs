use crate::communication;
use crate::communication::{EventShare, TeensyOut};
use core_dump::proto;

pub struct CommunicationChannels {
  pub rx: EventShare,
  pub tx: TeensyOut,
  pub udp_socket: tokio::net::UdpSocket,
}

pub struct PacketBuffer {
  pub cp_data: proto::CpRobot,
  pub vision_data: communication::VisionMsg,
  pub teensy_data: communication::TeensyRecMSG,
  pub robot_msg: communication::TeensySendMsg,
  pub robot_self: proto::CpTrackedRobot,
}

impl PacketBuffer {
  pub fn clear(&mut self) {
    self.cp_data = proto::CpRobot::default();
    self.vision_data = communication::VisionMsg::default();
    self.teensy_data = communication::TeensyRecMSG::default();
    self.robot_msg = communication::TeensySendMsg::default();
    self.robot_self = proto::CpTrackedRobot::default();
  }

  pub fn new() -> Self {
    Self {
      cp_data: proto::CpRobot::default(),
      vision_data: communication::VisionMsg::default(),
      teensy_data: communication::TeensyRecMSG::default(),
      robot_msg: communication::TeensySendMsg::default(),
      robot_self: proto::CpTrackedRobot::default(),
    }
  }
}
