use crate::communication::{TeensySendMsg, VisionMsg};
use crate::proto::CpRobot;
use crate::robot_logic::orca::{OrcaHandle, WorldSnapshot};

pub fn goalie(cp_data: &CpRobot, orca: &OrcaHandle, world: &WorldSnapshot, vision_data: &VisionMsg, msg: TeensySendMsg) -> TeensySendMsg {
  msg
}