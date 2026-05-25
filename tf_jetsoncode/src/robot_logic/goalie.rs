use crate::communication::{TeensySendMsg, VisionMsg};
use crate::proto::CpRobot;

#[inline]
pub fn goalie(cp_data: &CpRobot, vision_data: &VisionMsg, msg: TeensySendMsg) -> TeensySendMsg {
  msg
}
