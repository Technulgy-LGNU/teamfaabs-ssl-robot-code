use crate::communication::{TeensySendMsg, VisionMsg};
use crate::proto::CpRobot;

#[inline]
pub fn goalie(_cp_data: &CpRobot, _vision_data: &VisionMsg, msg: TeensySendMsg) -> TeensySendMsg {
  msg
}
