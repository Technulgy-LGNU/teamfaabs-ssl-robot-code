use crate::communication::{TeensySendMsg, VisionMsg};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};

#[inline]
pub fn goalie(cfg: &config::Config, cp_data: &CpRobot, robot_self: &CpTrackedRobot, vision_data: &VisionMsg, msg: TeensySendMsg) -> TeensySendMsg {
  msg
}
