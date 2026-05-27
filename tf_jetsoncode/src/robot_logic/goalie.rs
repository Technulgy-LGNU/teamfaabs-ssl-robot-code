use crate::communication::{TeensySendMsg, VisionMsg};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};

pub fn goalie(cfg: &config::Config, cp_data: &CpRobot, robot_self: &CpTrackedRobot, _vision: &VisionMsg, msg: TeensySendMsg) ->  TeensySendMsg {



  msg
}
