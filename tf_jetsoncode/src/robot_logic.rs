use crate::communication::{send_flags, TeensySendMsg, VisionMsg};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::orca::{OrcaHandle, OrcaRequest, Vec2i, WorldSnapshot};

pub mod orca;
pub mod goalie;


pub fn command(cfg: &config::Config, cp_data: &CpRobot, orca: &OrcaHandle, world: &WorldSnapshot, vision_data: &VisionMsg, mut msg: TeensySendMsg) -> TeensySendMsg {
  let mut robot_self: CpTrackedRobot = Default::default();
  if cfg.robot_team == "yellow"  {
    robot_self = *cp_data.robots_yellow.iter().find(|r| r.robot_id == cfg.robot_id as u32).unwrap_or_else(|| {
        return &robot_self;
    });
  } else if cfg.robot_team == "blue" {
    robot_self = *cp_data.robots_blue.iter().find(|r| r.robot_id == cfg.robot_id as u32).unwrap_or_else(|| {
        return &robot_self;
    });
  } else {
    panic!("Unknown team: {}", cfg.robot_team);
  }

  match cp_data.cmd.task {
    0 => {
      // UNKNOWN
      println!("UNKNOWN");
    },
    1 =>  {
      // Drive to pos

      let intent = orca::NavIntent::GoToPosition {
        target_pos_mm: Vec2i {
          x: cp_data.cmd.pos.unwrap_or_default().x,
          y: cp_data.cmd.pos.unwrap_or_default().y,
        },
        max_speed_mm_s: cp_data.cmd.speed.unwrap_or(1500),
      };

      orca.publish(OrcaRequest {
        world: world.clone(),
        intent,
      })
    },
    2 => {
      // Kick in kick dir

      // First rotate robot
      if (robot_self.orientation - cp_data.cmd.kick_orient.unwrap_or_default() as i32).abs() > 5 {
        msg.orient = cp_data.cmd.kick_orient.unwrap_or_default() as u16;
      } else {
        msg.kick_pwr = cp_data.cmd.kick_speed.unwrap_or_default() as u8;
        msg.set_flag(send_flags::KICK);
      }

    },
    3 => {
      // Chip in kick dir
    },
    4 => {
      // Rec Kick
    },
    5 => {
      // Steal Ball
    },
    6 => {
      // Dribble the Ball
    },
    7 => {
      // Position the Ball
    },
    9 => {
      // Kickoff
    },
    11 => {
      // Free kick
    },
    _ => {
      println!("Unknown task: {}", cp_data.cmd.task);
    }
  }


  msg
}
