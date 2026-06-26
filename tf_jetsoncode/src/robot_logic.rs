use crate::communication::send_flags;
use crate::robot_logic::orca::{
  nav_command_to_teensy, NavIntent, OrcaRequest, Vec2i, WorldSnapshot,
};
use crate::robot_logic::vec::{distance_cpv, Vec2f};
use crate::Robot;
use core_dump::proto::CpTask;

mod defense;
mod get_ball;
pub mod goalie;
pub mod helpers;
pub mod orca;
mod receive_ball;
pub mod vec;

// If we are inside this distance in the penalty area, stop using raw motion.
pub(crate) const RAW_STOP_RADIUS_MM: f32 = 40f32;
// Maximum translational speed for raw goalie movement inside the penalty area.
pub(crate) const RAW_MAX_SPEED_MM_S: f32 = 4_000f32;

impl<C> Robot<C> {
  #[inline]
  pub fn command(&mut self, world: &WorldSnapshot, stop: bool) {
    // Vars
    let robot_pos = Vec2f::new_from_cp(self.packets.robot_self.pos);
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);
    let ball_vel = Vec2f::new_from_cp(self.packets.cp_data.ball.vel.unwrap_or_default());
    let mut has_kicked: bool = false;

    match CpTask::try_from(self.packets.cp_data.cmd.task).unwrap_or(CpTask::TaskUnspecified) {
      CpTask::TaskUnspecified => {
        // UNKNOWN
        println!("UNKNOWN");
        self.packets.robot_msg.set_flag(send_flags::ERROR);
      }
      CpTask::TaskPos => {
        has_kicked = false;
        // Speed check
        let max_speed_mm_s = if self.packets.cp_data.cmd.speed > Some(1500) && stop {
          1500
        } else {
          self.packets.cp_data.cmd.speed.unwrap_or_default()
        };

        // Check if near of pos, and then stop
        if distance_cpv(
          self.packets.robot_self.pos,
          self.packets.cp_data.cmd.pos.unwrap_or_default(),
        ) < 60.0
        {
          let intent = NavIntent::Stop;
          let nav_command = self.orca.step(OrcaRequest {
            intent,
            world: world.clone(),
          });
          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        } else {
          let nav_intent = NavIntent::GoToPosition {
            target_pos_mm: Vec2i::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default()),
            max_speed_mm_s,
          };
          let nav_command = self.orca.step(OrcaRequest {
            intent: nav_intent,
            world: world.clone(),
          });
          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        }

        self.packets.robot_msg.orient =
          self.packets.cp_data.cmd.orientation.unwrap_or_default() as u16;
      }
      CpTask::TaskKick => {
        // Kick in kick dir
        // First rotate robot
        if (self.packets.robot_self.orientation
          - self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as i32)
          .abs()
          > 5
        {
          // If we are facing the right direction (variance of five degrees)
          self.packets.robot_msg.orient =
            self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as u16;
        } else {
          self.packets.robot_msg.kick_pwr =
            self.packets.cp_data.cmd.kick_speed.unwrap_or_default() as u8;
          self.packets.robot_msg.set_flag(send_flags::KICK);
        }

        if !self.packets.teensy_data.has_ball() {
          has_kicked = true;
        }
      }
      CpTask::TaskChip => {
        // Chip in kick dir
        // First rotate robot
        if (self.packets.robot_self.orientation
          - self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as i32)
          .abs()
          > 5
        {
          // If we are facing the right direction (variance of five degrees)
          self.packets.robot_msg.orient =
            self.packets.cp_data.cmd.kick_orient.unwrap_or_default() as u16;
        } else if !has_kicked {
          self.packets.robot_msg.kick_pwr =
            self.packets.cp_data.cmd.kick_speed.unwrap_or_default() as u8;
          self.packets.robot_msg.set_flag(send_flags::CHIP);
        }

        // Disable kicking if we have kicked (no Ball in the ball capturing zone)
        if !self.packets.teensy_data.has_ball() {
          has_kicked = true;
        }
      }
      CpTask::TaskRecKick => {
        has_kicked = false;
        // Rec Kick
        if ball_vel.norm() >= 200f32 && !self.packets.teensy_data.has_ball() {
          self.receive_ball();

          // Keep looking at the ball while moving.
          self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();
        } else {
          self.packets.robot_msg.speed = 0;
        }

        // Always enable dribbler
        self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
        self.packets.robot_msg.dribbler_pwr = 200;
      }
      CpTask::TaskSteal => {
        has_kicked = false;
        // Steal Ball
        self.get_ball(world);
      }
      CpTask::TaskDribble => {
        has_kicked = false;
        // Dribble the Ball
        // Run the steal algorithm, until we have the ball in the ball capturing zone
        if self.packets.teensy_data.has_ball() {
          let intent = NavIntent::GoToPosition {
            target_pos_mm: Vec2i::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default()),
            max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default(),
          };
          let nav_command = self.orca.step(OrcaRequest {
            intent,
            world: world.clone(),
          });

          // Enable Dribbler
          self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
          self.packets.robot_msg.dribbler_pwr = 200;

          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        } else {
          self.get_ball(world);
        }
      }
      CpTask::TaskBlock => {
        has_kicked = false;
        // Block a robot from receiving the ball
        // If enemy_id == None, defend own penalty area, else block robot
        match self.packets.cp_data.cmd.enemy_id {
          Some(_) => {
            self.defense_robot(world);
          }
          None => {
            self.defense_goal(world);
          }
        }

        // Keep looking at the ball while moving.
        self.packets.robot_msg.orient = (ball_pos - robot_pos).angle_to_u16();
      }
      CpTask::TaskPosBall => {
        has_kicked = false;
        // Position the Ball
        // Run the steal algorithm, until we have the ball in the ball capturing zone
        // After that slowly turn the dribbler off and drive away from the ball
        if self.packets.teensy_data.has_ball() {
          let intent = NavIntent::GoToPosition {
            target_pos_mm: Vec2i::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default()),
            max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default(),
          };
          let nav_command = self.orca.step(OrcaRequest {
            intent,
            world: world.clone(),
          });

          // Enable Dribbler
          self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
          self.packets.robot_msg.dribbler_pwr = 200;

          nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
        } else if robot_pos == Vec2f::new_from_cp(self.packets.cp_data.cmd.pos.unwrap_or_default())
        {
          // Logic to drive away from the ball
        } else {
          self.get_ball(world);
        }
      }
      CpTask::StateKickoff => {
        has_kicked = false;
        // Kickoff
      }
      CpTask::StateFreekick => {
        has_kicked = false;
        // Free kick
      }
    }
  }
}
