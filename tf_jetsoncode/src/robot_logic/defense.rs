use crate::Robot;
use crate::robot_logic::helpers::{own_goal_side, point_at_distance_from_a};
use crate::robot_logic::orca::{NavIntent, OrcaRequest, WorldSnapshot, nav_command_to_teensy};
use core_dump::proto::CpTrackedRobot;
use core_dump::vec::types::Vec2;

impl<C> Robot<C> {
  #[inline]
  pub fn defense_robot(&mut self, world: &WorldSnapshot) {
    let ball_pos = Vec2::new_from_cp_vec2(self.packets.cp_data.ball.pos);

    // Get the robot based on its id and cannot
    let to_block_robot = match self.packets.cp_data.infos.team_color {
      // False stands for being in the yellow team
      false => Vec2::new_from_cp_vec2(
        self
          .packets
          .cp_data
          .robots_blue
          .iter()
          .find(|r| r.robot_id == self.packets.cp_data.cmd.enemy_id.unwrap_or_default())
          .unwrap_or(&CpTrackedRobot::default())
          .pos,
      ),
      // True stands for being in the blue team
      true => Vec2::new_from_cp_vec2(
        self
          .packets
          .cp_data
          .robots_yellow
          .iter()
          .find(|r| r.robot_id == self.packets.cp_data.cmd.enemy_id.unwrap_or_default())
          .unwrap_or(&CpTrackedRobot::default())
          .pos,
      ),
    };

    let target =
      point_at_distance_from_a(to_block_robot, ball_pos, 500f32).unwrap_or(Vec2::new(0f32, 0f32));

    // If target is to far away, use orca
    let intent = NavIntent::GoToPosition {
      target_pos_mm: Vec2::new(target.x as i32, target.y as i32),
      max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default(),
    };
    let cmd = self.orca.step(OrcaRequest {
      intent,
      world: world.clone(),
    });

    nav_command_to_teensy(&mut self.packets.robot_msg, cmd);
  }

  #[inline]
  pub fn defense_goal(&mut self, world: &WorldSnapshot) {
    let goal_pos = Vec2::new(
      own_goal_side(&self.packets.cp_data.infos) * self.packets.cp_data.infos.width as f32 * 0.5,
      0f32,
    );
    let ball_pos = Vec2::new_from_cp_vec2(self.packets.cp_data.ball.pos);

    let target =
      point_at_distance_from_a(goal_pos, ball_pos, 1600f32).unwrap_or(Vec2::new(0f32, 0f32));

    // If target is to far away, use orca
    let intent = NavIntent::GoToPosition {
      target_pos_mm: Vec2::new(target.x as i32, target.y as i32),
      max_speed_mm_s: self.packets.cp_data.cmd.speed.unwrap_or_default(),
    };
    let cmd = self.orca.step(OrcaRequest {
      intent,
      world: world.clone(),
    });

    nav_command_to_teensy(&mut self.packets.robot_msg, cmd);
  }
}
