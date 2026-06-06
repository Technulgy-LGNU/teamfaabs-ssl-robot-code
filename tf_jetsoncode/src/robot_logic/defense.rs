use crate::Robot;
use crate::proto::CpTrackedRobot;
use crate::robot_logic::helpers::{Vec2f, own_goal_side, point_at_distance_from_a};
use crate::robot_logic::orca::{
  NavIntent, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};

impl<C> Robot<C> {
  #[inline]
  pub fn defense_robot(&mut self, world: &WorldSnapshot) {
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);

    // Get the robot based on its id and cannot
    let to_block_robot = match self.config.robot_team.as_str() {
      "yellow" => Vec2f::new_from_cp(
        self
          .packets
          .cp_data
          .robots_blue
          .iter()
          .find(|r| r.robot_id == self.packets.cp_data.cmd.enemy_id.unwrap_or_default())
          .unwrap_or(&CpTrackedRobot::default())
          .pos,
      ),
      "blue" => Vec2f::new_from_cp(
        self
          .packets
          .cp_data
          .robots_yellow
          .iter()
          .find(|r| r.robot_id == self.packets.cp_data.cmd.enemy_id.unwrap_or_default())
          .unwrap_or(&CpTrackedRobot::default())
          .pos,
      ),
      _ => {
        panic!("Unknown robot_team: {}", self.config.robot_team);
      }
    };

    let target =
      point_at_distance_from_a(to_block_robot, ball_pos, 500f32).unwrap_or(Vec2f::new(0f32, 0f32));

    // If target is to far away, use orca
    let intent = NavIntent::GoToPosition {
      target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
      max_speed_mm_s: 3000,
    };
    let cmd = self.orca.step(OrcaRequest {
      intent,
      world: world.clone(),
    });

    nav_command_to_teensy(&mut self.packets.robot_msg, cmd);
  }

  #[inline]
  pub fn defense_goal(&mut self, world: &WorldSnapshot) {
    let goal_pos = Vec2f::new(
      own_goal_side(&self.config) * self.config.field.width_mm() * 0.5,
      0f32,
    );
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);

    let target =
      point_at_distance_from_a(goal_pos, ball_pos, 1600f32).unwrap_or(Vec2f::new(0f32, 0f32));

    // If target is to far away, use orca
    let intent = NavIntent::GoToPosition {
      target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
      max_speed_mm_s: 3000,
    };
    let cmd = self.orca.step(OrcaRequest {
      intent,
      world: world.clone(),
    });

    nav_command_to_teensy(&mut self.packets.robot_msg, cmd);
  }
}
