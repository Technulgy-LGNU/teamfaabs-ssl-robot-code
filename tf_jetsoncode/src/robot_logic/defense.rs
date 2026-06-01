use crate::communication::TeensySendMsg;
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::helpers::{Vec2f, point_at_distance_from_a, own_goal_side};
use crate::robot_logic::orca::{
  NavIntent, OrcaHandle, OrcaRequest, Vec2i, WorldSnapshot, nav_command_to_teensy,
};

pub fn defense_robot(
  cfg: &config::Config, cp_data: &CpRobot, orca: &OrcaHandle, world: &WorldSnapshot,
  mut msg: TeensySendMsg,
) -> TeensySendMsg {
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);

  // Get the robot based on its id and cannot
  let to_block_robot = match cfg.robot_team.as_str() {
    "yellow" => Vec2f::new_from_cp(
      cp_data
        .robots_blue
        .iter()
        .find(|r| r.robot_id == cp_data.cmd.enemy_id.unwrap_or_default())
        .unwrap_or(&CpTrackedRobot::default())
        .pos,
    ),
    "blue" => Vec2f::new_from_cp(
      cp_data
        .robots_yellow
        .iter()
        .find(|r| r.robot_id == cp_data.cmd.enemy_id.unwrap_or_default())
        .unwrap_or(&CpTrackedRobot::default())
        .pos,
    ),
    _ => {
      panic!("Unknown robot_team: {}", cfg.robot_team);
    }
  };

  let target =
    point_at_distance_from_a(to_block_robot, ball_pos, 500f32).unwrap_or(Vec2f::new(0f32, 0f32));

  // If target is to far away, use orca
  let intent = NavIntent::GoToPosition {
    target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
    max_speed_mm_s: 3000,
  };
  orca.publish(OrcaRequest {
    intent,
    world: world.clone(),
  });

  msg = nav_command_to_teensy(msg, orca.latest());
  msg
}

pub fn defense_goal(
  cfg: &config::Config, cp_data: &CpRobot, orca: &OrcaHandle, world: &WorldSnapshot,
  mut msg: TeensySendMsg,
) -> TeensySendMsg {
  let goal_pos = Vec2f::new(own_goal_side(cfg) * cfg.field.width_mm() * 0.5, 0f32);
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);

  let target =
    point_at_distance_from_a(goal_pos, ball_pos, 1600f32).unwrap_or(Vec2f::new(0f32, 0f32));

  // If target is to far away, use orca
  let intent = NavIntent::GoToPosition {
    target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
    max_speed_mm_s: 3000,
  };
  orca.publish(OrcaRequest {
    intent,
    world: world.clone(),
  });

  msg = nav_command_to_teensy(msg, orca.latest());

  msg
}
