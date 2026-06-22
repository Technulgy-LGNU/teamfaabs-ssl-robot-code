use crate::Robot;
use crate::proto::CpInfos;
use crate::robot_logic::RAW_MAX_SPEED_MM_S;
use crate::robot_logic::helpers::{
  clamp_to_own_penalty, inside_own_penalty_area, own_goal_side, own_goal_x, raw_move_towards,
};
use crate::robot_logic::orca::{NavIntent, OrcaRequest, WorldSnapshot, nav_command_to_teensy};
use crate::robot_logic::vec::{Vec2f, Vec2i, lerp};

// How far the goalie should stay in front of the goal line when guarding.
const GOAL_LINE_MARGIN_MM: f32 = 120f32;
// Extra distance from the outer penalty-area edge when the ball is far away.
const PENALTY_EDGE_MARGIN_MM: f32 = 0f32;
// Distance in front of the goal line used as the interception lane.
const INTERCEPT_LINE_MM: f32 = 220f32;
// Prediction horizon used to detect a kick/shot that is likely to reach goal.
const SHOT_LOOKAHEAD_S: f32 = 4f32;
// Allowed vertical miss tolerance when deciding that a ball is heading at goal.
const SHOT_Y_MARGIN_MM: f32 = 220f32;
// Keeps the goalie inside the goal opening instead of hugging the exact edge.
const GUARD_Y_MARGIN_MM: f32 = 20f32;

impl<C> Robot<C> {
  #[inline]
  pub fn goalie(&mut self, world: &WorldSnapshot) {
    let self_pos = Vec2f::new_from_cp(self.packets.robot_self.pos);
    let ball_pos = Vec2f::new_from_cp(self.packets.cp_data.ball.pos);
    let ball_vel = self
      .packets
      .cp_data
      .ball
      .vel
      .map_or(Vec2f::new(0f32, 0f32), Vec2f::new_from_cp);

    // Always face the ball globally, independent of the movement direction.
    self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();

    // Choose a defensive target: either a predicted interception point or a guard point.
    let target = goalie_target(&self.packets.cp_data.infos, ball_pos, ball_vel);

    if inside_own_penalty_area(&self.packets.cp_data.infos, self_pos) {
      // Once inside the penalty area, use raw field-global motion instead of ORCA.
      raw_move_towards(&mut self.packets.robot_msg, self_pos, target);

      // Keep looking at the ball while moving.
      self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
      // self.packets.robot_msg.orient = ball_pos.scale(-1f32).angle_to_u16();
    } else {
      // ORCA is only used for the approach into the penalty area.
      let intent = NavIntent::GoToPosition {
        target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
        max_speed_mm_s: RAW_MAX_SPEED_MM_S as u32,
      };

      let cmd = self.orca.step(OrcaRequest {
        intent,
        world: world.clone(),
      });

      nav_command_to_teensy(&mut self.packets.robot_msg, cmd);
      self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
    }
  }
}

#[inline]
fn goalie_target(infos: &CpInfos, ball_pos: Vec2f, ball_vel: Vec2f) -> Vec2f {
  // Own goal is on x- or x+ depending on the robot_goal setting.
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  // Half the goal opening, used to keep the goalie aligned with the ball.
  let goal_half_width = infos.width as f32 * 0.5;
  // The inner edge of the penalty area on our side.
  let penalty_depth = infos.penalty_area_height as f32;
  let penalty_outer_x = goal_x - goal_side * penalty_depth;

  // If the ball is moving toward goal fast enough, try to intercept it.
  if let Some(intercept) = predict_intercept(infos, ball_pos, ball_vel) {
    return clamp_to_own_penalty(infos, intercept);
  }

  // Otherwise, guard the goal line when the ball is close, and move further out
  // as the ball gets farther away so the robot protects more of the goal area.
  let goal_guard_x = goal_x - goal_side * GOAL_LINE_MARGIN_MM;
  let outer_guard_x = penalty_outer_x - goal_side * PENALTY_EDGE_MARGIN_MM;
  let field_scale = (infos.width * 0.5).max(1f32);
  // 0f32 near our goal, 1f32 near the far side of the field.
  let outward = ((ball_pos.x - goal_x).abs() / field_scale).clamp(0f32, 1f32);

  Vec2f::new(
    lerp(goal_guard_x, outer_guard_x, outward),
    ball_pos.y.clamp(
      -goal_half_width + GUARD_Y_MARGIN_MM,
      goal_half_width - GUARD_Y_MARGIN_MM,
    ),
  )
}

#[inline]
pub(crate) fn predict_intercept(
  infos: &CpInfos, ball_pos: Vec2f, ball_vel: Vec2f,
) -> Option<Vec2f> {
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  // Positive values mean the ball is moving toward our goal line.
  let vel_toward_goal = ball_vel.x * goal_side;

  if vel_toward_goal <= 120f32 || ball_vel.x.abs() <= 1f32 {
    return None;
  }

  // Estimate when the ball reaches the goal line in the current trajectory.
  let t_goal = (goal_x - ball_pos.x) / ball_vel.x;
  if !(0f32..=SHOT_LOOKAHEAD_S).contains(&t_goal) {
    return None;
  }

  // Estimate the y-position at impact to see whether this is actually a shot.
  let predicted_y = ball_pos.y + ball_vel.y * t_goal;
  let goal_half_width = infos.goal_width as f32 * 0.5;
  if predicted_y.abs() > goal_half_width + SHOT_Y_MARGIN_MM {
    return None;
  }

  // Place the goalie slightly in front of the expected impact point.
  Some(Vec2f::new(
    goal_x - goal_side * INTERCEPT_LINE_MM,
    predicted_y,
  ))
}
