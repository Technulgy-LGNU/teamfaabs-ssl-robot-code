use crate::Robot;
use crate::communication::send_flags;
use crate::robot_logic::RAW_MAX_SPEED_MM_S;
use crate::robot_logic::helpers::{
  clamp_to_own_penalty, inside_own_penalty_area, own_goal_side, own_goal_x,
};
use crate::robot_logic::orca::{NavIntent, OrcaRequest, WorldSnapshot, nav_command_to_teensy};
use crate::robot_logic::vec::{Vec2f, Vec2i};
use core_dump::proto::CpInfos;

// How far the goalie should stay in front of the goal line when guarding.
const GOAL_LINE_MARGIN_MM: f32 = 120f32;
// Prediction horizon used to detect a kick/shot that is likely to reach goal.
const SHOT_LOOKAHEAD_S: f32 = 4f32;
// Allowed vertical miss tolerance when deciding that a ball is heading at goal.
const SHOT_Y_MARGIN_MM: f32 = 10_000f32;
// Only run the dribbler when the ball is close enough to be collected.
const DRIBBLER_RANGE_MM: f32 = 150f32;
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
    let self_vel = self
      .packets
      .robot_self
      .vel
      .map_or(Vec2f::new(0f32, 0f32), Vec2f::new_from_cp);

    // Always face the ball globally, independent of the movement direction.
    self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
    if should_run_goalie_dribbler(self_pos, ball_pos, ball_vel) {
      self.packets.robot_msg.set_flag(send_flags::DRIBBLER);
    }

    // Choose a defensive target: either a predicted interception point or a guard point.
    let target = goalie_target(&self.packets.cp_data.infos, self_pos, ball_pos, ball_vel);

    if inside_own_penalty_area(&self.packets.cp_data.infos, self_pos) {
      // Once inside the penalty area, use raw field-global motion instead of ORCA.
      goalie_move_towards(&mut self.packets.robot_msg, self_pos, self_vel, target);

      // Keep looking at the ball while moving.
      self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
      // self.packets.robot_msg.orient = ball_pos.scale(-1f32).angle_to_u16();
    } else {
      // ORCA is only used for the approach into the penalty area.
      let intent = NavIntent::GoToPosition {
        target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
        max_speed_mm_s: RAW_MAX_SPEED_MM_S as u32,
      };

      let nav_command = self.orca.step(OrcaRequest { intent, world });
      nav_command_to_teensy(&mut self.packets.robot_msg, nav_command);
      self.packets.robot_msg.orient = (ball_pos - self_pos).angle_to_u16();
    }
  }
}

#[inline]
fn should_run_goalie_dribbler(self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f) -> bool {
  (ball_pos - self_pos).norm_squared() <= DRIBBLER_RANGE_MM * DRIBBLER_RANGE_MM
    && ball_vel.norm_squared() > 100f32 * 100f32
}

#[inline]
fn goalie_move_towards(
  msg: &mut crate::communication::TeensySendMsg, self_pos: Vec2f, self_vel: Vec2f, target: Vec2f,
) {
  let delta = target - self_pos;
  let distance = delta.norm();
  if distance <= crate::robot_logic::RAW_STOP_RADIUS_MM && self_vel.norm() < 120f32 {
    msg.speed = 0;
    return;
  }

  let desired = delta * 12f32 - self_vel * 1.25f32;
  let speed = desired.norm();
  if speed <= 1f32 {
    msg.speed = 0;
    return;
  }

  msg.dir = desired.angle_to_u16();
  msg.speed = speed.clamp(0f32, RAW_MAX_SPEED_MM_S) as u16;
}

#[inline]
fn goalie_target(infos: &CpInfos, self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f) -> Vec2f {
  // Own goal is on x- or x+ depending on the robot_goal setting.
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  // Clamp to the goal width, allowing roughly one robot radius beyond each post.
  let goal_half_width = infos.goal_width as f32 * 0.5 + 90f32;

  // If the ball is moving toward goal fast enough, try to intercept it.
  if let Some(intercept) = predict_intercept(infos, self_pos, ball_pos, ball_vel) {
    return clamp_to_own_penalty(infos, intercept);
  }

  // No imminent shot: hug the goal line and slide laterally to stay between the
  // ball and the centre of the goal.
  let goal_guard_x = goal_x - goal_side * GOAL_LINE_MARGIN_MM;

  Vec2f::new(
    goal_guard_x,
    ball_pos.y.clamp(
      -goal_half_width + GUARD_Y_MARGIN_MM,
      goal_half_width - GUARD_Y_MARGIN_MM,
    ),
  )
}

#[inline]
pub(crate) fn predict_intercept(
  infos: &CpInfos, self_pos: Vec2f, ball_pos: Vec2f, ball_vel: Vec2f,
) -> Option<Vec2f> {
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  // Positive values mean the ball is moving toward our goal line.
  let vel_toward_goal = ball_vel.x * goal_side;

  if vel_toward_goal <= 120f32 || ball_vel.x.abs() <= 1f32 || ball_vel.norm() <= 100f32 {
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

  let penalty_depth = infos.penalty_area_height as f32;
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x) + 60f32;
  let x_max = goal_x.max(penalty_outer_x) - 60f32;
  let y_half = infos.penalty_area_width as f32 * 0.5 - 60f32;

  if predicted_y.abs() > y_half && ball_vel.y.abs() > 1f32 {
    let side_y = predicted_y.signum() * y_half;
    let t_side = (side_y - ball_pos.y) / ball_vel.y;
    if (0f32..=SHOT_LOOKAHEAD_S).contains(&t_side) {
      let side_x = (ball_pos.x + ball_vel.x * t_side).clamp(x_min, x_max);
      return Some(Vec2f::new(side_x, side_y));
    }
  }

  let v2 = ball_vel.norm_squared();
  let t_closest = ((self_pos - ball_pos).dot(ball_vel) / v2).clamp(0f32, SHOT_LOOKAHEAD_S);
  let mut target = ball_pos + ball_vel * t_closest;

  if target.x < x_min || target.x > x_max {
    let target_x = target.x.clamp(x_min, x_max);
    let t_at_x = ((target_x - ball_pos.x) / ball_vel.x).clamp(0f32, SHOT_LOOKAHEAD_S);
    target = ball_pos + ball_vel * t_at_x;
  }

  target.x = target.x.clamp(x_min, x_max);
  target.y = target.y.clamp(-y_half, y_half);

  Some(target)
}
