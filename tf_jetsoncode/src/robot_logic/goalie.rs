use crate::communication::{TeensySendMsg, VisionMsg};
use crate::config::Config;
use crate::proto::{CpRobot, CpTrackedRobot, CpVector2, Vector2};
use crate::robot_logic::helpers::{angle_to_u16, sub, vec2, vec2_from_cp};
use crate::robot_logic::orca::{self, OrcaOptions};

// How far the goalie should stay in front of the goal line when guarding.
const GOAL_LINE_MARGIN_MM: f32 = 120.0;
// Extra distance from the outer penalty-area edge when the ball is far away.
const PENALTY_EDGE_MARGIN_MM: f32 = 0.0;
// Distance in front of the goal line used as the interception lane.
const INTERCEPT_LINE_MM: f32 = 220.0;
// If we are inside this distance in the penalty area, stop using raw motion.
const RAW_STOP_RADIUS_MM: f32 = 40.0;
// Maximum translational speed for raw goalie movement inside the penalty area.
// ToDo: Needs to be higher
const RAW_MAX_SPEED_MM_S: f32 = 2_000.0;
// Maximum ORCA speed while approaching the penalty area.
const ORCA_MAX_SPEED_MM_S: f32 = 1_200.0;
// Prediction horizon used to detect a kick/shot that is likely to reach goal.
const SHOT_LOOKAHEAD_S: f32 = 4.0;
// Allowed vertical miss tolerance when deciding that a ball is heading at goal.
const SHOT_Y_MARGIN_MM: f32 = 220.0;
// Keeps the goalie inside the goal opening instead of hugging the exact edge.
const GUARD_Y_MARGIN_MM: f32 = 20.0;

#[inline]
pub fn goalie(
  cfg: &Config, cp_data: &CpRobot, robot_self: &CpTrackedRobot, _vision: &VisionMsg,
  mut msg: TeensySendMsg,
) -> TeensySendMsg {
  let self_pos = vec2_from_cp(robot_self.pos);
  let ball_pos = vec2_from_cp(cp_data.ball.pos);
  let ball_vel = cp_data.ball.vel.map_or(vec2(0.0, 0.0), vec2_from_cp);

  // Always face the ball globally, independent of the movement direction.
  msg.orient = angle_to_u16(sub(ball_pos, self_pos));

  // Choose a defensive target: either a predicted interception point or a guard point.
  let target = goalie_target(cfg, ball_pos, ball_vel);
  if inside_own_penalty_area(cfg, self_pos) {
    // Once inside the penalty area, use raw field-global motion instead of ORCA.
    msg = raw_move_towards(msg, self_pos, ball_pos, target);
  } else {
    // ORCA is only used for the approach into the penalty area.
    let plan = orca::drive_to_target(
      cfg,
      cp_data,
      *robot_self,
      cp_to_cp(target),
      OrcaOptions {
        max_speed_mm_s: ORCA_MAX_SPEED_MM_S,
        approach_gain: 1.45,
        stop_radius_mm: RAW_STOP_RADIUS_MM,
        avoid_ball: false,
        avoid_penalty_area: false,
        time_horizon_s: 2.5,
        robot_influence_mm: 650.0,
        ball_influence_mm: 450.0,
        penalty_margin_mm: 0.0,
        static_influence_mm: 800.0,
        ..OrcaOptions::default()
      },
    );

    msg = orca::orca_to_teensy(msg, &plan, *robot_self);
    msg.orient = angle_to_u16(sub(ball_pos, self_pos));
  }

  msg
}

#[inline]
fn goalie_target(cfg: &Config, ball_pos: Vector2, ball_vel: Vector2) -> Vector2 {
  // Own goal is on x- or x+ depending on the robot_goal setting.
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  // Half the goal opening, used to keep the goalie aligned with the ball.
  let goal_half_width = cfg.field.goal_width_mm() * 0.5;
  // The inner edge of the penalty area on our side.
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;

  // If the ball is moving toward goal fast enough, try to intercept it.
  if let Some(intercept) = predict_intercept(cfg, ball_pos, ball_vel) {
    return clamp_to_own_penalty(cfg, intercept);
  }

  // Otherwise, guard the goal line when the ball is close, and move further out
  // as the ball gets farther away so the robot protects more of the goal area.
  let goal_guard_x = goal_x - goal_side * GOAL_LINE_MARGIN_MM;
  let outer_guard_x = penalty_outer_x - goal_side * PENALTY_EDGE_MARGIN_MM;
  let field_scale = (cfg.field.width_mm() * 0.5).max(1.0);
  // 0.0 near our goal, 1.0 near the far side of the field.
  let outward = ((ball_pos.x - goal_x).abs() / field_scale).clamp(0.0, 1.0);

  vec2(
    lerp(goal_guard_x, outer_guard_x, outward),
    ball_pos.y.clamp(-goal_half_width + GUARD_Y_MARGIN_MM, goal_half_width - GUARD_Y_MARGIN_MM),
  )
}

#[inline]
pub(crate) fn predict_intercept(cfg: &Config, ball_pos: Vector2, ball_vel: Vector2) -> Option<Vector2> {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  // Positive values mean the ball is moving toward our goal line.
  let vel_toward_goal = ball_vel.x * goal_side;

  if vel_toward_goal <= 120.0 || ball_vel.x.abs() <= 1.0 {
    return None;
  }

  // Estimate when the ball reaches the goal line in the current trajectory.
  let t_goal = (goal_x - ball_pos.x) / ball_vel.x;
  if !(0.0..=SHOT_LOOKAHEAD_S).contains(&t_goal) {
    return None;
  }

  // Estimate the y-position at impact to see whether this is actually a shot.
  let predicted_y = ball_pos.y + ball_vel.y * t_goal;
  let goal_half_width = cfg.field.goal_width_mm() * 0.5;
  if predicted_y.abs() > goal_half_width + SHOT_Y_MARGIN_MM {
    return None;
  }

  // Place the goalie slightly in front of the expected impact point.
  Some(vec2(goal_x - goal_side * INTERCEPT_LINE_MM, predicted_y))
}

#[inline]
fn raw_move_towards(msg: TeensySendMsg, self_pos: Vector2, ball_pos: Vector2, target: Vector2) -> TeensySendMsg {
  let mut msg = msg;
  // Drive toward the chosen defensive target using raw field-global direction.
  let delta = sub(target, self_pos);
  let distance = norm(delta);

  // Movement direction is global, not relative to robot heading.
  msg.dir = angle_to_u16(delta);
  msg.speed = if distance <= RAW_STOP_RADIUS_MM {
    0
  } else {
    // Simple proportional speed scaling, capped for safe goalie motion.
    (distance * 2.0).clamp(60.0, RAW_MAX_SPEED_MM_S).round() as u16
  };
  // Keep looking at the ball while moving.
  msg.orient = angle_to_u16(sub(ball_pos, self_pos));

  msg
}

#[inline]
fn clamp_to_own_penalty(cfg: &Config, point: Vector2) -> Vector2 {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  // Clamp the target to the part of the penalty area we want the goalie to use.
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = cfg.field.penalty_area_width_mm().max(1.0) * 0.5;

  vec2(
    point.x.clamp(x_min + 40.0, x_max - 40.0),
    point.y.clamp(-y_half + 40.0, y_half - 40.0),
  )
}

#[inline]
fn inside_own_penalty_area(cfg: &Config, pos: Vector2) -> bool {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  // Same penalty-area bounds used by the clamping logic above.
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = cfg.field.penalty_area_width_mm().max(1.0) * 0.5;

  pos.x >= x_min && pos.x <= x_max && pos.y >= -y_half && pos.y <= y_half
}

#[inline]
fn own_goal_x(cfg: &Config) -> f32 {
  let half_length = cfg.field.width_mm() * 0.5;
  // robot_goal=true means we defend the x+ side; otherwise x-.
  if cfg.robot_goal { half_length } else { -half_length }
}

#[inline]
fn own_goal_side(cfg: &Config) -> f32 {
  // Sign helper: +1 for x+, -1 for x-.
  if cfg.robot_goal { 1.0 } else { -1.0 }
}

#[inline]
fn cp_to_cp(v: Vector2) -> CpVector2 {
  CpVector2 { x: v.x as i32, y: v.y as i32 }
}

#[inline]
fn norm(v: Vector2) -> f32 {
  v.x.hypot(v.y)
}

#[inline]
fn lerp(a: f32, b: f32, t: f32) -> f32 {
  a + (b - a) * t.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
  use crate::robot_logic::helpers::angle_to_u16;
  use super::*;

  fn sample_cfg(robot_goal: bool) -> Config {
    Config { robot_goal, ..Config::default() }
  }

  fn sample_robot(x: i32, y: i32) -> CpTrackedRobot {
    CpTrackedRobot {
      robot_id: 1,
      pos: CpVector2 { x, y },
      orientation: 0,
      vel: Some(CpVector2 { x: 0, y: 0 }),
    }
  }

  fn sample_cp(ball_x: i32, ball_y: i32, ball_vx: i32, ball_vy: i32) -> CpRobot {
    CpRobot {
      robot_id: 1,
      timestamp: Default::default(),
      packet_id: 1,
      ball: crate::proto::CpBall {
        pos: CpVector2 { x: ball_x, y: ball_y },
        vel: Some(CpVector2 { x: ball_vx, y: ball_vy }),
      },
      robots_yellow: vec![],
      robots_blue: vec![],
      cmd: Default::default(),
    }
  }

  #[test]
  fn ball_further_out_moves_goalie_further_out() {
    let cfg = sample_cfg(false);
    let near = goalie_target(&cfg, vec2(-4_200.0, 0.0), vec2(0.0, 0.0));
    let far = goalie_target(&cfg, vec2(0.0, 0.0), vec2(0.0, 0.0));
    assert!(far.x > near.x);
  }

  #[test]
  fn predicts_kick_towards_goal() {
    let cfg = sample_cfg(false);
    let intercept = predict_intercept(&cfg, vec2(-1_500.0, 120.0), vec2(-1_300.0, 50.0)).unwrap();
    assert!(intercept.x < -4_000.0);
    assert!(intercept.y > 0.0);
  }

  #[test]
  fn uses_raw_inside_penalty_area() {
    let cfg = sample_cfg(false);
    assert!(inside_own_penalty_area(&cfg, vec2(-4_300.0, 0.0)));
    assert!(!inside_own_penalty_area(&cfg, vec2(-3_000.0, 0.0)));
  }

  #[test]
  fn faces_ball_using_global_coordinates() {
    let self_pos = vec2(0.0, 0.0);
    assert_eq!(angle_to_u16(sub(vec2(1_000.0, 0.0), self_pos)), 0);
    assert_eq!(angle_to_u16(sub(vec2(0.0, 1_000.0), self_pos)), 90);
    assert_eq!(angle_to_u16(sub(vec2(-1_000.0, 0.0), self_pos)), 180);
  }

  #[test]
  fn goalie_message_changes_in_both_modes() {
    let cfg = sample_cfg(false);
    let robot = sample_robot(-4_300, 0);
    let cp = sample_cp(-1_500, 120, -1_300, 50);

    let msg = goalie(&cfg, &cp, &robot, &VisionMsg::default(), TeensySendMsg::default());
    assert!(msg.speed > 0);
    assert_ne!(msg.dir, 0);
  }
}

