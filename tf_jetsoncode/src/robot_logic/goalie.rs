use crate::communication::{TeensySendMsg, VisionMsg};
use crate::config::Config;
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::helpers::{Vec2f, clamp_to_own_penalty, inside_own_penalty_area, lerp, own_goal_side, own_goal_x, raw_move_towards, RAW_STOP_RADIUS_MM};
use crate::robot_logic::orca::{self, OrcaOptions};

// How far the goalie should stay in front of the goal line when guarding.
const GOAL_LINE_MARGIN_MM: f32 = 120f32;
// Extra distance from the outer penalty-area edge when the ball is far away.
const PENALTY_EDGE_MARGIN_MM: f32 = 0f32;
// Distance in front of the goal line used as the interception lane.
const INTERCEPT_LINE_MM: f32 = 220f32;
// Maximum ORCA speed while approaching the penalty area.
const ORCA_MAX_SPEED_MM_S: f32 = 1_200f32;
// Prediction horizon used to detect a kick/shot that is likely to reach goal.
const SHOT_LOOKAHEAD_S: f32 = 4f32;
// Allowed vertical miss tolerance when deciding that a ball is heading at goal.
const SHOT_Y_MARGIN_MM: f32 = 220f32;
// Keeps the goalie inside the goal opening instead of hugging the exact edge.
const GUARD_Y_MARGIN_MM: f32 = 20f32;

#[inline]
pub fn goalie(
  cfg: &Config, cp_data: &CpRobot, robot_self: &CpTrackedRobot, _vision: &VisionMsg,
  mut msg: TeensySendMsg,
) -> TeensySendMsg {
  let self_pos = Vec2f::new_from_cp(robot_self.pos);
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
  let ball_vel = cp_data.ball.vel.map_or(Vec2f::new(0f32, 0f32), Vec2f::new_from_cp);

  // Always face the ball globally, independent of the movement direction.
  msg.orient = (ball_pos - self_pos).angle_to_u16();

  // Choose a defensive target: either a predicted interception point or a guard point.
  let target = goalie_target(cfg, ball_pos, ball_vel);
  if inside_own_penalty_area(cfg, self_pos) {
    // Once inside the penalty area, use raw field-global motion instead of ORCA.
    msg = raw_move_towards(msg, self_pos, target);
    // Keep looking at the ball while moving.
    msg.orient = (ball_pos - self_pos).angle_to_u16();
    // msg.orient = ball_pos.scale(-1f32).angle_to_u16();
  } else {
    // ORCA is only used for the approach into the penalty area.
    let plan = orca::drive_to_target(
      cfg,
      cp_data,
      *robot_self,
      target.vec2f_to_cp(),
      OrcaOptions {
        max_speed_mm_s: ORCA_MAX_SPEED_MM_S,
        approach_gain: 1.45,
        stop_radius_mm: RAW_STOP_RADIUS_MM,
        avoid_ball: false,
        avoid_penalty_area: false,
        time_horizon_s: 2.5,
        robot_influence_mm: 650f32,
        ball_influence_mm: 450f32,
        penalty_margin_mm: 0f32,
        static_influence_mm: 800f32,
        ..OrcaOptions::default()
      },
    );

    msg = orca::orca_to_teensy(msg, &plan, *robot_self);
    msg.orient = (ball_pos - self_pos).angle_to_u16();
  }

  msg
}

#[inline]
fn goalie_target(cfg: &Config, ball_pos: Vec2f, ball_vel: Vec2f) -> Vec2f {
  // Own goal is on x- or x+ depending on the robot_goal setting.
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  // Half the goal opening, used to keep the goalie aligned with the ball.
  let goal_half_width = cfg.field.goal_width_mm() * 0.5;
  // The inner edge of the penalty area on our side.
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1f32);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;

  // If the ball is moving toward goal fast enough, try to intercept it.
  if let Some(intercept) = predict_intercept(cfg, ball_pos, ball_vel) {
    return clamp_to_own_penalty(cfg, intercept);
  }

  // Otherwise, guard the goal line when the ball is close, and move further out
  // as the ball gets farther away so the robot protects more of the goal area.
  let goal_guard_x = goal_x - goal_side * GOAL_LINE_MARGIN_MM;
  let outer_guard_x = penalty_outer_x - goal_side * PENALTY_EDGE_MARGIN_MM;
  let field_scale = (cfg.field.width_mm() * 0.5).max(1f32);
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
pub(crate) fn predict_intercept(cfg: &Config, ball_pos: Vec2f, ball_vel: Vec2f) -> Option<Vec2f> {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
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
  let goal_half_width = cfg.field.goal_width_mm() * 0.5;
  if predicted_y.abs() > goal_half_width + SHOT_Y_MARGIN_MM {
    return None;
  }

  // Place the goalie slightly in front of the expected impact point.
  Some(Vec2f::new(goal_x - goal_side * INTERCEPT_LINE_MM, predicted_y))
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::proto::CpVector2;

  fn sample_cfg(robot_goal: bool) -> Config {
    Config {
      robot_goal,
      ..Config::default()
    }
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
        pos: CpVector2 {
          x: ball_x,
          y: ball_y,
        },
        vel: Some(CpVector2 {
          x: ball_vx,
          y: ball_vy,
        }),
      },
      robots_yellow: vec![],
      robots_blue: vec![],
      cmd: Default::default(),
    }
  }

  #[test]
  fn ball_further_out_moves_goalie_further_out() {
    let cfg = sample_cfg(false);
    let near = goalie_target(&cfg, Vec2f::new(-4_200f32, 0f32), Vec2f::new(0f32, 0f32));
    let far = goalie_target(&cfg, Vec2f::new(0f32, 0f32), Vec2f::new(0f32, 0f32));
    assert!(far.x > near.x);
  }

  #[test]
  fn predicts_kick_towards_goal() {
    let cfg = sample_cfg(false);
    let intercept = predict_intercept(&cfg, Vec2f::new(-1_500f32, 120f32), Vec2f::new(-1_300f32, 50f32)).unwrap();
    assert!(intercept.x < -4_000f32);
    assert!(intercept.y > 0f32);
  }

  #[test]
  fn uses_raw_inside_penalty_area() {
    let cfg = sample_cfg(false);
    assert!(inside_own_penalty_area(&cfg, Vec2f::new(-4_300f32, 0f32)));
    assert!(!inside_own_penalty_area(&cfg, Vec2f::new(-3_000f32, 0f32)));
  }

  #[test]
  fn faces_ball_using_global_coordinates() {
    let self_pos = Vec2f::new(0f32, 0f32);
    assert_eq!((Vec2f::new(1_000f32, 0f32) - self_pos).angle_to_u16(), 0);
    assert_eq!((Vec2f::new(0f32, 1_000f32) - self_pos).angle_to_u16(), 90);
    assert_eq!((Vec2f::new(-1_000f32, 0f32) - self_pos).angle_to_u16(), 180);
  }

  #[test]
  fn goalie_message_changes_in_both_modes() {
    let cfg = sample_cfg(false);
    let robot = sample_robot(-4_300, 0);
    let cp = sample_cp(-1_500, 120, -1_300, 50);

    let msg = goalie(
      &cfg,
      &cp,
      &robot,
      &VisionMsg::default(),
      TeensySendMsg::default(),
    );
    assert!(msg.speed > 0);
    assert_ne!(msg.dir, 0);
  }
}
