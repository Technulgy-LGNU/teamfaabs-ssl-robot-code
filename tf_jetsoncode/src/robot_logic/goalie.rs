use crate::communication::{TeensySendMsg, VisionMsg};
use crate::config;
use crate::proto::{CpRobot, CpTrackedRobot, CpVector2, Vector2};
use crate::robot_logic::helpers::distance_cpv;

const GOAL_LINE_MARGIN_MM: f32 = 120.0;
const GOALIE_POST_MARGIN_MM: f32 = 70.0;
const GOALIE_MIN_SPEED_MM_S: f32 = 300.0;
const GOALIE_MAX_SPEED_MM_S: f32 = 1_200.0;

#[inline]
pub fn goalie(
  cfg: &config::Config, cp_data: &CpRobot, robot_self: &CpTrackedRobot, _vision_data: &VisionMsg, msg: TeensySendMsg,
) -> TeensySendMsg {
  let robot_self = *robot_self;
  let ball_pos = cp_to_vec2(cp_data.ball.pos);
  let ball_vel = cp_data.ball.vel.map_or(vec2(0.0, 0.0), cp_to_vec2);
  let predicted_ball = predict_ball_position(cfg, ball_pos, ball_vel);

  let target = goalie_target(cfg, robot_self.pos, ball_pos, predicted_ball, ball_vel);
  let target_distance = distance_cpv(robot_self.pos, vec2_to_cpv(target));
  let ball_speed = magnitude(ball_vel);
  let toward_goal = is_ball_moving_toward_our_goal(cfg, ball_vel);
  let direction_to_target = relative_direction_deg(robot_self, target);

  let mut max_speed_mm_s = if toward_goal { 3_000.0 } else { 2_500.0 };
  max_speed_mm_s += (target_distance * 0.35).clamp(0.0, 1_500.0);
  max_speed_mm_s += (ball_speed * 0.35).clamp(0.0, 900.0);
  max_speed_mm_s = max_speed_mm_s.clamp(GOALIE_MIN_SPEED_MM_S, GOALIE_MAX_SPEED_MM_S);

  let mut msg = msg;
  if target_distance <= 95.0 {
    msg.speed = 0;
    msg.dir = direction_to_target;
  } else {
    let speed = (target_distance * 1.05 + ball_speed * if toward_goal { 0.55 } else { 0.35 })
      .clamp(0.0, max_speed_mm_s)
      .round() as u16;
    msg.speed = speed;
    msg.dir = direction_to_target;
  }

  msg.orient = facing_direction_deg(robot_self.pos, predicted_ball);
  msg
}

#[inline]
fn goalie_target(
  cfg: &config::Config, robot_pos: CpVector2, ball_pos: Vector2, predicted_ball: Vector2, ball_vel: Vector2,
) -> Vector2 {
  let defend_side = goal_side(cfg);
  let half_field_length = cfg.field.width_mm() * 0.5;
  let goal_line_x = goal_line_x(cfg);

  let ball_distance_to_goal = ((goal_line_x - ball_pos.x) * defend_side).max(0.0);
  let predicted_distance_to_goal = ((goal_line_x - predicted_ball.x) * defend_side).max(0.0);
  let ball_speed = magnitude(ball_vel);

  // Far away from our goal: move more forward to reduce the available shooting angle.
  // Close to our goal: stay deeper in the penalty area to protect the line.
  let mut forward_ratio = (ball_distance_to_goal / half_field_length).clamp(0.0, 1.0);
  forward_ratio = ((forward_ratio * 0.65) + ((predicted_distance_to_goal / half_field_length).clamp(0.0, 1.0) * 0.25))
    .clamp(0.0, 1.0);
  forward_ratio += (ball_speed / 2_500.0).clamp(0.0, 0.18);
  if is_ball_moving_toward_our_goal(cfg, ball_vel) {
    forward_ratio += 0.12;
  }
  forward_ratio = forward_ratio.clamp(0.0, 1.0);

  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let min_offset = GOAL_LINE_MARGIN_MM;
  let max_offset = (penalty_depth - GOAL_LINE_MARGIN_MM).max(min_offset + 1.0);
  let x_offset = min_offset + forward_ratio * (max_offset - min_offset);
  let x = goal_line_x - defend_side * x_offset;

  let goal_half_width = (cfg.field.goal_width_mm() * 0.5).max(1.0);
  let penalty_half_width = (cfg.field.penalty_area_width_mm() * 0.5).max(goal_half_width);
  let y_limit = goal_half_width.min(penalty_half_width) - GOALIE_POST_MARGIN_MM;
  let y_limit = y_limit.max(150.0);

  let current_y = robot_pos.y as f32;
  let predicted_y = predicted_ball.y.clamp(-y_limit, y_limit);
  let y = (predicted_y * 0.75 + current_y * 0.25).clamp(-y_limit, y_limit);

  vec2(x, y)
}

#[inline]
fn predict_ball_position(cfg: &config::Config, ball_pos: Vector2, ball_vel: Vector2) -> Vector2 {
  let speed = magnitude(ball_vel);
  if speed <= 1.0 {
    return clamp_to_field(cfg, ball_pos);
  }

  let defend_side = goal_side(cfg);
  let goal_line_x = goal_line_x(cfg);
  let distance_to_goal = ((goal_line_x - ball_pos.x) * defend_side).abs();
  let toward_goal = is_ball_moving_toward_our_goal(cfg, ball_vel);

  let prediction_time_s = if toward_goal {
    (distance_to_goal / speed).clamp(0.08, 0.65)
  } else {
    (0.18 + (distance_to_goal / cfg.field.width_mm().max(1.0)) * 0.45).clamp(0.12, 1.15)
  };

  let predicted = vec2(ball_pos.x + ball_vel.x * prediction_time_s, ball_pos.y + ball_vel.y * prediction_time_s);
  clamp_to_field(cfg, predicted)
}

#[inline]
fn facing_direction_deg(from: CpVector2, to: Vector2) -> u16 {
  let dx = to.x - from.x as f32;
  let dy = to.y - from.y as f32;
  normalize_deg(dy.atan2(dx).to_degrees()).round().clamp(0.0, 359.0) as u16
}

#[inline]
fn relative_direction_deg(robot_self: CpTrackedRobot, target: Vector2) -> u16 {
  let dx = target.x - robot_self.pos.x as f32;
  let dy = target.y - robot_self.pos.y as f32;
  normalize_deg(dy.atan2(dx).to_degrees() - robot_self.orientation as f32)
    .round()
    .clamp(0.0, 359.0) as u16
}

#[inline]
fn is_ball_moving_toward_our_goal(cfg: &config::Config, ball_vel: Vector2) -> bool {
  ball_vel.x * goal_side(cfg) > 0.0
}

#[inline]
fn goal_side(cfg: &config::Config) -> f32 {
  if cfg.robot_goal { 1.0 } else { -1.0 }
}

#[inline]
fn goal_line_x(cfg: &config::Config) -> f32 {
  goal_side(cfg) * (cfg.field.width_mm() * 0.5)
}

#[inline]
fn clamp_to_field(cfg: &config::Config, point: Vector2) -> Vector2 {
  let half_width = cfg.field.width_mm() * 0.5;
  let half_height = cfg.field.height_mm() * 0.5;
  vec2(point.x.clamp(-half_width, half_width), point.y.clamp(-half_height, half_height))
}

#[inline]
fn magnitude(v: Vector2) -> f32 {
  v.x.hypot(v.y)
}

#[inline]
fn cp_to_vec2(v: CpVector2) -> Vector2 {
  vec2(v.x as f32, v.y as f32)
}

#[inline]
fn vec2_to_cpv(v: Vector2) -> CpVector2 {
  CpVector2 { x: v.x.round() as i32, y: v.y.round() as i32 }
}

#[inline]
fn vec2(x: f32, y: f32) -> Vector2 {
  Vector2 { x, y }
}

#[inline]
fn normalize_deg(mut deg: f32) -> f32 {
  while deg < 0.0 {
    deg += 360.0;
  }
  while deg >= 360.0 {
    deg -= 360.0;
  }
  deg
}

#[cfg(test)]
mod tests {
  use super::*;

  fn sample_robot(x: i32, y: i32) -> CpTrackedRobot {
    CpTrackedRobot {
      robot_id: 7,
      pos: CpVector2 { x, y },
      orientation: 0,
      vel: Some(CpVector2 { x: 0, y: 0 }),
    }
  }

  fn sample_cp(robot_self: CpTrackedRobot, ball_pos: CpVector2, ball_vel: Option<CpVector2>) -> CpRobot {
    CpRobot {
      robot_id: 7,
      timestamp: Default::default(),
      packet_id: 1,
      ball: crate::proto::CpBall { pos: ball_pos, vel: ball_vel },
      robots_yellow: vec![robot_self],
      robots_blue: vec![],
      cmd: Default::default(),
    }
  }

  #[test]
  fn moves_forward_when_ball_is_farther_away() {
    let cfg = config::Config::default();
    let robot_self = sample_robot(-4_300, 0);

    let far_ball = goalie_target(
      &cfg,
      robot_self.pos,
      vec2(-1_000.0, 0.0),
      vec2(-1_000.0, 0.0),
      vec2(0.0, 0.0),
    );
    let close_ball = goalie_target(
      &cfg,
      robot_self.pos,
      vec2(-4_350.0, 0.0),
      vec2(-4_350.0, 0.0),
      vec2(0.0, 0.0),
    );

    assert!(far_ball.x > close_ball.x);
  }

  #[test]
  fn ball_heading_toward_goal_pushes_goalie_forward_faster() {
    let cfg = config::Config::default();
    let robot_self = sample_robot(-4_300, 0);
    let ball_pos = vec2(-2_000.0, 100.0);

    let stationary = goalie_target(&cfg, robot_self.pos, ball_pos, ball_pos, vec2(0.0, 0.0));
    let moving_toward_goal = goalie_target(
      &cfg,
      robot_self.pos,
      ball_pos,
      predict_ball_position(&cfg, ball_pos, vec2(-1_500.0, 0.0)),
      vec2(-1_500.0, 0.0),
    );

    assert!(moving_toward_goal.x > stationary.x);
  }

  #[test]
  fn goalie_faces_the_ball() {
    let heading = facing_direction_deg(CpVector2 { x: 0, y: 0 }, vec2(0.0, 1_000.0));
    assert_eq!(heading, 90);
  }

  #[test]
  fn integrates_into_teensy_message() {
    let cfg = config::Config::default();
    let robot_self = sample_robot(-4_300, 0);
    let cp = sample_cp(robot_self, CpVector2 { x: -1_500, y: 0 }, Some(CpVector2 { x: -1_000, y: 0 }));

    let msg = goalie(&cfg, &cp, &robot_self, &VisionMsg::default(), TeensySendMsg::default());
    assert!(msg.speed > 0);
    assert!(msg.orient <= 359);
  }
}
