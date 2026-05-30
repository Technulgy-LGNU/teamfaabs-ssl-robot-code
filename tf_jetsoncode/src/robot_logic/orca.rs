use crate::communication::TeensySendMsg;
use crate::config::Config;
use crate::proto::{CpRobot, CpTrackedRobot, CpVector2};
use crate::robot_logic::helpers::Vec2f;
use crate::{DEFAULT_ACCEL_MM_S2, DEFAULT_DECEL_MM_S2};
use std::ops::Add;

const DEFAULT_DT_S: f32 = 0.004;
const DEFAULT_MAX_SPEED_MM_S: f32 = 6000f32;
const DEFAULT_APPROACH_GAIN: f32 = 1.6;
const DEFAULT_ROBOT_RADIUS_MM: f32 = 90f32;
const DEFAULT_ROBOT_INFLUENCE_MM: f32 = 520f32;
const DEFAULT_BALL_AVOID_MM: f32 = 500f32;
const DEFAULT_PENALTY_MARGIN_MM: f32 = 200f32;
const DEFAULT_STATIC_INFLUENCE_MM: f32 = 900f32;

#[derive(Debug, Clone, Copy)]
pub struct OrcaOptions {
  pub max_speed_mm_s: f32,
  pub approach_gain: f32,
  pub stop_radius_mm: f32,
  pub avoid_ball: bool,
  pub ball_avoid_mm: f32,
  pub avoid_penalty_area: bool,
  pub time_horizon_s: f32,
  pub robot_radius_mm: f32,
  pub robot_influence_mm: f32,
  pub ball_influence_mm: f32,
  pub penalty_margin_mm: f32,
  pub static_influence_mm: f32,
  pub accel_mm_s2: f32,
  pub decel_mm_s2: f32,
}

impl Default for OrcaOptions {
  fn default() -> Self {
    Self {
      max_speed_mm_s: DEFAULT_MAX_SPEED_MM_S,
      approach_gain: DEFAULT_APPROACH_GAIN,
      stop_radius_mm: 180f32,
      avoid_ball: true,
      ball_avoid_mm: DEFAULT_BALL_AVOID_MM,
      avoid_penalty_area: true,
      time_horizon_s: 4f32,
      robot_radius_mm: DEFAULT_ROBOT_RADIUS_MM,
      robot_influence_mm: DEFAULT_ROBOT_INFLUENCE_MM,
      ball_influence_mm: DEFAULT_BALL_AVOID_MM,
      penalty_margin_mm: DEFAULT_PENALTY_MARGIN_MM,
      static_influence_mm: DEFAULT_STATIC_INFLUENCE_MM,
      accel_mm_s2: DEFAULT_ACCEL_MM_S2,
      decel_mm_s2: DEFAULT_DECEL_MM_S2,
    }
  }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Default)]
pub struct OrcaPlan {
  pub velocity: Vec2f,
  pub target: Vec2f,
  pub target_distance_mm: f32,
  pub speed_mm_s: f32,
  pub direction_deg: f32,
  pub current_speed_mm_s: f32,
  pub accel_hint_mm_s2: f32,
  pub decel_hint_mm_s2: f32,
}

#[derive(Debug, Clone, Copy)]
struct Rect {
  min_x: f32,
  max_x: f32,
  min_y: f32,
  max_y: f32,
}

#[allow(dead_code)]
#[inline]
pub fn default_options() -> OrcaOptions {
  OrcaOptions::default()
}

#[inline]
pub fn drive_to_target(
  cfg: &Config, cp_data: &CpRobot, robot_self: CpTrackedRobot, target: CpVector2,
  options: OrcaOptions,
) -> OrcaPlan {
  let self_pos = Vec2f::new_from_cp(robot_self.pos);
  let self_vel = robot_velocity(robot_self);
  let mut target = Vec2f::new_from_cp(target);

  if options.avoid_penalty_area {
    target = project_target_outside_penalty_areas(cfg, target, options.penalty_margin_mm);
  }

  let to_target = target - self_pos;
  let distance_mm = to_target.norm();
  let current_speed_mm_s = self_vel.norm();

  if distance_mm <= options.stop_radius_mm {
    return OrcaPlan {
      velocity: Vec2f::new(0f32, 0f32),
      target,
      target_distance_mm: distance_mm,
      speed_mm_s: 0f32,
      direction_deg: 0f32,
      current_speed_mm_s,
      accel_hint_mm_s2: options.accel_mm_s2,
      decel_hint_mm_s2: options.decel_mm_s2,
    };
  }

  let mut desired_speed = (distance_mm * options.approach_gain).min(options.max_speed_mm_s);
  desired_speed = decel_limit(desired_speed, distance_mm, options.decel_mm_s2);
  let _accel_preview = accel_limit(
    current_speed_mm_s,
    desired_speed,
    options.accel_mm_s2,
    DEFAULT_DT_S,
  );
  desired_speed = desired_speed.min(options.max_speed_mm_s);

  let mut velocity = to_target.normalize().scale(desired_speed);
  velocity = avoid_dynamic_robots(cfg, cp_data, robot_self, velocity, options);

  if options.avoid_ball {
    velocity = avoid_ball(cp_data, robot_self, velocity, options);
  }

  if options.avoid_penalty_area {
    velocity = avoid_penalty_areas(cfg, robot_self, velocity, options);
  }

  velocity = clamp_magnitude(velocity, options.max_speed_mm_s);

  let speed_mm_s = velocity.norm();
  OrcaPlan {
    velocity,
    target,
    target_distance_mm: distance_mm,
    speed_mm_s,
    direction_deg: velocity_to_direction_deg(velocity),
    current_speed_mm_s,
    accel_hint_mm_s2: options.accel_mm_s2,
    decel_hint_mm_s2: options.decel_mm_s2,
  }
}

#[inline]
pub fn orca_to_teensy(
  mut msg: TeensySendMsg, plan: &OrcaPlan, _robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  let speed = plan
    .velocity
    .x
    .hypot(plan.velocity.y)
    .round()
    .clamp(0f32, u16::MAX as f32) as u16;
  let dir = velocity_to_teensy_dir(plan.velocity);

  msg.speed = speed;
  msg.dir = dir;
  msg
}

#[allow(dead_code)]
#[inline]
pub fn accel_limit(
  current_speed_mm_s: f32, target_speed_mm_s: f32, accel_mm_s2: f32, dt_s: f32,
) -> f32 {
  let max_step = accel_mm_s2.max(0f32) * dt_s.max(0f32);
  if target_speed_mm_s >= current_speed_mm_s {
    (current_speed_mm_s + max_step).min(target_speed_mm_s)
  } else {
    target_speed_mm_s
  }
}

#[inline]
pub fn decel_limit(speed_mm_s: f32, distance_mm: f32, decel_mm_s2: f32) -> f32 {
  if distance_mm <= 0f32 || decel_mm_s2 <= 0f32 {
    return 0f32;
  }

  let braking_speed = (2f32 * decel_mm_s2 * distance_mm).sqrt();
  speed_mm_s.min(braking_speed)
}

#[inline]
fn avoid_dynamic_robots(
  cfg: &Config, cp_data: &CpRobot, robot_self: CpTrackedRobot, velocity: Vec2f,
  options: OrcaOptions,
) -> Vec2f {
  let self_pos = Vec2f::new_from_cp(robot_self.pos);
  let self_vel = robot_velocity(robot_self);
  let horizon = options.time_horizon_s.max(0.1);
  let mut out = velocity;

  let (friendly, opponents) = if is_yellow_team(cfg) {
    (&cp_data.robots_yellow, &cp_data.robots_blue)
  } else {
    (&cp_data.robots_blue, &cp_data.robots_yellow)
  };

  for robot in opponents {
    if robot.robot_id == robot_self.robot_id {
      continue;
    }

    let obstacle_pos = Vec2f::new_from_cp(robot.pos);
    let obstacle_vel = robot_velocity(*robot);
    let base_radius = options.robot_radius_mm + 65f32;
    let influence = options.robot_influence_mm + 120f32;
    out = steer_around_dynamic(
      out,
      self_pos,
      self_vel,
      obstacle_pos,
      obstacle_vel,
      base_radius,
      influence,
      horizon,
      1f32,
    );
  }

  for robot in friendly {
    if robot.robot_id == robot_self.robot_id {
      continue;
    }

    let obstacle_pos = Vec2f::new_from_cp(robot.pos);
    let obstacle_vel = robot_velocity(*robot);
    let base_radius = options.robot_radius_mm + 40f32;
    let influence = options.robot_influence_mm + 50f32;
    out = steer_around_dynamic(
      out,
      self_pos,
      self_vel,
      obstacle_pos,
      obstacle_vel,
      base_radius,
      influence,
      horizon,
      0.55,
    );
  }

  out
}

#[inline]
fn avoid_ball(
  cp_data: &CpRobot, robot_self: CpTrackedRobot, velocity: Vec2f, options: OrcaOptions,
) -> Vec2f {
  let self_pos = Vec2f::new_from_cp(robot_self.pos);
  let self_vel = robot_velocity(robot_self);
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
  let ball_vel = cp_data
    .ball
    .vel
    .map_or(Vec2f::new(0f32, 0f32), Vec2f::new_from_cp);

  steer_around_dynamic(
    velocity,
    self_pos,
    self_vel,
    ball_pos,
    ball_vel,
    options.ball_avoid_mm.max(150f32),
    options.ball_influence_mm.max(options.ball_avoid_mm),
    options.time_horizon_s.max(0.1),
    1.25,
  )
}

#[inline]
fn avoid_penalty_areas(
  cfg: &Config, robot_self: CpTrackedRobot, velocity: Vec2f, options: OrcaOptions,
) -> Vec2f {
  let self_pos = Vec2f::new_from_cp(robot_self.pos);
  let mut out = velocity;

  for rect in penalty_rects(cfg).into_iter() {
    let expanded = expand_rect(rect, options.penalty_margin_mm);
    out = steer_around_rect(
      out,
      self_pos,
      expanded,
      options.static_influence_mm,
      options.max_speed_mm_s,
    );
  }

  out
}

#[inline]
fn project_target_outside_penalty_areas(cfg: &Config, target: Vec2f, margin_mm: f32) -> Vec2f {
  let mut out = target;

  for rect in penalty_rects(cfg).into_iter() {
    let expanded = expand_rect(rect, margin_mm);
    if point_in_rect(out, expanded) {
      out = project_outside_rect(out, expanded, margin_mm);
    }
  }

  out
}

#[inline]
fn steer_around_dynamic(
  velocity: Vec2f, self_pos: Vec2f, self_vel: Vec2f, obstacle_pos: Vec2f, obstacle_vel: Vec2f,
  radius_mm: f32, influence_mm: f32, horizon_s: f32, weight: f32,
) -> Vec2f {
  let rel_pos = obstacle_pos - self_pos;
  let rel_vel = velocity - obstacle_vel;
  let rel_speed_sq = dot(rel_vel, rel_vel).max(1f32);
  let mut t_star = -dot(rel_pos, rel_vel) / rel_speed_sq;
  t_star = t_star.clamp(0f32, horizon_s);

  let closest: Vec2f = rel_pos + rel_vel.scale(t_star);
  let closest_dist = closest.norm();
  let current_dist = rel_pos.norm();
  let threshold = radius_mm.max(1f32);
  let mut correction = Vec2f::new(0f32, 0f32);

  if closest_dist < influence_mm || current_dist < threshold {
    let away = if closest_dist > 1f32 {
      closest.scale(-1f32 / closest_dist)
    } else if current_dist > 1f32 {
      rel_pos.scale(-1f32 / current_dist)
    } else if self_vel.norm() > 1f32 {
      self_vel.scale(-1f32 / self_vel.norm())
    } else {
      Vec2f::new(1f32, 0f32)
    };

    let lateral = Vec2f::new(-rel_pos.y, rel_pos.x).normalize();
    let dodge_dir = if rel_pos.y.abs() < radius_mm * 0.5 {
      lateral
    } else {
      away
    };

    let closeness = if closest_dist < influence_mm {
      (influence_mm - closest_dist) / influence_mm
    } else {
      0f32
    };
    let overlap = if current_dist < threshold {
      (threshold - current_dist).max(0f32) / threshold
    } else {
      0f32
    };

    let urgency = (closeness + overlap).clamp(0f32, 1.5);
    let speed_push = 0.7 * influence_mm + 1_300f32 * urgency;
    correction = dodge_dir.scale(speed_push * weight);
  }

  velocity + correction
}

#[inline]
fn steer_around_rect(
  velocity: Vec2f, self_pos: Vec2f, rect: Rect, influence_mm: f32, max_speed_mm_s: f32,
) -> Vec2f {
  let nearest = clamp_point(self_pos, rect);
  let offset = self_pos - nearest;
  let dist = offset.norm();
  let inside = point_in_rect(self_pos, rect);

  if !inside && dist >= influence_mm {
    return velocity;
  }

  let dir = if inside {
    rect_escape_direction(self_pos, rect)
  } else if dist > 1f32 {
    offset.scale(1f32 / dist)
  } else {
    rect_escape_direction(self_pos, rect)
  };

  let strength = if inside {
    1.4 + ((influence_mm - dist).max(0f32) / influence_mm)
  } else {
    ((influence_mm - dist).max(0f32) / influence_mm).clamp(0f32, 1f32)
  };

  velocity + dir.scale(max_speed_mm_s * 1.15 * strength)
}

#[inline]
fn rect_escape_direction(point: Vec2f, rect: Rect) -> Vec2f {
  let to_left = (point.x - rect.min_x).abs();
  let to_right = (rect.max_x - point.x).abs();
  let to_bottom = (point.y - rect.min_y).abs();
  let to_top = (rect.max_y - point.y).abs();

  if to_left <= to_right && to_left <= to_bottom && to_left <= to_top {
    Vec2f::new(-1f32, 0f32)
  } else if to_right <= to_bottom && to_right <= to_top {
    Vec2f::new(1f32, 0f32)
  } else if to_bottom <= to_top {
    Vec2f::new(0f32, -1f32)
  } else {
    Vec2f::new(0f32, 1f32)
  }
}

#[inline]
fn project_outside_rect(point: Vec2f, rect: Rect, margin_mm: f32) -> Vec2f {
  let mut out = point;
  let push = margin_mm.max(1f32);
  let to_left = (point.x - rect.min_x).abs();
  let to_right = (rect.max_x - point.x).abs();
  let to_bottom = (point.y - rect.min_y).abs();
  let to_top = (rect.max_y - point.y).abs();

  if to_left <= to_right && to_left <= to_bottom && to_left <= to_top {
    out.x = rect.min_x - push;
  } else if to_right <= to_bottom && to_right <= to_top {
    out.x = rect.max_x + push;
  } else if to_bottom <= to_top {
    out.y = rect.min_y - push;
  } else {
    out.y = rect.max_y + push;
  }

  out
}

#[inline]
fn penalty_rects(cfg: &Config) -> [Rect; 2] {
  let half_length = cfg.field.width_mm() * 0.5;
  let half_width = cfg.field.height_mm() * 0.5;
  let depth = cfg.field.penalty_area_height_mm().max(1f32);
  let width = cfg.field.penalty_area_width_mm().max(1f32) * 0.5;
  let y_min = (-width).max(-half_width);
  let y_max = width.min(half_width);

  [
    Rect {
      min_x: -half_length,
      max_x: -half_length + depth,
      min_y: y_min,
      max_y: y_max,
    },
    Rect {
      min_x: half_length - depth,
      max_x: half_length,
      min_y: y_min,
      max_y: y_max,
    },
  ]
}

#[inline]
fn expand_rect(rect: Rect, margin_mm: f32) -> Rect {
  Rect {
    min_x: rect.min_x - margin_mm,
    max_x: rect.max_x + margin_mm,
    min_y: rect.min_y - margin_mm,
    max_y: rect.max_y + margin_mm,
  }
}

#[inline]
fn point_in_rect(point: Vec2f, rect: Rect) -> bool {
  point.x >= rect.min_x && point.x <= rect.max_x && point.y >= rect.min_y && point.y <= rect.max_y
}

#[inline]
fn clamp_point(point: Vec2f, rect: Rect) -> Vec2f {
  Vec2f::new(
    point.x.clamp(rect.min_x, rect.max_x),
    point.y.clamp(rect.min_y, rect.max_y),
  )
}

#[inline]
fn is_yellow_team(cfg: &Config) -> bool {
  cfg.robot_team.eq_ignore_ascii_case("yellow")
}

#[inline]
fn robot_velocity(robot: CpTrackedRobot) -> Vec2f {
  robot.vel.map_or(Vec2f::new(0f32, 0f32), Vec2f::new_from_cp)
}

#[inline]
fn dot(a: Vec2f, b: Vec2f) -> f32 {
  a.x * b.x + a.y * b.y
}

#[inline]
fn clamp_magnitude(v: Vec2f, max: f32) -> Vec2f {
  let n = v.norm();
  if n <= max || n <= 1e-6 {
    v
  } else {
    v.scale(max / n)
  }
}

#[inline]
fn normalize_deg(mut deg: f32) -> f32 {
  while deg < 0f32 {
    deg += 360f32;
  }
  while deg >= 360f32 {
    deg -= 360f32;
  }
  deg
}

#[inline]
fn velocity_to_direction_deg(velocity: Vec2f) -> f32 {
  let speed = velocity.norm();
  if speed <= 1f32 {
    return 0f32;
  }

  normalize_deg(velocity.y.atan2(velocity.x).to_degrees())
}

#[inline]
fn velocity_to_teensy_dir(velocity: Vec2f) -> u16 {
  velocity_to_direction_deg(velocity)
    .round()
    .clamp(0f32, 359f32) as u16
}

#[cfg(test)]
mod tests {
  use super::*;

  fn sample_config() -> Config {
    Config::default()
  }

  fn sample_robot(x: i32, y: i32, orientation: i32, vel: Option<(i32, i32)>) -> CpTrackedRobot {
    CpTrackedRobot {
      robot_id: 7,
      pos: CpVector2 { x, y },
      orientation,
      vel: vel.map(|(x, y)| CpVector2 { x, y }),
    }
  }

  fn sample_cp(robot_self: CpTrackedRobot) -> CpRobot {
    CpRobot {
      robot_id: 7,
      timestamp: Default::default(),
      packet_id: 1,
      ball: crate::proto::CpBall {
        pos: CpVector2 { x: 0, y: 0 },
        vel: Some(CpVector2 { x: 0, y: 0 }),
      },
      robots_yellow: vec![robot_self],
      robots_blue: vec![],
      cmd: Default::default(),
    }
  }

  #[test]
  fn converts_velocity_to_teensy_message() {
    let robot_self = sample_robot(0, 0, 0, Some((0, 0)));
    let plan = OrcaPlan {
      velocity: Vec2f::new(1_000f32, 0f32),
      target: Vec2f::new(1_000f32, 0f32),
      target_distance_mm: 1_000f32,
      speed_mm_s: 1_000f32,
      direction_deg: 0f32,
      current_speed_mm_s: 0f32,
      accel_hint_mm_s2: 0f32,
      decel_hint_mm_s2: 0f32,
    };

    let msg = orca_to_teensy(TeensySendMsg::default(), &plan, robot_self);
    assert_eq!(msg.speed, 1_000);
  }

  #[test]
  fn direction_uses_x_as_zero_degrees() {
    assert_eq!(velocity_to_teensy_dir(Vec2f::new(1_000f32, 0f32)), 0);
    assert_eq!(velocity_to_teensy_dir(Vec2f::new(0f32, 1_000f32)), 90);
    assert_eq!(velocity_to_teensy_dir(Vec2f::new(-1_000f32, 0f32)), 180);
    assert_eq!(velocity_to_teensy_dir(Vec2f::new(0f32, -1_000f32)), 270);
  }

  #[test]
  fn direction_is_field_global_not_robot_relative() {
    let _robot_self = sample_robot(0, 0, 90, Some((0, 0)));
    assert_eq!(velocity_to_teensy_dir(Vec2f::new(1_000f32, 0f32)), 0);
    assert_eq!(velocity_to_teensy_dir(Vec2f::new(0f32, 1_000f32)), 90);
  }

  #[test]
  fn avoids_robots_in_front() {
    let robot_self = sample_robot(0, 0, 0, Some((0, 0)));
    let mut cp = sample_cp(robot_self);
    let mut obstacle = sample_robot(500, 0, 0, Some((0, 0)));
    obstacle.robot_id = 42;
    cp.robots_blue = vec![obstacle];

    let plan = drive_to_target(
      &sample_config(),
      &cp,
      robot_self,
      CpVector2 { x: 1_000, y: 0 },
      OrcaOptions {
        avoid_ball: false,
        avoid_penalty_area: false,
        ..OrcaOptions::default()
      },
    );
    assert!(plan.velocity.y.abs() > 1f32 || plan.velocity.x < 700f32);
  }

  #[test]
  fn pushes_out_of_penalty_area() {
    let robot_self = sample_robot(-4_100, 0, 0, Some((0, 0)));
    let cp = sample_cp(robot_self);
    let plan = drive_to_target(
      &sample_config(),
      &cp,
      robot_self,
      CpVector2 { x: -4_700, y: 0 },
      OrcaOptions {
        avoid_ball: false,
        ..OrcaOptions::default()
      },
    );
    assert!(plan.velocity.x < 0f32);
  }
}
