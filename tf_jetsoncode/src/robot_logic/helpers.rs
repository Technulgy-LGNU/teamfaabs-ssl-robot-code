use crate::communication::TeensySendMsg;
use crate::config;
use crate::proto::CpVector2;

// If we are inside this distance in the penalty area, stop using raw motion.
pub(crate) const RAW_STOP_RADIUS_MM: f32 = 40.0;
// Maximum translational speed for raw goalie movement inside the penalty area.
// ToDo: Needs to be higher
pub(crate) const RAW_MAX_SPEED_MM_S: f32 = 2_000.0;
// Look ahead time - used for receiving the ball
pub(crate) const LOOK_AHEAD_TIME: f32 = 2.0;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Vec2i {
  pub(crate) x: i32,
  pub(crate) y: i32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vec2f {
  pub(crate) x: f32,
  pub(crate) y: f32,
}

#[inline]
pub fn distance_cpv(a: CpVector2, b: CpVector2) -> f32 {
  let dx = (a.x - b.x) as f32;
  let dy = (a.y - b.y) as f32;
  (dx * dx + dy * dy).sqrt()
}

#[inline]
pub fn calculate_vector_2i(a: CpVector2, b: CpVector2) -> Vec2i {
  Vec2i {
    x: a.x - b.x,
    y: a.y - b.y,
  }
}

#[inline]
pub fn calculate_vector_2f(a: CpVector2, b: CpVector2) -> Vec2f {
  Vec2f {
    x: (a.x - b.x) as f32,
    y: (a.y - b.y) as f32,
  }
}

#[inline]
pub fn vec2i_length(v: Vec2i) -> f32 {
  let x = v.x as f32;
  let y = v.y as f32;
  (x * x + y * y).sqrt()
}

#[inline]
pub fn cp_vec2i_length(v: CpVector2) -> f32 {
  let x = v.x as f32;
  let y = v.y as f32;
  (x * x + y * y).sqrt()
}

#[inline]
pub(crate) fn vec2f_length(v: Vec2f) -> f32 {
  (v.x * v.x + v.y * v.y).sqrt()
}

#[inline]
pub fn vec2i_to_f32(v: Vec2i) -> (f32, f32) {
  (v.x as f32, v.y as f32)
}

#[inline]
pub fn cpv_to_vec2i(v: CpVector2) -> Vec2i {
  Vec2i { x: v.x, y: v.y }
}

#[inline]
pub fn cp_to_vec2f(v: CpVector2) -> Vec2f {
  vec2f(v.x as f32, v.y as f32)
}

#[inline]
pub(crate) fn add(a: Vec2f, b: Vec2f) -> Vec2f {
  vec2f(a.x + b.x, a.y + b.y)
}

#[inline]
pub(crate) fn sub(a: Vec2f, b: Vec2f) -> Vec2f {
  vec2f(a.x - b.x, a.y - b.y)
}

#[inline]
pub(crate) fn vec2f(x: f32, y: f32) -> Vec2f {
  Vec2f { x, y }
}

#[inline]
pub(crate) fn norm(v: Vec2f) -> f32 {
  v.x.hypot(v.y)
}

#[inline]
pub(crate) fn normalize(v: Vec2f) -> Vec2f {
  let n = norm(v);
  if n <= 1e-6 {
    vec2f(0.0, 0.0)
  } else {
    scale(v, 1.0 / n)
  }
}

#[inline]
pub(crate) fn scale(v: Vec2f, s: f32) -> Vec2f {
  vec2f(v.x * s, v.y * s)
}

#[inline]
pub(crate) fn lerp(a: f32, b: f32, t: f32) -> f32 {
  a + (b - a) * t.clamp(0.0, 1.0)
}

#[inline]
pub(crate) fn cp_to_cp(v: Vec2f) -> CpVector2 {
  CpVector2 {
    x: v.x as i32,
    y: v.y as i32,
  }
}

#[inline]
pub(crate) fn vec2f_from_cp(v: CpVector2) -> Vec2f {
  vec2f(v.x as f32, v.y as f32)
}


#[derive(Debug, Clone, Copy)]
pub(crate) struct Circle {
  pub center: Vec2f,
  pub radius: f32,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Ray {
  pub origin: Vec2f,
  /// Does not need to be normalized — the function handles that.
  pub direction: Vec2f,
}

/// All intersection points a ray has with a circle.
#[derive(Debug, PartialEq)]
pub(crate) enum RayCircleIntersection {
  /// The ray misses the circle entirely.
  None,
  /// The ray is tangent to the circle (one touch point).
  Tangent(Vec2f),
  /// The ray crosses the circle (two points, ordered nearest-first).
  TwoPoints(Vec2f, Vec2f),
}

#[inline]
pub(crate) fn own_goal_x(cfg: &config::Config) -> f32 {
  let half_length = cfg.field.width_mm() * 0.5;
  if cfg.robot_goal {
    -half_length
  } else {
    half_length
  }
}

#[inline]
pub(crate) fn own_goal_side(cfg: &config::Config) -> f32 {
  if cfg.robot_goal { -1.0 } else { 1.0 }
}

#[inline]
pub(crate) fn angle_to_u16(v: Vec2f) -> u16 {
  let mut deg = v.y.atan2(v.x).to_degrees();
  while deg < 0.0 {
    deg += 360.0;
  }
  while deg >= 360.0 {
    deg -= 360.0;
  }
  deg.round().clamp(0.0, 359.0) as u16
}

#[inline]
pub(crate) fn inside_own_penalty_area(cfg: &config::Config, pos: Vec2f) -> bool {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = cfg.field.penalty_area_width_mm().max(1.0) * 0.5;

  pos.x >= x_min && pos.x <= x_max && pos.y >= -y_half && pos.y <= y_half
}

#[inline]
pub(crate) fn clamp_outside_own_penalty(cfg: &config::Config, point: Vec2f) -> Vec2f {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let y_half = cfg.field.penalty_area_width_mm().max(1.0) * 0.5;
  let safety_margin = 40.0;

  let x = if goal_side > 0.0 {
    point.x.min(penalty_outer_x - safety_margin)
  } else {
    point.x.max(penalty_outer_x + safety_margin)
  };

  vec2f(
    x,
    point
      .y
      .clamp(-y_half + safety_margin, y_half - safety_margin),
  )
}

#[inline]
pub(crate) fn clamp_to_own_penalty(cfg: &config::Config, point: Vec2f) -> Vec2f {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  // Clamp the target to the part of the penalty area we want the goalie to use.
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1.0);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = cfg.field.penalty_area_width_mm().max(1.0) * 0.5;

  vec2f(
    point.x.clamp(x_min + 40.0, x_max - 40.0),
    point.y.clamp(-y_half + 40.0, y_half - 40.0),
  )
}

#[inline]
pub(crate) fn raw_move_towards(
  msg: TeensySendMsg, self_pos: Vec2f, ball_pos: Vec2f, target: Vec2f,
) -> TeensySendMsg {
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
