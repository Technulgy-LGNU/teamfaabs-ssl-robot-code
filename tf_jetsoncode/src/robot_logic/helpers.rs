use crate::communication::TeensySendMsg;
use crate::proto::{CpState, CpTask, CpTrackedRobot};
pub(crate) use crate::robot_logic::vec::Vec2f;
pub(crate) use crate::robot_logic::{RAW_MAX_SPEED_MM_S, RAW_STOP_RADIUS_MM};
use crate::{config, proto};

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
  if cfg.robot_goal { -1f32 } else { 1f32 }
}

#[inline]
pub(crate) fn inside_own_penalty_area(cfg: &config::Config, pos: Vec2f) -> bool {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1f32);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = cfg.field.penalty_area_width_mm().max(1f32) * 0.5;

  pos.x >= x_min && pos.x <= x_max && pos.y >= -y_half && pos.y <= y_half
}

pub(crate) fn inside_field(cfg: &config::Config, pos: Vec2f) -> bool {
  let x_half = cfg.field.width_mm() * 0.5 + cfg.field.runoff_width_mm();
  let y_half = cfg.field.height_mm() * 0.5 + cfg.field.runoff_width_mm();

  -pos.x >= x_half && pos.x <= x_half && pos.y >= -y_half && pos.y <= y_half
}

#[inline]
pub(crate) fn clamp_to_own_penalty(cfg: &config::Config, point: Vec2f) -> Vec2f {
  let goal_x = own_goal_x(cfg);
  let goal_side = own_goal_side(cfg);
  // Clamp the target to the part of the penalty area we want the goalie to use.
  let penalty_depth = cfg.field.penalty_area_height_mm().max(1f32);
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = cfg.field.penalty_area_width_mm().max(1f32) * 0.5;

  Vec2f::new(
    point.x.clamp(x_min + 40f32, x_max - 40f32),
    point.y.clamp(-y_half + 40f32, y_half - 40f32),
  )
}

#[inline]
pub(crate) fn raw_move_towards(
  msg: TeensySendMsg, self_pos: Vec2f, target: Vec2f,
) -> TeensySendMsg {
  let mut msg = msg;
  // Drive toward the chosen defensive target using raw field-global direction.
  let delta = target - self_pos;
  let distance = delta.norm();

  // Movement direction is global, not relative to robot heading.
  msg.dir = delta.angle_to_u16();
  msg.speed = if distance <= RAW_STOP_RADIUS_MM {
    0
  } else {
    // Simple proportional speed scaling, capped for safe goalie motion.
    raw_movement_accel(distance) as u16
  };

  msg
}

#[inline]
pub(crate) fn raw_movement_accel(dist: f32) -> f32 {
  (dist * 3.0).clamp(60.0, RAW_MAX_SPEED_MM_S)
}

pub(crate) fn ball_avoidance_margin_mm(
  cp_data: &proto::CpRobot, robot_self: CpTrackedRobot,
) -> u32 {
  match CpState::try_from(cp_data.cmd.state).unwrap_or(CpState::StateUnspecified) {
    CpState::StateStop => 550,
    CpState::StateFree => {
      match CpTask::try_from(cp_data.cmd.task).unwrap_or_else(|_| CpTask::TaskUnspecified) {
        CpTask::TaskSteal => {
          let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
          let robot_pos = Vec2f::new_from_cp(robot_self.pos);
          let to_ball = Vec2f::calculate_vector_2f(robot_pos, ball_pos);
          // Transformation vector with respected input angle
          let trans_vector = Vec2f {
            x: -to_ball.x * f32::sin((robot_self.orientation as f32).to_radians())
              + to_ball.y * f32::cos((robot_self.orientation as f32).to_radians()),
            y: -to_ball.x * f32::cos((robot_self.orientation as f32).to_radians())
              - to_ball.y * f32::sin((robot_self.orientation as f32).to_radians()),
          };

          if trans_vector.angle_from_y_axis().abs() > 30f32 {
            10
          } else {
            0
          }
        }
        _ => 0,
      }
    }
    _ => 0,
  }
}

pub(crate) fn allow_own_penalty_area(cp_data: &proto::CpRobot) -> bool {
  matches!(
    CpState::try_from(cp_data.cmd.state),
    Ok(CpState::StateGoalie)
  )
}

pub fn point_at_distance_from_a(a: Vec2f, b: Vec2f, distance: f32) -> Option<Vec2f> {
  let dx = b.x - a.x;
  let dy = b.y - a.y;

  let length = (dx * dx + dy * dy).sqrt();

  if length == 0.0 {
    return None; // A and B are the same point
  }

  Some(Vec2f {
    x: a.x + dx / length * distance,
    y: a.y + dy / length * distance,
  })
}
