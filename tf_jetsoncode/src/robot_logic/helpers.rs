use crate::communication::TeensySendMsg;
use crate::robot_logic::{RAW_MAX_SPEED_MM_S, RAW_STOP_RADIUS_MM};
use core_dump::proto::{CpInfos, CpRobot, CpState, CpTask, CpTrackedRobot};
use core_dump::vec::types::Vec2;

#[inline]
pub(crate) fn own_goal_x(infos: &CpInfos) -> f32 {
  let half_length = infos.width as f32 * 0.5;
  if infos.team_site {
    -half_length
  } else {
    half_length
  }
}

#[inline]
pub(crate) fn own_goal_side(infos: &CpInfos) -> f32 {
  if infos.team_site { -1f32 } else { 1f32 }
}

#[inline]
pub(crate) fn inside_own_penalty_area(infos: &CpInfos, pos: Vec2<f32>) -> bool {
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  let penalty_depth = infos.penalty_area_height as f32;
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = infos.penalty_area_width as f32 * 0.5;

  pos.x >= x_min && pos.x <= x_max && pos.y >= -y_half && pos.y <= y_half
}

pub(crate) fn inside_field(infos: &CpInfos, pos: Vec2<f32>) -> bool {
  let x_half = infos.width as f32 * 0.5 + infos.runoff_width as f32;
  let y_half = infos.height as f32 * 0.5 + infos.runoff_width as f32;

  -pos.x >= x_half && pos.x <= x_half && pos.y >= -y_half && pos.y <= y_half
}

#[inline]
pub(crate) fn clamp_to_own_penalty(infos: &CpInfos, point: Vec2<f32>) -> Vec2<f32> {
  let goal_x = own_goal_x(infos);
  let goal_side = own_goal_side(infos);
  // Clamp the target to the part of the penalty area we want the goalie to use.
  let penalty_depth = infos.penalty_area_height as f32;
  let penalty_outer_x = goal_x - goal_side * penalty_depth;
  let x_min = goal_x.min(penalty_outer_x);
  let x_max = goal_x.max(penalty_outer_x);
  let y_half = infos.penalty_area_width as f32 * 0.5;

  Vec2::new(
    point.x.clamp(x_min + 40f32, x_max - 40f32),
    point.y.clamp(-y_half + 40f32, y_half - 40f32),
  )
}

#[inline]
pub(crate) fn raw_move_towards(msg: &mut TeensySendMsg, self_pos: Vec2<f32>, target: Vec2<f32>) {
  // Drive toward the chosen defensive target using raw field-global direction.
  let delta = target - self_pos;
  let distance = delta.norm();

  // Movement direction is global, not relative to robot heading.
  msg.dir = delta.angle_in_u16();
  msg.speed = if distance <= RAW_STOP_RADIUS_MM {
    0
  } else {
    // Simple proportional speed scaling, capped for safe goalie motion.
    raw_movement_accel(distance) as u16
  };
}

#[inline]
pub(crate) fn raw_movement_accel(dist: f32) -> f32 {
  (dist * 3.0).clamp(60.0, RAW_MAX_SPEED_MM_S)
}

pub(crate) fn ball_avoidance_margin_mm(cp_data: &CpRobot, robot_self: CpTrackedRobot) -> u32 {
  match CpState::try_from(cp_data.cmd.state).unwrap_or(CpState::StateUnspecified) {
    CpState::StateStop => 550,
    CpState::StateFree => {
      match CpTask::try_from(cp_data.cmd.task).unwrap_or_else(|_| CpTask::TaskUnspecified) {
        CpTask::TaskSteal => {
          let ball_pos = Vec2::new_from_cp_vec2(cp_data.ball.pos);
          let robot_pos = Vec2::new_from_cp_vec2(robot_self.pos);
          let to_ball = robot_pos + ball_pos;
          // Transformation vector with respected input angle
          let trans_vector = Vec2 {
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

pub(crate) fn allow_own_penalty_area(cp_data: &CpRobot) -> bool {
  matches!(
    CpState::try_from(cp_data.cmd.state),
    Ok(CpState::StateGoalie)
  )
}

pub fn point_at_distance_from_a(a: Vec2<f32>, b: Vec2<f32>, distance: f32) -> Option<Vec2<f32>> {
  let dx = b.x - a.x;
  let dy = b.y - a.y;

  let length = (dx * dx + dy * dy).sqrt();

  if length == 0.0 {
    return None; // A and B are the same point
  }

  Some(Vec2 {
    x: a.x + dx / length * distance,
    y: a.y + dy / length * distance,
  })
}

#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
  a + (b - a) * t.clamp(0f32, 1f32)
}
