use crate::communication::{TeensySendMsg, VisionMsg, send_flags};
use crate::proto::{CpRobot, CpTrackedRobot};
use crate::robot_logic::helpers::point_at_distance_from_a;
use crate::robot_logic::orca::{
  NavIntent, OrcaHandle, OrcaRequest, WorldSnapshot, nav_command_to_teensy,
};
use crate::robot_logic::vec::{Vec2f, Vec2i};

/// Distance to ball where we switch to direct control
const BALL_APPROACH_DISTANCE: f32 = 150f32;
/// Functions drives behind the ball and then drives forward and stops with ball in capturing zone
#[inline]
pub fn get_ball(
  cp_data: &CpRobot, _vision_data: &VisionMsg, orca: &OrcaHandle, world: &WorldSnapshot,
  mut msg: TeensySendMsg, robot_self: CpTrackedRobot,
) -> TeensySendMsg {
  let direction_vec = Vec2f::new(
    f32::cos((cp_data.cmd.orientation.unwrap_or_default() as f32).to_radians()),
    f32::sin((cp_data.cmd.orientation.unwrap_or_default() as f32).to_radians()),
  );
  let robot_pos = Vec2f::new_from_cp(robot_self.pos);
  let ball_pos = Vec2f::new_from_cp(cp_data.ball.pos);
  let to_ball = Vec2f::calculate_vector_2f(robot_pos, ball_pos);
  let capture_zone_to_ball = Vec2f::calculate_vector_2f(ball_pos,robot_pos + direction_vec.scale(80f32)).angle_to_u16();


  // Check based on the distance, if dribbler should be enabled
  if to_ball.norm() < 200f32 {
    msg.set_flag(send_flags::DRIBBLER);
    msg.dribbler_pwr = 200;
  }

  // First check, if we already are in front of the ball, if yes, move forwards
  // Transformation vector with respected input angle
  let trans_vector = Vec2f {
    x: -to_ball.x * f32::sin((robot_self.orientation as f32).to_radians())
      + to_ball.y * f32::cos((robot_self.orientation as f32).to_radians()),
    y: -to_ball.x * f32::cos((robot_self.orientation as f32).to_radians())
      - to_ball.y * f32::sin((robot_self.orientation as f32).to_radians()),
  };
  if trans_vector.y.is_sign_positive() && trans_vector.x.abs() < 15f32 {
    msg.dir = cp_data.cmd.orientation.unwrap_or_default() as u16;
    msg.speed = 500;

    return msg;
  } else if trans_vector.y.is_sign_positive() && trans_vector.x.abs() < 35f32 {
    msg.dir = capture_zone_to_ball;
    msg.speed = 500;

    return msg;
  }

  // else move behind the ball
  let target = point_at_distance_from_a(ball_pos, ball_pos - direction_vec, BALL_APPROACH_DISTANCE)
    .unwrap_or(ball_pos);

  let intent = NavIntent::GoToPosition {
    target_pos_mm: Vec2i::new(target.x as i32, target.y as i32),
    max_speed_mm_s: cp_data.cmd.speed.unwrap_or_default(),
  };
  orca.publish(OrcaRequest {
    intent,
    world: world.clone(),
  });

  msg = nav_command_to_teensy(msg, orca.latest());
  msg.speed = msg.speed.max(500);
  msg.orient = cp_data.cmd.orientation.unwrap_or_default() as u16;
  msg
}
