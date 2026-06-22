#[derive(Clone, Copy, PartialEq, Eq, Hash, ::prost::Message)]
pub struct CpBall {
  /// The position \[mm\] in the ssl-vision coordinate system
  #[prost(message, required, tag = "1")]
  pub pos: CpVector2,
  /// The velocity \[mm/s\] in the ssl-vision coordinate system
  #[prost(message, optional, tag = "2")]
  pub vel: Option<CpVector2>,
}
/// From the tracked ssl vision packet, removed unnecessary fields
/// A single tracked robot
#[derive(Clone, Copy, PartialEq, Eq, Hash, ::prost::Message)]
pub struct CpTrackedRobot {
  #[prost(uint32, required, tag = "1")]
  pub robot_id: u32,
  /// The position \[mm\] in the ssl-vision coordinate system
  #[prost(message, required, tag = "2")]
  pub pos: CpVector2,
  /// The orientation \[degree\] in the ssl-vision coordinate system
  #[prost(int32, required, tag = "3")]
  pub orientation: i32,
  /// The velocity \[m/s\] in the ssl-vision coordinate system
  #[prost(message, optional, tag = "4")]
  pub vel: Option<CpVector2>,
  /// The visibility, 0 means not visible, 255 means fully visible, the rest is in between
  #[prost(uint32, required, tag = "5")]
  pub visibility: u32,
}
#[derive(Clone, Copy, PartialEq, Eq, Hash, ::prost::Message)]
pub struct CpVector2 {
  #[prost(int32, required, tag = "1")]
  pub x: i32,
  #[prost(int32, required, tag = "2")]
  pub y: i32,
}
/// The message from the crash pilot to the robot
#[derive(Clone, PartialEq, ::prost::Message)]
pub struct CpRobot {
  /// Some fields to check stuff, drop all packets that are really late (for now 400ms) and the packet id should also be newer than the last one
  #[prost(uint32, required, tag = "1")]
  pub robot_id: u32,
  #[prost(message, required, tag = "2")]
  pub timestamp: prost_types::Timestamp,
  #[prost(uint32, required, tag = "3")]
  pub packet_id: u32,
  /// the ball data,
  #[prost(message, required, tag = "4")]
  pub ball: CpBall,
  /// The robots, the robot can extract their own position easily, because you should now your own robot id.
  #[prost(message, repeated, tag = "5")]
  pub robots_yellow: Vec<CpTrackedRobot>,
  #[prost(message, repeated, tag = "6")]
  pub robots_blue: Vec<CpTrackedRobot>,
  /// The actual command
  #[prost(message, required, tag = "7")]
  pub cmd: CpCommand,
  /// Info about the current game, including team, field setup etc
  #[prost(message, required, tag = "8")]
  pub infos: CpInfos,
}
/// The commands as enums and the fields are for stuff like drive to position and kick
#[derive(Clone, Copy, PartialEq, Eq, Hash, ::prost::Message)]
pub struct CpCommand {
  #[prost(enumeration = "CpState", required, tag = "1")]
  pub state: i32,
  #[prost(enumeration = "CpTask", required, tag = "2")]
  pub task: i32,
  #[prost(message, optional, tag = "3")]
  pub pos: Option<CpVector2>,
  #[prost(uint32, optional, tag = "4")]
  pub speed: Option<u32>,
  #[prost(uint32, optional, tag = "5")]
  pub orientation: Option<u32>,
  #[prost(uint32, optional, tag = "6")]
  pub kick_orient: Option<u32>,
  #[prost(uint32, optional, tag = "7")]
  pub kick_speed: Option<u32>,
  #[prost(uint32, optional, tag = "8")]
  pub enemy_id: Option<u32>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum CpState {
  StateUnspecified = 0,
  /// The GameController Halt Command
  /// Robot is not allowed to move
  StateHalt = 1,
  /// The GameController Stop Command
  /// Max velocity is 1.5m/s and distance to the ball should be 0.5m
  StateStop = 2,
  /// Should listen to CP_Task Commands
  StateFree = 3,
  /// This robot is the goalie, only listens to the GC_Task::Kick commands, to receive and kick the ball.
  StateGoalie = 4,
  /// Drive to the substitution area and turn all motors  off (we need to define the exact position for each robot at the start, so they don't touch
  /// when we call all our robots back)
  StateSubstitute = 5,
}
impl CpState {
  /// String value of the enum field names used in the ProtoBuf definition.
  ///
  /// The values are not transformed in any way and thus are considered stable
  /// (if the ProtoBuf definition does not change) and safe for programmatic use.
  pub fn as_str_name(&self) -> &'static str {
    match self {
      Self::StateUnspecified => "STATE_UNSPECIFIED",
      Self::StateHalt => "STATE_HALT",
      Self::StateStop => "STATE_STOP",
      Self::StateFree => "STATE_FREE",
      Self::StateGoalie => "STATE_GOALIE",
      Self::StateSubstitute => "STATE_SUBSTITUTE",
    }
  }
  /// Creates an enum from field names used in the ProtoBuf definition.
  pub fn from_str_name(value: &str) -> Option<Self> {
    match value {
      "STATE_UNSPECIFIED" => Some(Self::StateUnspecified),
      "STATE_HALT" => Some(Self::StateHalt),
      "STATE_STOP" => Some(Self::StateStop),
      "STATE_FREE" => Some(Self::StateFree),
      "STATE_GOALIE" => Some(Self::StateGoalie),
      "STATE_SUBSTITUTE" => Some(Self::StateSubstitute),
      _ => None,
    }
  }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, prost::Enumeration)]
#[repr(i32)]
pub enum CpTask {
  TaskUnspecified = 0,
  /// Drive to that position, if CP_State::STOP, max velocity is 1.5m/s
  TaskPos = 1,
  /// Kick the ball in the CP_Command::kick_orient direction
  TaskKick = 2,
  /// Chip the ball in the CP_Command::kick_orient direction
  TaskChip = 3,
  /// Receive the ball from the CP_Command::kick_orient direction
  TaskRecKick = 4,
  /// Try to steal the ball from another robot.
  /// This is to steal the ball from the ball capturing zone, the CrashPilot will try to position robots accordingly to intercept balls
  TaskSteal = 5,
  /// Dribble the ball to the CP_Command::pos position
  TaskDribble = 6,
  /// Get the ball and move it to the CP_Command::pos position
  TaskPosBall = 7,
  /// Moves the robot between the ball and an enemy, defined with the CP_Command::enemy_id value
  TaskBlock = 8,
  /// This robot should do a kickoff, basically kick the ball in the CP_Command::kick_orient direction, but adhere to the kickoff rules
  StateKickoff = 9,
  /// Free Kick, use the CP_Command::kick_orientation direction
  StateFreekick = 11,
}
impl CpTask {
  /// String value of the enum field names used in the ProtoBuf definition.
  ///
  /// The values are not transformed in any way and thus are considered stable
  /// (if the ProtoBuf definition does not change) and safe for programmatic use.
  pub fn as_str_name(&self) -> &'static str {
    match self {
      Self::TaskUnspecified => "TASK_UNSPECIFIED",
      Self::TaskPos => "TASK_POS",
      Self::TaskKick => "TASK_KICK",
      Self::TaskChip => "TASK_CHIP",
      Self::TaskRecKick => "TASK_REC_KICK",
      Self::TaskSteal => "TASK_STEAL",
      Self::TaskDribble => "TASK_DRIBBLE",
      Self::TaskPosBall => "TASK_PosBall",
      Self::TaskBlock => "TASK_BLOCK",
      Self::StateKickoff => "STATE_KICKOFF",
      Self::StateFreekick => "STATE_FREEKICK",
    }
  }
  /// Creates an enum from field names used in the ProtoBuf definition.
  pub fn from_str_name(value: &str) -> Option<Self> {
    match value {
      "TASK_UNSPECIFIED" => Some(Self::TaskUnspecified),
      "TASK_POS" => Some(Self::TaskPos),
      "TASK_KICK" => Some(Self::TaskKick),
      "TASK_CHIP" => Some(Self::TaskChip),
      "TASK_REC_KICK" => Some(Self::TaskRecKick),
      "TASK_STEAL" => Some(Self::TaskSteal),
      "TASK_DRIBBLE" => Some(Self::TaskDribble),
      "TASK_PosBall" => Some(Self::TaskPosBall),
      "STATE_KICKOFF" => Some(Self::StateKickoff),
      "STATE_FREEKICK" => Some(Self::StateFreekick),
      _ => None,
    }
  }
}
/// The packet the robot should send back
#[derive(Clone, Copy, PartialEq, Eq, Hash, ::prost::Message)]
pub struct RobotCp {
  #[prost(uint32, required, tag = "1")]
  pub robot_id: u32,
  #[prost(uint32, optional, tag = "2")]
  pub battery_voltage: Option<u32>,
  #[prost(uint32, optional, tag = "3")]
  pub current: Option<u32>,
  #[prost(bool, required, tag = "4")]
  pub kicker_ready: bool,
  #[prost(bool, required, tag = "5")]
  pub has_ball: bool,
  #[prost(bool, optional, tag = "6")]
  pub has_error: Option<bool>,
  #[prost(bool, optional, tag = "7")]
  pub acting: Option<bool>,
  #[prost(uint32, optional, tag = "8")]
  pub last_rec_packet: Option<u32>,
}
#[derive(Clone, Copy, PartialEq, Eq, Hash, ::prost::Message)]
pub struct CpInfos {
  /// false for yellow and true for blue
  #[prost(bool, required, tag = "1")]
  pub team_color: bool,
  /// Our defensive site
  /// false for x+ and true for x-
  #[prost(bool, required, tag = "2")]
  pub team_site: bool,
  /// General field info, used for orca
  /// Everything is in mm
  #[prost(uint32, required, tag = "3")]
  pub width: u32,
  #[prost(uint32, required, tag = "4")]
  pub height: u32,
  #[prost(uint32, required, tag = "5")]
  pub runoff_width: u32,
  #[prost(uint32, required, tag = "6")]
  pub penalty_area_width: u32,
  #[prost(uint32, required, tag = "7")]
  pub penalty_area_height: u32,
  #[prost(uint32, required, tag = "8")]
  pub goal_width: u32,
}
