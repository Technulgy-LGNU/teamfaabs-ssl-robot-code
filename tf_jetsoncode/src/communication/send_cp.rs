use crate::config;
use crate::proto::RobotCp;
use prost::Message;
use std::net::{SocketAddr, SocketAddrV4};
use tokio::net::UdpSocket;

pub async fn send_cp(cfg: &config::Config, socket: &UdpSocket, msg: RobotCp) {
  let mut buf: Vec<u8> = Vec::with_capacity(msg.encoded_len());

  if let Err(e) = msg.encode(&mut buf) {
    println!("Failed to encode message: {}", e);
    return;
  }
  if buf.is_empty() {
    println!("Failed to encode message, buffer is empty");
    return;
  }

  let addr = SocketAddr::V4(SocketAddrV4::new(
    cfg.cp_config.host,
    cfg.cp_config.port_outgoing,
  ));
  match socket.send_to(&buf, addr).await {
    Ok(_) => (),
    Err(e) => {
      println!("Failed to send message: {}", e);
    }
  }
}
