use crate::communication::EventShare;
use crate::config;
use crate::proto::CpRobot;
use prost::Message;

pub async fn receive_cp(cfg: &config::Config, tx: EventShare) {
  let cp_socket: tokio::net::UdpSocket =
    match tokio::net::UdpSocket::bind(format!("0.0.0.0:{}", cfg.cp_config.port)).await {
      Ok(s) => s,
      Err(e) => {
        eprintln!(
          "Failed to bind UDP socket for CP with port {}: {}",
          cfg.cp_config.port, e
        );
        return;
      }
    };

  tokio::spawn(async move {
    let mut buf = [0u8; 1024];

    loop {
      match cp_socket.recv_from(&mut buf).await {
        Ok((size, _)) => {
          if let Ok(msg) = CpRobot::decode(&buf[..size]) {
            let mut lock = tx.lock().await;

            lock.0 = Some(msg);
          }
        }
        Err(e) => {
          eprintln!("Failed to read CP UDP: {}", e);
        }
      }
    }
  });
}
