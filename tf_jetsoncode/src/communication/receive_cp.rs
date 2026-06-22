use crate::communication::EventShare;
use crate::config;
use core_dump::proto::CpRobot;
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
    let mut buf = [0u8; 65536];

    loop {
      match cp_socket.recv_from(&mut buf).await {
        Ok((size, _)) => {
          if let Ok(mut latest_msg) = CpRobot::decode(&buf[..size]) {
            // Drain all buffered packets, keeping only the most recent
            loop {
              match cp_socket.try_recv_from(&mut buf) {
                Ok((size, _)) => {
                  if let Ok(msg) = CpRobot::decode(&buf[..size]) {
                    latest_msg = msg;
                  }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => {
                  eprintln!("recv error: {:?}", e);
                  break;
                }
              }
            }

            let mut lock = tx.write().await;
            lock.cp = Some(latest_msg);
          }
        }
        Err(e) => {
          eprintln!("recv error: {:?}", e);
        }
      }
    }
  });
}
