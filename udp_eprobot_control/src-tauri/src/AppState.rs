use std::{sync::Mutex, net::UdpSocket};

pub struct AppState {
  udp_: Mutex<Option<UdpSocket>>
}

#[derive(serde::Deserialize)]
pub struct ConnectConfig {
  pub ip: String,
  pub port: String
}

#[derive(serde::Deserialize, serde::Serialize , Debug)]
pub struct ControlOffset {
  pub x: f32,
  pub z: f32,
}

impl AppState {
  pub fn new() -> Self {
    Self {
      udp_: Mutex::new(None)
    }
  }
}

#[tauri::command]
pub async fn connect(cfg: ConnectConfig, _state: tauri::State<'_, AppState>) -> Result<(), String> {
  let addr = format!("{}:{}", cfg.ip, cfg.port);
  let socket = UdpSocket::bind("127.0.0.1:8000").map_err(|e| e.to_string())?;
  socket.connect(addr).map_err(|e| e.to_string())?;
  *_state.udp_.lock().unwrap() = Some(socket);
  Ok(())
}

#[tauri::command]
pub async fn disconnect(_state: tauri::State<'_, AppState>) -> Result<(), String> {
  *_state.udp_.lock().unwrap() = None;
  Ok(())
}

#[tauri::command]
pub async fn send(data: ControlOffset, _state: tauri::State<'_, AppState>) -> Result<(), String> {
  let socket = _state.udp_.lock().unwrap();
  if let Some(socket) = &*socket {
    let data = serde_json::to_string(&data);
    if let Ok(data_str) = data {
      println!("data: {}", data_str);
      socket.send(data_str.as_bytes()).map_err(|e| e.to_string())?;
    } else {
      return Err("data error".to_string());
    }
  }
  Ok(())
}
