// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use udp_sender::{AppState::{AppState, connect, disconnect, send}, __cmd__connect, __cmd__disconnect, __cmd__send};


fn main() {
    tauri::Builder::default()
        .manage(AppState::new())
        .invoke_handler(tauri::generate_handler![connect, disconnect, send])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
