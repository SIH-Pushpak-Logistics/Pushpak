//! Routerless Pushpak telemetry over Zenoh peer mode.

use prost::Message;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
use zenoh::{Config, Session};

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/swarm.telemetry.rs"));
}
pub use proto::{Heartbeat, SubMapKeyframe, SurvivorEvent};

#[derive(Debug, Clone)]
pub enum TelemetryMessage {
    Keyframe(SubMapKeyframe),
    Survivor(SurvivorEvent),
    Heartbeat(Heartbeat),
}

#[derive(Debug, thiserror::Error)]
pub enum TelemetryError {
    #[error("Zenoh error: {0}")]
    Zenoh(String),
}
pub type Result<T> = std::result::Result<T, TelemetryError>;

pub struct Telemetry {
    drone_id: u32,
    session: Session,
    last_heartbeat: Arc<Mutex<HashMap<u32, Instant>>>,
}

impl Telemetry {
    pub async fn open_peer(drone_id: u32) -> Result<Self> {
        Self::open_peer_with_endpoints(drone_id, std::iter::empty::<String>()).await
    }

    pub async fn open_peer_with_endpoints<I, S>(drone_id: u32, endpoints: I) -> Result<Self>
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        Self::open_peer_with_config(drone_id, endpoints, std::iter::empty::<String>()).await
    }

    pub async fn open_peer_with_config<CI, CS, LI, LS>(
        drone_id: u32,
        connect_endpoints: CI,
        listen_endpoints: LI,
    ) -> Result<Self>
    where
        CI: IntoIterator<Item = CS>,
        CS: AsRef<str>,
        LI: IntoIterator<Item = LS>,
        LS: AsRef<str>,
    {
        let mut config = Config::default();
        config
            .insert_json5("mode", "\"peer\"")
            .map_err(|error| TelemetryError::Zenoh(error.to_string()))?;
        let endpoints = connect_endpoints
            .into_iter()
            .map(|endpoint| endpoint.as_ref().to_owned())
            .collect::<Vec<_>>();
        if !endpoints.is_empty() {
            config
                .insert_json5(
                    "connect/endpoints",
                    &serde_json::to_string(&endpoints).expect("endpoint list is serializable"),
                )
                .map_err(|error| TelemetryError::Zenoh(error.to_string()))?;
        }
        let listen_endpoints = listen_endpoints
            .into_iter()
            .map(|endpoint| endpoint.as_ref().to_owned())
            .collect::<Vec<_>>();
        if !listen_endpoints.is_empty() {
            config
                .insert_json5(
                    "listen/endpoints",
                    &serde_json::to_string(&listen_endpoints)
                        .expect("endpoint list is serializable"),
                )
                .map_err(|error| TelemetryError::Zenoh(error.to_string()))?;
        }
        let session = zenoh::open(config)
            .await
            .map_err(|error| TelemetryError::Zenoh(error.to_string()))?;
        Ok(Self {
            drone_id,
            session,
            last_heartbeat: Arc::new(Mutex::new(HashMap::new())),
        })
    }

    pub fn drone_id(&self) -> u32 {
        self.drone_id
    }
    pub async fn publish_keyframe(&self, message: &SubMapKeyframe) -> Result<()> {
        self.publish("keyframe", message).await
    }
    pub async fn publish_survivor(&self, message: &SurvivorEvent) -> Result<()> {
        self.publish("survivor", message).await
    }
    pub async fn publish_heartbeat(&self, message: &Heartbeat) -> Result<()> {
        self.publish("heartbeat", message).await
    }

    async fn publish<M: Message>(&self, kind: &str, message: &M) -> Result<()> {
        self.session
            .put(
                format!("pushpak/{kind}/{}", self.drone_id),
                message.encode_to_vec(),
            )
            .await
            .map_err(|error| TelemetryError::Zenoh(error.to_string()))
    }

    pub async fn subscribe_all<F>(&self, callback: F) -> Result<()>
    where
        F: Fn(TelemetryMessage) + Send + Sync + 'static,
    {
        let heartbeats = Arc::clone(&self.last_heartbeat);
        self.session
            .declare_subscriber("pushpak/**")
            .callback(move |sample| {
                let key = sample.key_expr().as_str();
                let bytes = sample.payload().to_bytes();
                let decoded = if key.starts_with("pushpak/keyframe/") {
                    SubMapKeyframe::decode(bytes.as_ref())
                        .ok()
                        .map(TelemetryMessage::Keyframe)
                } else if key.starts_with("pushpak/survivor/") {
                    SurvivorEvent::decode(bytes.as_ref())
                        .ok()
                        .map(TelemetryMessage::Survivor)
                } else if key.starts_with("pushpak/heartbeat/") {
                    Heartbeat::decode(bytes.as_ref())
                        .ok()
                        .map(TelemetryMessage::Heartbeat)
                } else {
                    None
                };
                if let Some(message) = decoded {
                    if let TelemetryMessage::Heartbeat(heartbeat) = &message {
                        if let Ok(mut seen) = heartbeats.lock() {
                            seen.insert(heartbeat.drone_id, Instant::now());
                        }
                    }
                    callback(message);
                }
            })
            .background()
            .await
            .map_err(|error| TelemetryError::Zenoh(error.to_string()))
    }

    pub fn peers_alive(&self, timeout: Duration) -> Vec<u32> {
        let now = Instant::now();
        let mut peers = self
            .last_heartbeat
            .lock()
            .expect("heartbeat table mutex poisoned")
            .iter()
            .filter_map(|(&id, &seen)| {
                (id != self.drone_id && now.duration_since(seen) <= timeout).then_some(id)
            })
            .collect::<Vec<_>>();
        peers.sort_unstable();
        peers
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn round_trip<M: Message + Default + PartialEq + std::fmt::Debug>(message: M) {
        let bytes = message.encode_to_vec();
        assert_eq!(message, M::decode(bytes.as_slice()).unwrap());
    }

    #[test]
    fn all_messages_round_trip() {
        round_trip(Heartbeat {
            timestamp_ms: 42,
            drone_id: 2,
            status_flags: 7,
        });
        round_trip(SubMapKeyframe {
            timestamp_ms: 42,
            drone_id: 2,
            pos_x_mm: -1_000,
            pos_y_mm: 2_000,
            pos_z_mm: 3_000,
            roll_cdeg: -900,
            pitch_cdeg: 450,
            yaw_cdeg: 18_000,
            status_flags: 7,
        });
        round_trip(SurvivorEvent {
            timestamp_ms: 42,
            drone_id: 2,
            survivor_id: 9,
            pos_x_mm: -1_000,
            pos_y_mm: 2_000,
            pos_z_mm: 3_000,
            confidence_pct: 99,
            hit_count: 4,
        });
    }

    #[test]
    fn operational_extremes_fit_50_bytes() {
        let messages = [
            Heartbeat {
                timestamp_ms: u32::MAX,
                drone_id: u32::MAX,
                status_flags: u32::MAX,
            }
            .encode_to_vec(),
            SubMapKeyframe {
                timestamp_ms: u32::MAX,
                drone_id: u32::MAX,
                pos_x_mm: i32::MIN,
                pos_y_mm: i32::MAX,
                pos_z_mm: i32::MIN,
                roll_cdeg: -18_000,
                pitch_cdeg: 18_000,
                yaw_cdeg: 36_000,
                status_flags: u32::MAX,
            }
            .encode_to_vec(),
            SurvivorEvent {
                timestamp_ms: u32::MAX,
                drone_id: u32::MAX,
                survivor_id: u32::MAX,
                pos_x_mm: i32::MIN,
                pos_y_mm: i32::MAX,
                pos_z_mm: i32::MIN,
                confidence_pct: 100,
                hit_count: u32::MAX,
            }
            .encode_to_vec(),
        ];
        for bytes in messages {
            assert!(
                bytes.len() <= 50,
                "serialized message is {} bytes",
                bytes.len()
            );
        }
    }
}
