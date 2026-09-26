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

impl TelemetryMessage {
    pub fn drone_id(&self) -> u32 {
        match self {
            Self::Keyframe(m) => m.drone_id,
            Self::Survivor(m) => m.drone_id,
            Self::Heartbeat(m) => m.drone_id,
        }
    }
}

// Check the entire key, not just a prefix, and reject mismatched identities.
fn decode_sample(key: &str, bytes: &[u8]) -> Option<TelemetryMessage> {
    if bytes.len() > 50 {
        return None;
    }
    let parts: Vec<_> = key.split('/').collect();
    if parts.len() != 3 || parts[0] != "pushpak" {
        return None;
    }
    let id: u32 = parts[2].parse().ok()?;
    let message = match parts[1] {
        "keyframe" => TelemetryMessage::Keyframe(SubMapKeyframe::decode(bytes).ok()?),
        "survivor" => {
            let m = SurvivorEvent::decode(bytes).ok()?;
            if m.confidence_pct > 100 {
                return None;
            }
            TelemetryMessage::Survivor(m)
        }
        "heartbeat" => TelemetryMessage::Heartbeat(Heartbeat::decode(bytes).ok()?),
        _ => return None,
    };
    (message.drone_id() == id).then_some(message)
}

#[derive(Debug, thiserror::Error)]
pub enum TelemetryError {
    #[error("Zenoh error: {0}")]
    Zenoh(String),
    #[error("invalid telemetry: {0}")]
    Invalid(String),
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
        config
            .insert_json5("scouting/multicast/enabled", "true")
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
        let last_heartbeat = Arc::new(Mutex::new(HashMap::new()));
        let seen = Arc::clone(&last_heartbeat);
        session
            .declare_subscriber("pushpak/heartbeat/*")
            .callback(move |sample| {
                if let Some(TelemetryMessage::Heartbeat(m)) = decode_sample(
                    sample.key_expr().as_str(),
                    sample.payload().to_bytes().as_ref(),
                ) {
                    if m.drone_id != drone_id {
                        seen.lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .insert(m.drone_id, Instant::now());
                    }
                }
            })
            .background()
            .await
            .map_err(|error| TelemetryError::Zenoh(error.to_string()))?;
        Ok(Self {
            drone_id,
            session,
            last_heartbeat,
        })
    }

    pub fn drone_id(&self) -> u32 {
        self.drone_id
    }
    pub async fn publish_keyframe(&self, message: &SubMapKeyframe) -> Result<()> {
        self.publish("keyframe", message.drone_id, message).await
    }
    pub async fn publish_survivor(&self, message: &SurvivorEvent) -> Result<()> {
        if message.confidence_pct > 100 {
            return Err(TelemetryError::Invalid(
                "confidence_pct must be 0..=100".into(),
            ));
        }
        self.publish("survivor", message.drone_id, message).await
    }
    pub async fn publish_heartbeat(&self, message: &Heartbeat) -> Result<()> {
        self.publish("heartbeat", message.drone_id, message).await
    }

    async fn publish<M: Message>(&self, kind: &str, id: u32, message: &M) -> Result<()> {
        if id != self.drone_id || message.encoded_len() > 50 {
            return Err(TelemetryError::Invalid(
                "drone_id must match session and payload must be <=50 bytes".into(),
            ));
        }
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
        self.session
            .declare_subscriber("pushpak/**")
            .callback(move |sample| {
                let key = sample.key_expr().as_str();
                let bytes = sample.payload().to_bytes();
                let decoded = decode_sample(key, bytes.as_ref());
                if let Some(message) = decoded {
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
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .filter_map(|(&id, &seen)| {
                (id != self.drone_id && now.saturating_duration_since(seen) < timeout).then_some(id)
            })
            .collect::<Vec<_>>();
        peers.sort_unstable();
        peers
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_malformed_keys_and_identity_mismatch() {
        let bytes = Heartbeat {
            drone_id: 4,
            ..Default::default()
        }
        .encode_to_vec();
        assert!(decode_sample("pushpak/heartbeat/4", &bytes).is_some());
        for key in [
            "pushpak/heartbeat/3",
            "pushpak/heartbeat/4/extra",
            "other/heartbeat/4",
            "pushpak/heartbeat/no",
        ] {
            assert!(decode_sample(key, &bytes).is_none());
        }
        assert!(decode_sample("pushpak/heartbeat/4", &[255]).is_none());
    }

    #[test]
    fn frozen_schema_full_integer_extremes_exceed_budget() {
        let m = SubMapKeyframe {
            timestamp_ms: u32::MAX,
            drone_id: u32::MAX,
            pos_x_mm: i32::MIN,
            pos_y_mm: i32::MIN,
            pos_z_mm: i32::MIN,
            roll_cdeg: i32::MIN,
            pitch_cdeg: i32::MIN,
            yaw_cdeg: i32::MIN,
            status_flags: u32::MAX,
        };
        assert_eq!(m.encoded_len(), 54);
        assert!(decode_sample(
            &format!("pushpak/keyframe/{}", u32::MAX),
            &m.encode_to_vec()
        )
        .is_none());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn direct_peers_exchange_all_messages_and_expire_without_router() {
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let endpoint = format!("tcp/127.0.0.1:{port}");
        let a = Telemetry::open_peer_with_config(30, Vec::<String>::new(), [&endpoint])
            .await
            .unwrap();
        let b = Telemetry::open_peer_with_endpoints(31, [&endpoint])
            .await
            .unwrap();
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
        a.subscribe_all(move |m| {
            let _ = tx.send(m);
        })
        .await
        .unwrap();
        let heartbeat = Heartbeat {
            drone_id: 31,
            ..Default::default()
        };
        assert!(a.publish_heartbeat(&heartbeat).await.is_err());
        tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                b.publish_heartbeat(&heartbeat).await.unwrap();
                b.publish_keyframe(&SubMapKeyframe {
                    drone_id: 31,
                    ..Default::default()
                })
                .await
                .unwrap();
                b.publish_survivor(&SurvivorEvent {
                    drone_id: 31,
                    confidence_pct: 99,
                    ..Default::default()
                })
                .await
                .unwrap();
                a.publish_heartbeat(&Heartbeat {
                    drone_id: 30,
                    ..Default::default()
                })
                .await
                .unwrap();
                tokio::time::sleep(Duration::from_millis(100)).await;
                let mut kinds = [false; 3];
                while let Ok(m) = rx.try_recv() {
                    if m.drone_id() == 31 {
                        kinds[match m {
                            TelemetryMessage::Heartbeat(_) => 0,
                            TelemetryMessage::Keyframe(_) => 1,
                            TelemetryMessage::Survivor(_) => 2,
                        }] = true;
                    }
                }
                if kinds.iter().all(|v| *v) && b.peers_alive(Duration::from_secs(2)).contains(&30) {
                    break;
                }
            }
        })
        .await
        .unwrap();
        assert!(a.peers_alive(Duration::from_secs(2)).contains(&31));
        drop(b);
        tokio::time::sleep(Duration::from_millis(300)).await;
        assert!(a.peers_alive(Duration::from_millis(200)).is_empty());
    }
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
