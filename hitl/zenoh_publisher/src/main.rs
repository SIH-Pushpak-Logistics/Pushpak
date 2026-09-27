use clap::Parser;
use pushpak_telemetry::{Heartbeat, SubMapKeyframe, SurvivorEvent, Telemetry, TelemetryMessage};
use serde::Deserialize;
use std::{error::Error, time::Duration};
use tokio::io::{self, AsyncBufReadExt, BufReader};

#[derive(Parser)]
#[command(about = "ROS adapter stdin to routerless Pushpak Zenoh telemetry")]
struct Args {
    #[arg(long, default_value_t = 2)]
    drone_id: u32,
    #[arg(long = "connect", value_name = "tcp/IP:7447")]
    connect: Vec<String>,
    #[arg(long = "listen", value_name = "tcp/IP:7447")]
    listen: Vec<String>,
}

// The ROS adapter owns rates and ROS timestamps. This process owns protobuf
// encoding and the Zenoh session; the adapter never needs a Zenoh dependency.
#[derive(Deserialize, Debug)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Input {
    Heartbeat {
        timestamp_ms: u32,
        status_flags: u32,
    },
    Keyframe {
        timestamp_ms: u32,
        pos_x_mm: i32,
        pos_y_mm: i32,
        pos_z_mm: i32,
        roll_cdeg: i32,
        pitch_cdeg: i32,
        yaw_cdeg: i32,
        status_flags: u32,
    },
    Survivor {
        timestamp_ms: u32,
        survivor_id: u32,
        pos_x_mm: i32,
        pos_y_mm: i32,
        pos_z_mm: i32,
        confidence_pct: u32,
        hit_count: u32,
    },
}

async fn publish(telemetry: &Telemetry, input: Input) -> pushpak_telemetry::Result<()> {
    let drone_id = telemetry.drone_id();
    match input {
        Input::Heartbeat {
            timestamp_ms,
            status_flags,
        } => {
            telemetry
                .publish_heartbeat(&Heartbeat {
                    timestamp_ms,
                    drone_id,
                    status_flags,
                })
                .await
        }
        Input::Keyframe {
            timestamp_ms,
            pos_x_mm,
            pos_y_mm,
            pos_z_mm,
            roll_cdeg,
            pitch_cdeg,
            yaw_cdeg,
            status_flags,
        } => {
            telemetry
                .publish_keyframe(&SubMapKeyframe {
                    timestamp_ms,
                    drone_id,
                    pos_x_mm,
                    pos_y_mm,
                    pos_z_mm,
                    roll_cdeg,
                    pitch_cdeg,
                    yaw_cdeg,
                    status_flags,
                })
                .await
        }
        Input::Survivor {
            timestamp_ms,
            survivor_id,
            pos_x_mm,
            pos_y_mm,
            pos_z_mm,
            confidence_pct,
            hit_count,
        } => {
            telemetry
                .publish_survivor(&SurvivorEvent {
                    timestamp_ms,
                    drone_id,
                    survivor_id,
                    pos_x_mm,
                    pos_y_mm,
                    pos_z_mm,
                    confidence_pct,
                    hit_count,
                })
                .await
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.drone_id != 2 {
        return Err("HITL rig is assigned drone_id=2 by the frozen contract".into());
    }
    let telemetry =
        Telemetry::open_peer_with_config(args.drone_id, &args.connect, &args.listen).await?;
    telemetry
        .subscribe_all(|message| {
            let id = message.drone_id();
            if id != 2 {
                match message {
                    TelemetryMessage::Heartbeat(_) => println!("HEARTBEAT peer={id}"),
                    TelemetryMessage::Keyframe(_) => println!("KEYFRAME peer={id}"),
                    TelemetryMessage::Survivor(_) => println!("SURVIVOR peer={id}"),
                }
            }
        })
        .await?;
    println!("HITL Zenoh peer 2 ready (no zenohd)");

    let mut lines = BufReader::new(io::stdin()).lines();
    let mut previous = Vec::new();
    let mut poll = tokio::time::interval(Duration::from_millis(100));
    poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            line = lines.next_line() => match line? {
                Some(line) => {
                    match serde_json::from_str::<Input>(&line) {
                        Ok(input) => {
                            if let Err(error) = publish(&telemetry, input).await {
                                eprintln!("telemetry publish failed: {error}");
                            }
                        }
                        Err(error) => eprintln!("invalid ROS adapter input: {error}"),
                    }
                }
                None => break,
            },
            _ = poll.tick() => {
                let current = telemetry.peers_alive(Duration::from_millis(1750));
                for id in previous.iter().filter(|id| !current.contains(id)) { println!("PEER LOST {id}"); }
                for id in current.iter().filter(|id| !previous.contains(id)) { println!("PEER ALIVE {id}"); }
                previous = current;
            }
            _ = tokio::signal::ctrl_c() => break,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_each_ros_adapter_message() {
        for line in [
            r#"{"kind":"heartbeat","timestamp_ms":100,"status_flags":0}"#,
            r#"{"kind":"keyframe","timestamp_ms":100,"pos_x_mm":1,"pos_y_mm":2,"pos_z_mm":3,"roll_cdeg":0,"pitch_cdeg":0,"yaw_cdeg":90,"status_flags":0}"#,
            r#"{"kind":"survivor","timestamp_ms":100,"survivor_id":1,"pos_x_mm":1,"pos_y_mm":2,"pos_z_mm":3,"confidence_pct":85,"hit_count":1}"#,
        ] {
            serde_json::from_str::<Input>(line).unwrap();
        }
        assert!(serde_json::from_str::<Input>(r#"{"kind":"heartbeat","timestamp_ms":1}"#).is_err());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn publishes_adapter_input_to_a_direct_peer() {
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let endpoint = format!("tcp/127.0.0.1:{port}");
        let receiver = Telemetry::open_peer_with_config(1, Vec::<String>::new(), [&endpoint])
            .await
            .unwrap();
        let publisher = Telemetry::open_peer_with_endpoints(2, [&endpoint])
            .await
            .unwrap();
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
        receiver
            .subscribe_all(move |message| {
                let _ = tx.send(message);
            })
            .await
            .unwrap();
        let lines = [
            r#"{"kind":"heartbeat","timestamp_ms":100,"status_flags":0}"#,
            r#"{"kind":"keyframe","timestamp_ms":100,"pos_x_mm":1000,"pos_y_mm":0,"pos_z_mm":1000,"roll_cdeg":0,"pitch_cdeg":0,"yaw_cdeg":9000,"status_flags":0}"#,
            r#"{"kind":"survivor","timestamp_ms":100,"survivor_id":7,"pos_x_mm":1000,"pos_y_mm":0,"pos_z_mm":1000,"confidence_pct":85,"hit_count":2}"#,
        ];
        tokio::time::timeout(Duration::from_secs(10), async {
            let mut received = [false; 3];
            while !received.iter().all(|v| *v) {
                for line in lines {
                    publish(&publisher, serde_json::from_str(line).unwrap())
                        .await
                        .unwrap();
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
                while let Ok(message) = rx.try_recv() {
                    match message {
                        TelemetryMessage::Heartbeat(m) if m.drone_id == 2 => received[0] = true,
                        TelemetryMessage::Keyframe(m) if m.drone_id == 2 && m.yaw_cdeg == 9000 => {
                            received[1] = true
                        }
                        TelemetryMessage::Survivor(m)
                            if m.drone_id == 2 && m.survivor_id == 7 && m.hit_count == 2 =>
                        {
                            received[2] = true
                        }
                        _ => {}
                    }
                }
            }
        })
        .await
        .unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn remaining_peers_keep_exchanging_after_ground_station_stops() {
        fn free_port() -> u16 {
            std::net::TcpListener::bind("127.0.0.1:0")
                .unwrap()
                .local_addr()
                .unwrap()
                .port()
        }
        let ground_endpoint = format!("tcp/127.0.0.1:{}", free_port());
        let sitl_endpoint = format!("tcp/127.0.0.1:{}", free_port());
        let ground = Telemetry::open_peer_with_config(0, Vec::<String>::new(), [&ground_endpoint])
            .await
            .unwrap();
        let sitl = Telemetry::open_peer_with_config(1, [&ground_endpoint], [&sitl_endpoint])
            .await
            .unwrap();
        let rig = Telemetry::open_peer_with_endpoints(2, [&ground_endpoint, &sitl_endpoint])
            .await
            .unwrap();

        tokio::time::timeout(Duration::from_secs(10), async {
            loop {
                ground
                    .publish_heartbeat(&Heartbeat {
                        drone_id: 0,
                        ..Default::default()
                    })
                    .await
                    .unwrap();
                sitl.publish_heartbeat(&Heartbeat {
                    drone_id: 1,
                    ..Default::default()
                })
                .await
                .unwrap();
                rig.publish_heartbeat(&Heartbeat {
                    drone_id: 2,
                    ..Default::default()
                })
                .await
                .unwrap();
                tokio::time::sleep(Duration::from_millis(100)).await;
                if sitl.peers_alive(Duration::from_secs(2)).contains(&2)
                    && sitl.peers_alive(Duration::from_secs(2)).contains(&0)
                    && rig.peers_alive(Duration::from_secs(2)).contains(&1)
                    && rig.peers_alive(Duration::from_secs(2)).contains(&0)
                {
                    break;
                }
            }
        })
        .await
        .unwrap();

        drop(ground);
        tokio::time::sleep(Duration::from_millis(1900)).await;
        sitl.publish_heartbeat(&Heartbeat {
            drone_id: 1,
            ..Default::default()
        })
        .await
        .unwrap();
        rig.publish_heartbeat(&Heartbeat {
            drone_id: 2,
            ..Default::default()
        })
        .await
        .unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(!sitl.peers_alive(Duration::from_millis(1750)).contains(&0));
        assert!(!rig.peers_alive(Duration::from_millis(1750)).contains(&0));
        assert!(sitl.peers_alive(Duration::from_millis(1750)).contains(&2));
        assert!(rig.peers_alive(Duration::from_millis(1750)).contains(&1));
    }
}
