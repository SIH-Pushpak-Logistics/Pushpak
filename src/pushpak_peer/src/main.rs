use clap::Parser;
use pushpak_telemetry::{Heartbeat, SubMapKeyframe, Telemetry, TelemetryMessage};
use std::{path::PathBuf, sync::Arc, time::Duration};
use tokio::{fs, time};

#[derive(Parser, Debug)]
#[command(about = "Routerless Pushpak Zenoh telemetry peer")]
struct Args {
    #[arg(long)]
    drone_id: u32,
    #[arg(long)]
    keyframes: PathBuf,
    #[arg(long = "connect", value_name = "tcp/IP:7447")]
    connect: Vec<String>,
    #[arg(long = "listen", value_name = "tcp/IP:7447")]
    listen: Vec<String>,
}

#[derive(Clone, Copy)]
struct CsvKeyframe {
    timestamp_ms: u32,
    x_mm: i32,
    y_mm: i32,
    z_mm: i32,
    yaw_cdeg: i32,
}

fn parse_keyframes(input: &str) -> Result<Vec<CsvKeyframe>, String> {
    let mut keyframes = Vec::new();
    for (index, line) in input.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || (index == 0 && line.starts_with("t_ms,")) {
            continue;
        }
        let values = line.split(',').map(str::trim).collect::<Vec<_>>();
        if values.len() != 5 {
            return Err(format!("line {}: expected 5 fields", index + 1));
        }
        keyframes.push(CsvKeyframe {
            timestamp_ms: values[0]
                .parse()
                .map_err(|_| format!("line {}: invalid t_ms", index + 1))?,
            x_mm: values[1]
                .parse()
                .map_err(|_| format!("line {}: invalid x_mm", index + 1))?,
            y_mm: values[2]
                .parse()
                .map_err(|_| format!("line {}: invalid y_mm", index + 1))?,
            z_mm: values[3]
                .parse()
                .map_err(|_| format!("line {}: invalid z_mm", index + 1))?,
            yaw_cdeg: values[4]
                .parse()
                .map_err(|_| format!("line {}: invalid yaw_cdeg", index + 1))?,
        });
    }
    if keyframes.is_empty() {
        Err("keyframe log contains no rows".into())
    } else {
        Ok(keyframes)
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let keyframes = parse_keyframes(&fs::read_to_string(&args.keyframes).await?)
        .map_err(std::io::Error::other)?;
    let telemetry = Arc::new(
        Telemetry::open_peer_with_config(args.drone_id, &args.connect, &args.listen).await?,
    );
    let own_id = args.drone_id;
    telemetry
        .subscribe_all(move |message| match message {
            TelemetryMessage::Heartbeat(m) if m.drone_id == own_id => {}
            TelemetryMessage::Keyframe(m) if m.drone_id == own_id => {}
            TelemetryMessage::Survivor(m) if m.drone_id == own_id => {}
            TelemetryMessage::Heartbeat(m) => println!(
                "HEARTBEAT peer={} t_ms={} flags={}",
                m.drone_id, m.timestamp_ms, m.status_flags
            ),
            TelemetryMessage::Keyframe(m) => println!(
                "KEYFRAME peer={} t_ms={} pos=({},{},{})",
                m.drone_id, m.timestamp_ms, m.pos_x_mm, m.pos_y_mm, m.pos_z_mm
            ),
            TelemetryMessage::Survivor(m) => println!(
                "SURVIVOR peer={} id={} confidence={}pct",
                m.drone_id, m.survivor_id, m.confidence_pct
            ),
        })
        .await?;
    println!(
        "pushpak_peer {} ready (Zenoh peer mode; no zenohd)",
        args.drone_id
    );

    let heartbeat_peer = Arc::clone(&telemetry);
    let heartbeat_task = tokio::spawn(async move {
        let started = time::Instant::now();
        let mut interval = time::interval(Duration::from_millis(500));
        loop {
            interval.tick().await;
            let m = Heartbeat {
                timestamp_ms: started.elapsed().as_millis() as u32,
                drone_id: heartbeat_peer.drone_id(),
                status_flags: 0,
            };
            if let Err(e) = heartbeat_peer.publish_heartbeat(&m).await {
                eprintln!("heartbeat publish failed: {e}");
            }
        }
    });
    let keyframe_peer = Arc::clone(&telemetry);
    let keyframe_task = tokio::spawn(async move {
        let mut index = 0usize;
        let mut interval = time::interval(Duration::from_millis(200));
        loop {
            interval.tick().await;
            let row = keyframes[index];
            let m = SubMapKeyframe {
                timestamp_ms: row.timestamp_ms,
                drone_id: keyframe_peer.drone_id(),
                pos_x_mm: row.x_mm,
                pos_y_mm: row.y_mm,
                pos_z_mm: row.z_mm,
                roll_cdeg: 0,
                pitch_cdeg: 0,
                yaw_cdeg: row.yaw_cdeg,
                status_flags: 0,
            };
            if let Err(e) = keyframe_peer.publish_keyframe(&m).await {
                eprintln!("keyframe publish failed: {e}");
            }
            index = (index + 1) % keyframes.len();
        }
    });
    let liveness_peer = Arc::clone(&telemetry);
    let liveness_task = tokio::spawn(async move {
        let mut previous = Vec::new();
        let mut interval = time::interval(Duration::from_millis(250));
        loop {
            interval.tick().await;
            let current = liveness_peer.peers_alive(Duration::from_secs(2));
            for id in previous.iter().filter(|id| !current.contains(id)) {
                println!("PEER LOST {id}");
            }
            for id in current.iter().filter(|id| !previous.contains(id)) {
                println!("PEER ALIVE {id}");
            }
            previous = current;
        }
    });
    tokio::signal::ctrl_c().await?;
    heartbeat_task.abort();
    keyframe_task.abort();
    liveness_task.abort();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn parses_header_and_rows() {
        let rows = parse_keyframes("t_ms,x_mm,y_mm,z_mm,yaw_cdeg\n0,1,-2,3,900\n").unwrap();
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].y_mm, -2);
    }
}
