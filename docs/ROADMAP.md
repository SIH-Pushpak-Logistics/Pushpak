# PUSHPAK · Roadmap

Better slides cannot raise PUSHPAK's technical credibility; only measurements can. Each item below replaces an estimate with a number, or a design with something that runs.

## The four phases (as submitted)

| Phase | Content | Status |
|---|---|---|
| **1 · Done in simulation** | GPS-denied flight on IMU + radar + ToF; search pattern, 10/10 waypoints; people detection at any angle; peer-to-peer link (also on two real laptops) + live dashboard; fail-safes FS-2, FS-4, FS-5; bench started with a flight controller and a laptop camera | Done (radar emulated) |
| **2 · Next, in simulation** | One filter (radar + rangefinder into EKF3, optical-flow fallback); hazards from thermal hotspots + fire/smoke; reports stored and re-sent after link loss; repeated fixes merged into one pin with an error circle; rescue order + suggested route; training on rotated images | Not started |
| **3 · Build hardware** | Full Drone B bench rig → hardware-in-the-loop; Orin Nano TensorRT timing + heat pipes; ducted frame + prop guards, OAK-D avoidance; measured endurance and ArduPilot fail-safes; Drone A with GNSS + thermal + NPU; power board rev B → PCB | Design rev A done (wiring, power-board concept) |
| **4 · Field and scale** | Trials at NDRF / SDRF training sites; Drone Rules 2021 registration and certified pilot; lat/long pins + photo confirmation; relay nodes dropped at void entrances; extension to floods and landslides; research on radar-inertial odometry | Not started |

## Next measurements, in priority order

| # | Measurement | Owner | Replaces |
|---|---|---|---|
| 1 | 12-rotation offline sweep + controlled 12 vs 24 timing | **Done 2 Oct** | 288/288, lowest 0.69; model call 35 vs 71 ms |
| 2 | Cause of the speed surging in GUIDED velocity mode (`ODOMETRY_ANALYSIS.md` §6) | Ashutosh | an unexplained 1.30 m/s against a 0.70 m/s command |
| 3 | Tr(Σv) during the bias hovers, quoted from the logs | Ashutosh | "the covariance was the same at every bias", without the value |
| 4 | Hover endurance and temperatures on the 5-inch test quad | Kanishk | single-digit-minutes estimate |
| 5 | Companion-computer power cut on the bench: what ArduPilot does and how fast | Kanishk / Ashutosh | "FC fail-safe: to measure" |
| 6 | YOLOv8n TensorRT on the Orin Nano under sustained load, with temperatures and clocks | Raunak / Kanishk | 77 ms on a laptop GPU |
| 7 | Jetson input power with all peripherals, to choose 12 V vs 19 V | Kanishk | the open power budget |
| 8 | Real IWR6843 on the bench with a Doppler ego-velocity estimator; bias at rest and in motion | Ashutosh / Kanishk | the emulated radar (largest credibility gain) |
| 9 | Detection on real or aerial search-and-rescue images, including partly covered people | Raunak | three rendered decals |

## Engineering backlog (simulation)

| Item | Why |
|---|---|
| Single EKF3 estimator; retire `ekf_node` + `vio_bridge_node` | IMU is fused twice today |
| Radar vs optical-flow consistency check; re-enable Pipeline A | Only way to see a velocity bias in flight |
| FS-3 (velocity lost → optical flow, else hold or land) | Designed; deferred from the demo build |
| FS-2 retrace to the last point with a link | Holding in place drifts |
| On-drone survivor log, sequence numbers, replay on reconnect, acks | Reports during an outage are lost |
| Merge repeated fixes; error circle on the dashboard | A falsely precise pin (victim_01, 1.6 m) |
| `collapse.sdf` rubble world; obstacle avoidance from OAK-D depth | Every flight so far is a 6 × 6 m box on clear ground |
| Drone A in simulation; a two-drone relay run | The two-tier claim is unproven; three live peers is the most tested |
| UWB reset in simulation | Bounded drift needs absolute resets |
| Commit the bench script and the track evaluator to `tools/` | Evidence tooling must live in the repo |

## Open risks

| # | Risk | Why it matters | What closes it |
|---|---|---|---|
| 1 | The GPS-free result rests on an emulated radar | The central claim | Measurement 8 |
| 2 | Detection tested on three rendered figures; in-flight confidence 41–84 % | Real survivors are partly buried and dusty | Measurement 9 |
| 3 | Orin Nano never timed; thermal behaviour unknown | Compute and heat budget | Measurement 6 |
| 4 | Jetson power budget does not close at 12 V | Board design | Measurement 7 |
| 5 | Endurance unmeasured | Sortie length and pack count | Measurement 4 |
| 6 | Survivor reports lost during link outages | Evidence lost when it matters most | Backlog: on-drone log + replay |
| 7 | No obstacle avoidance; clear-ground flights only | First real void flight may end wedged | OAK-D avoidance, ducted frame, rubble world |
| 8 | Drone A not simulated; two-drone run never done | Two-tier claim unproven | Drone A in simulation |
| 9 | Hazard detectors and datasets unnamed; no walkability layer | Route only as good as the map | Phase 2 hazard work |
| 10 | BOM out of date (Drone A additions; two parts listed at ₹0, not owned) | Cost claim | Re-priced BOM |
| 11 | Weather (rain, dust, monsoon wind at 50 m) not addressed | Field operation | Wind and dust limits per drone |
| 12 | Nothing fabricated in a hardware-category entry | A jury will ask to see it fly | Bench → HIL → first flight of the test quad |
| 13 | Drone Rules 2021 permissions | Needed before any field trial | Register early |
| 14 | Team split between cities | No joint live demo | Remote demos, shared rig schedule |
| 15 | Compute platform lock-in | The PS sponsor may expect other edge-AI hardware | The stack is ROS 2 plus a standard exported model; a porting study to other edge-AI boards is on the list once availability in India is checked |

## Hardware sequence

1. Bench: power and interface wiring on the 5-inch quad; measure endurance; ArduPilot on a supported flight controller.
2. Drone B bench rig: Jetson + flight controller + sensors on the bench, the same ROS 2 topics as simulation (only the producers change, README §5).
3. Hardware-in-the-loop: the flight stack on the Jetson against SITL.
4. First tethered and then free indoor flight of the test quad under the stack.
5. Drone B airframe; Drone A after the survey software exists in simulation.
