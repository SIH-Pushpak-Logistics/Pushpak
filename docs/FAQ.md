# PUSHPAK · Frequently asked questions

Hard questions reviewers have asked about PUSHPAK, answered from this repository. Where something is not measured or not decided, the answer says so. Numbers are from [`EVIDENCE.md`](EVIDENCE.md); E-numbers refer to rows in that file.

**Status in one line.** Simulation results are measured. Hardware is designed and priced, not built. Nothing has flown.

## Finding people

### 1. In a dark, dusty void, which sensor finds the person?

Today, an RGB camera with YOLOv8n. It has been tested on clear ground in simulation and on one dim-room bench clip. The radar measures the drone's own velocity; it does not find people. A FLIR Lepton 3.5 thermal camera on the scout drone is planned for dark and dust. Aligning thermal with RGB and the rule for combining them are not designed yet (`MISSION_DESIGN.md` §3).

### 2. Are 3 / 3 and 288 / 288 recall figures?

No. 3 / 3 is three rendered figures on clear ground in two simulated flights (E12, E14). 288 / 288 is three clean rendered images at 96 body angles each (E15, E19). They show the pipeline works end to end. Partly buried or dusty people are untested.

### 3. The only real-camera result is a box in 55 % of frames. Is that good enough?

It is one person, one room, 21 seconds (E21). The longest gap between boxes was about 0.7 s, so the person was not lost for long, but it is not a recall measurement. The next step is detection on real or aerial search-and-rescue images, including partly covered people (`ROADMAP.md`, measurement 9). The dataset and the target number are not chosen yet.

### 4. The fire and smoke scores come from D-Fire. Is that aerial imagery?

No. D-Fire is ground-level and surveillance views. The scores (mAP50 0.81 smoke, 0.70 fire, E20) are dataset scores, not flight results. The model is not in the flight software and has not run in simulation. Aerial training data is not chosen; there are no water or damage classes yet.

### 5. From 30–50 m, a person is a few pixels on a 160 × 120 thermal camera.

The Lepton 3.5 is on the scout drone, which works at close range. The survey drone carries a radiometric thermal core to flag hotspots and fire. The design does not rely on the survey drone seeing people: it ranks which structure to search first from building type, time of day and visible damage (`MISSION_DESIGN.md` §2).

### 6. If the scout finds nobody, is the building clear?

No. A camera cannot see through rubble. The scout covers open voids and the surface; dogs and life detectors confirm. The system never rules a building out.

## Flying without GPS

### 7. The radar is emulated. What changes with a real one?

The simulated radar is truth plus white noise (σ 0.15 m/s). A real radar adds bias, scale-factor and mounting errors and needs an ego-velocity estimator. With a constant bias the position error grows as bias × time: 3.2 m in 60 s at 0.05 m/s (E4), and the filter cannot see it. A real TI IWR6843 on the bench is measurement 8, before any flight.

### 8. What bounds the drift?

Nothing yet. The design uses UWB markers at the entrance to reset position whenever the drone is in range, and a radar against optical-flow check to catch a growing bias (`GPS_INDEPENDENCE.md`). Neither is built.

### 9. What is the yaw source with no compass?

Gyro integration only. The simulated gyro has no bias, so the 0.02° result is optimistic. The planned correction is two UWB tags on the airframe, or a heading fit from a straight leg between two UWB fixes (`GPS_INDEPENDENCE.md`).

### 10. Can the survey drone fly without GPS?

Planned, not designed in detail. It uses GNSS when the fix is healthy. The fallback is ArduPilot EKF3 source switching to a downward camera (optical flow or visual odometry, not chosen) with LiDAR height. This is unproven from 30–50 m over rubble and smoke.

### 11. Is 62 s for a 6 × 6 m area slow?

It is about 35 m² a minute at a commanded speed of 0.70 m/s or less, with 1.5 m lanes (E1). That flight tested navigation, not coverage rate; speed and lane spacing are parameters. We have not compared it with the time a rescue team takes.

## Hardware

### 12. What does the scout weigh, and how long does it fly?

Not established. "Single-digit minutes per 6S pack" is an estimate. A hover test on the 5-inch test quad comes first (measurement 4).

### 13. Does the power budget close?

Not at 12 V. The Jetson's input allows 42 W at 12 V, or 32.3 W with our 30 % margin, and the module alone is allocated 25 W in its highest mode. A 19 V rail, a separate 5 V branch for peripherals, or capped loads are under review (`HARDWARE_AND_THERMAL.md`).

### 14. Will the computer overheat in a guarded frame?

Unknown. Nothing thermal has been measured. Timing YOLOv8n with TensorRT on the Orin Nano under sustained load, with temperatures, is measurement 6.

### 15. Depth cameras struggle in the dark. How will the scout avoid walls?

Obstacle avoidance is not built; flights so far are on open ground with preset waypoints. The design uses the depth output of an OAK-D Lite camera. Depth sensing in darkness and dust is untested, and the choice of depth sensor for dark voids is open.

### 16. The bench photo shows Betaflight, but the stack is ArduPilot.

Correct. The bench flight controller (HAKRC F405 V2) runs Betaflight; ArduPilot has no build for it. The bench script only reads its attitude and is not part of the flight stack. An ArduPilot flight controller has to be bought (`HARDWARE_AND_THERMAL.md`).

### 17. Why a Jetson on a Qualcomm problem statement?

The Jetson Orin Nano Super is the design choice for the scout; no drone computer has been timed. The stack is ROS 2 plus a standard exported model, so it is not tied to one board. Profiling the same detector on a Qualcomm QCS6490 through Qualcomm AI Hub is planned and has not been run (`ROADMAP.md`, measurement 10).

## Software and fail-safes

### 18. Why two languages?

Python runs the ROS 2 nodes and the detector; the guidance maths is pure functions under test. Rust is the telemetry library and a stand-alone peer on Zenoh's own Rust library, checked against the Python peers (`RUST_PYTHON_HEARTBEAT_CHECK.md`).

### 19. What happens if the guidance software or the computer dies?

A separate watchdog process commanded LAND 1.05 s after guidance was killed in simulated flight (E7). It shares the computer. If the computer loses power, ArduPilot's own timeout and battery fail-safe are the last layer; that is not measured yet (measurement 5).

### 20. What happens when the radio link drops?

The search continues: with the command post killed, the drone flew on 5.9 m in the next 8 s (E9). With no peer for 2.0 s it holds position. Alerts sent during an outage are lost today; storing them on the drone and re-sending is planned.

### 21. What proves there is no central server?

Two real laptops linked directly with no router or broker; a lost peer was detected in 1.78 s (E10). In simulation, three peers ran with the command post as one of them (E9).

## Impact, cost and use

### 22. Where do 74 %, 22 % and 6 % come from?

They are published survival figures by day of rescue, quoted in Chiu et al., *Sustainability* 12(19), 2020, who cite earlier earthquake studies. They are not our data, and we do not claim a time saving yet.

### 23. What does the full system cost?

The prototype parts estimate is ₹3.87–4.50 lakh (₹4.59 lakh worst case with GST) against our ₹5 lakh cap. It predates three survey-drone parts and lists two parts at ₹0 that are not owned. The bill of materials is being re-priced; until then the total is incomplete.

### 24. Who decides the rescue order and the route?

The commander. The system suggests an order and a route around flagged hazards, and never marks a route safe. The weights behind the order are not fixed; the design says they are to be set with responders (`MISSION_DESIGN.md` §5).

### 25. What is needed before a field trial?

The Drone Rules 2021 steps: a unique identification number on Digital Sky, a certified remote pilot and permission for autonomous flight.
