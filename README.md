1. System Objective & Scope
The goal of this system is to achieve decentralized, GPS-independent drone navigation and fleet deconfliction within a Gazebo SITL environment.

Core Constraint: Nodes must be decoupled. Perception and localization do not have direct authority over the motors.

The Bridge: All state intentions are published to a localized Redis Broker.

The Authority: A Master State Machine evaluates network telemetry, applies safety heuristics, and publishes the final kinematic execution to MAVROS.


2. Team Responsibilities & Node Architecture

A. The Perception Layer (Owner: Raunak)
Node: vision_nav_node.py
Role: Calculate X/Y drift velocity using Lucas-Kanade optical flow.
Strict Constraints: * Must use message_filters.ApproximateTimeSynchronizer to time-sync the camera frames (/camera/image_raw) with the altimeter (/drone/altitude) to accurately perform the pinhole camera conversion ($V_x = \frac{u \cdot Z}{f_x}$).
Must utilize the queue.Queue(maxsize=5) background thread for Redis I/O.

NEW TASK/DIRECTIVE

Must extract timestamps from the ROS 2 headers to calculate the precise time delta ($\Delta t$) between frames. Velocity calculations must rigidly follow $V_x = \frac{u \cdot Z}{f_x \cdot \Delta t}$. Calculating raw displacement per frame is strictly forbidden.

Forbidden: Subscribing to or publishing to /mavros/setpoint_velocity/cmd_vel.

B. The Altitude & Landing Layer (Owner: Shivam)
Node: landing_state_node.py
Role: Calculate the Z-axis velocity required to hold altitude or execute a precision landing.
Strict Constraints:
Must read from the altimeter and output a proportional $V_z$ command.
Must clamp maximum ascent/descent speeds to prevent simulation physics instability.

NEW TASK/DIRECTIVE

Must implement an explicit max_ascent_speed (e.g., 1.5 m/s) to clamp the P-controller's upward velocity. Uncapped positive Z-axis commands will destabilize the simulation physics.

Forbidden: Publishing directly to MAVROS. Output must go strictly to the designated Redis stream.

C. The Machine Learning Data Pipeline (Owners: Bhavya & Priya)

Script: telemetry_logger.py
Role: Ingest simulation state data to build the dataset for the non-linear battery degradation routing model.
Strict Constraints:
Do not interface with ROS 2 DDS.
Subscribe directly to the Redis telemetry:*:velocity streams and log outputs to normalized CSVs at 10Hz.

D. The Simulation Environment (Owner: Aditya)

Platform: Blender to Gazebo Classic.
Strict Constraints:
Scale: 1 Blender Unit = 1 Real-World Meter.
Performance: Low-poly meshes only. All transforms must be applied (Origin at 0,0,0 geometric base).
Format: Export strictly as .dae (Collada) with high-contrast ground textures for optical flow validation.

E. The Master State Machine & Overwatch (Owner: Lead Architect)

Node: state_machine_node.py
Role: The absolute brain of the drone. Ingests all Redis streams, calculates staleness, handles decentralized swarm collision overrides, and executes motor commands.

NEW TASK/DIRECTIVE

Node: state_machine_node.py

Strict Constraints:

Asynchronous I/O: The 20Hz control loop must be entirely non-blocking. Redis polling (xrevrange) must be offloaded to a background threading.Thread that updates a local, thread-safe state dictionary.

The MAVROS Handshake: Must publish 0.0 m/s velocities for 2 seconds, explicitly call the /mavros/set_mode service to request OFFBOARD mode, and wait for confirmation before calling the arming service.

F. The Simulation Orchestration (The Spine) (Owner: Lead Architect)

   File: sitl_bringup.launch.py (inside drone_description/launch/)

Role: The master orchestrator that bridges the ROS 2 software stack with the Gazebo physics engine.

Strict Sequence:

Initialize robot_state_publisher to broadcast the URDF TF tree.

Launch the Gazebo server and client.

Execute spawn_entity.py to inject the drone into the simulation.

Trigger swarm_bringup.launch.py only after physics are confirmed running.

3. The Data Contract (Stream & Topic Registry)
This is the immutable data structure of the system. Any deviation in naming conventions or JSON payloads will result in immediate node failure.

ROS 2 Subscriptions (Input Layer)
| Topic Name | Type | Consumer | Purpose |
| :--- | :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/Image` | Perception | Raw visual feed. |
| `/drone/altitude` | `std_msgs/Float32` | Perception, Landing | Metric altitude (Z). |



Redis Broker Streams (The Decoupled Network)
1. X/Y Vision Telemetry
Stream Key: telemetry:<drone_id>:velocity        
JSON Schema:{
  "timestamp": "float (UNIX epoch)",
  "drone_id": "string",
  "linear_x": "float",
  "linear_y": "float",
  "linear_z": "0.0",
  "angular_z": "0.0"
}

2. Z-Axis Landing Telemetry

Stream Key: telemetry:<drone_id>:altitude_cmd
JSON Schema: Identical to above, utilized exclusively for linear_z instructions.

3. Global Swarm Override
Stream Key: emergency_override:<drone_id>
JSON Schema: Identical to above. Dictated by the off-board distributed engine.
ROS 2 Publishers (Execution Layer)

| Topic Name | Type | Producer | Purpose |
| :--- | :--- | :--- | :--- |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | State Machine | Final unarguable flight controller input. |

4. Failsafes & Execution Priorities
The state_machine_node.py operates at 20Hz and adheres to the following strict priority hierarchy.

Priority 1: Swarm Override. If a valid, non-stale vector exists in emergency_override:<drone_id>, execute it immediately and ignore all local perception.

Priority 2: Local Perception Integration. Pull the latest X/Y vector from telemetry:<drone_id>:velocity and the Z vector from telemetry:<drone_id>:altitude_cmd. Merge them into a single TwistStamped message.

Priority 3: The Staleness Failsafe. The state machine compares the payload timestamp against the system clock. If the delta exceeds 0.3 seconds (adjusted for 15Hz camera processing limits), the data is stale. The drone drops the command and defaults to a zero-velocity hover.
