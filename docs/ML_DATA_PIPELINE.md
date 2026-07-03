# ML Data Pipeline: Setup & Recreation Guide (Beginner Friendly)

This document provides a step-by-step guide to recreating the decoupled Redis-based Machine Learning Data Pipeline from scratch. Even if you have no prior experience with Redis or ROS, you can follow these instructions to generate high-frequency drone telemetry data.

---

## Prerequisites

Before starting, ensure you have:
1. **Python 3** installed on your system.
2. The `redis` Python library installed. Open your terminal and run:
   ```powershell
   pip install redis
   ```

---

## Step 1: Install Local Redis Server (Windows)

To run Redis natively on Windows without needing complex Linux subsystems (WSL):

1. **Download Redis:** Go to the [Microsoft Archive for Redis on GitHub](https://github.com/microsoftarchive/redis/releases) and download the `.zip` release (e.g. `Redis-x64-3.0.504.zip`). 
2. **Extract:** Extract the downloaded `.zip` file into a folder named `redis_win` directly inside the root of your project folder.
3. **Start the Server:** Open a new terminal, navigate to that folder, and start the server:
   ```powershell
   cd redis_win
   ./redis-server.exe
   ```
   **Important:** Leave this terminal window open and running in the background! It acts as the "broker" that passes data between scripts.

---

## Step 2: Set Up the Pipeline Directory

To keep your main project clean, create a dedicated folder for your Machine Learning scripts. Open a *new* terminal window (since the first one is running Redis) and run:

```powershell
# Make sure you are in the root of your Pushpak repository
mkdir ml_pipeline
cd ml_pipeline
```

---

## Step 3: Create the Mock Publisher

This script simulates the drone, rapidly generating 200,000 rows of fake sensor data (velocity, coordinates, etc.).

Inside the `ml_pipeline/` folder, create a new file named `mock_telemetry_publisher.py` and paste the following code:

```python
import time
import json
import random
import redis
import argparse
import math

def main():
    try:
        # Step 1: Connect to the Redis "Memory Bucket" running on your computer
        r = redis.Redis(host='localhost', port=6379, decode_responses=True, protocol=2)
        r.ping() # Check if it's actually awake and listening
        print("Connected to Redis successfully.")
    except redis.ConnectionError:
        print("Error: Could not connect to Redis. Ensure your Redis server is running.")
        return

    # Parse command line arguments to allow setting custom record counts
    parser = argparse.ArgumentParser(description="Generate fake drone telemetry data.")
    parser.add_argument('--records', type=int, default=200000, help='Total number of fake records to generate')
    args = parser.parse_args()

    stream_key = "telemetry:drone_00:velocity"
    print(f"Publishing strict JSON telemetry to '{stream_key}'. Press Ctrl+C to stop.")

    try:
        TOTAL_RECORDS = args.records  # How many fake speed readings we want to create
        BATCH_SIZE = min(2500, TOTAL_RECORDS)       # How many we send to the bucket at the same time (for speed)
        
        # Pre-calculate the total expected capacity drawn to dynamically tune the logistic curve
        print("Pre-calculating total capacity drawn...")
        temp_capacity = 0.0
        for idx in range(TOTAL_RECORDS):
            t = idx * 0.1
            p = idx / max(1, TOTAL_RECORDS - 1)
            
            # Replicate velocity math (noise components omitted for deterministic expectation)
            linear_x = 2.5 * math.sin(0.05 * t) + 1.5 * math.sin(0.15 * t + 1.0) + 1.0 * math.sin(0.3 * t + 2.0)
            linear_y = 2.5 * math.sin(0.04 * t + 0.5) + 1.5 * math.sin(0.12 * t + 1.5) + 1.0 * math.sin(0.25 * t + 3.0)
            
            if p < 0.05:
                linear_z = 1.2 * math.sin(math.pi * (p / 0.05))
            elif p > 0.95:
                linear_z = -0.8 * math.sin(math.pi * ((1.0 - p) / 0.05))
            else:
                linear_z = 0.05 * math.sin(0.02 * t)
                
            angular_z = 0.02 * math.sin(0.01 * t)
            maneuver_interval = 1000.0
            time_since_maneuver = t % maneuver_interval
            center_of_maneuver = 500.0
            if abs(time_since_maneuver - center_of_maneuver) < 20.0:
                direction = 1 if int(t / maneuver_interval) % 2 == 0 else -1
                pulse = math.exp(-((time_since_maneuver - center_of_maneuver) ** 2) / (2 * (5.0 ** 2)))
                angular_z += direction * 0.5 * pulse

            base_current = 15.0 + 2.0 * abs(linear_x) + 2.0 * abs(linear_y) + 3.0 * linear_z + 1.0 * abs(angular_z)
            total_current_amps = max(5.0, min(30.0, base_current))
            temp_capacity += total_current_amps * (0.1 / 3600.0)

        final_expected_capacity = max(0.001, temp_capacity)
        cliff_capacity = 0.5 * final_expected_capacity
        k = 12.0 / final_expected_capacity
        print(f"Pre-calculation complete. Expected capacity: {final_expected_capacity:.4f} Ah, k={k:.4f}, cliff={cliff_capacity:.4f} Ah")

        # Initialize the actual capacity state tracker
        capacity_drawn = 0.0
        print(f"Generating {TOTAL_RECORDS} records as fast as possible...")
        
        # We loop over the total records in chunks
        for i in range(0, TOTAL_RECORDS, BATCH_SIZE):
            
            # If the final batch is smaller than BATCH_SIZE, only do the remainder
            current_batch_size = min(BATCH_SIZE, TOTAL_RECORDS - i)
            
            # Open a "pipeline" (a tube that lets us send lots of data at once instead of one-by-one)
            pipeline = r.pipeline()
            
            # Create fake readings...
            for j in range(current_batch_size):
                idx = i + j
                t = idx * 0.1
                p = idx / max(1, TOTAL_RECORDS - 1)
                
                # Overlapping sine waves for smooth, physically realistic velocity drift
                linear_x = 2.5 * math.sin(0.05 * t) + 1.5 * math.sin(0.15 * t + 1.0) + 1.0 * math.sin(0.3 * t + 2.0)
                linear_y = 2.5 * math.sin(0.04 * t + 0.5) + 1.5 * math.sin(0.12 * t + 1.5) + 1.0 * math.sin(0.25 * t + 3.0)
                
                # Continuous vertical velocity (linear_z) flight profile (takeoff, hover, landing)
                if p < 0.05:  # Takeoff climb
                    linear_z = 1.2 * math.sin(math.pi * (p / 0.05))
                elif p > 0.95:  # Landing descent
                    linear_z = -0.8 * math.sin(math.pi * ((1.0 - p) / 0.05))
                else:  # Cruising hover
                    linear_z = 0.05 * math.sin(0.02 * t) + random.normalvariate(0.0, 0.015)
                
                # Continuous yaw rate (angular_z) turning profile (background noise + sparse turns)
                angular_z = 0.02 * math.sin(0.01 * t) + random.normalvariate(0.0, 0.01)
                maneuver_interval = 1000.0
                time_since_maneuver = t % maneuver_interval
                center_of_maneuver = 500.0
                if abs(time_since_maneuver - center_of_maneuver) < 20.0:
                    direction = 1 if int(t / maneuver_interval) % 2 == 0 else -1
                    pulse = math.exp(-((time_since_maneuver - center_of_maneuver) ** 2) / (2 * (5.0 ** 2)))
                    angular_z += direction * 0.5 * pulse

                # Total current draw correlated with flight velocity/effort (pitch, roll, vertical, yaw)
                base_current = 15.0 + 2.0 * abs(linear_x) + 2.0 * abs(linear_y) + 3.0 * linear_z + 1.0 * abs(angular_z) + random.uniform(-1.0, 1.0)
                total_current_amps = max(5.0, min(30.0, base_current))
                
                # Stateful Euler integration for capacity drawn (Ah)
                capacity_drawn += total_current_amps * (0.1 / 3600.0)
                
                # Battery voltage base curve (Logistic decay model)
                v_logistic = 21.0 + (4.2 / (1.0 + math.exp(k * (capacity_drawn - cliff_capacity))))
                
                # Battery voltage under load (base voltage - sag + sensor noise)
                voltage_sag = 0.01 * total_current_amps
                sensor_noise = random.normalvariate(0.0, 0.01)
                battery_voltage_v = max(20.0, min(26.0, v_logistic - voltage_sag + sensor_noise))
                
                # Base hover throttle + adjustments for motor PWM outputs (Quad-X mixing)
                base_pwm = 1100.0 + (total_current_amps - 5.0) * (800.0 / 25.0)
                m1 = base_pwm + 40.0 * linear_x + 40.0 * linear_y + 100.0 * angular_z + random.uniform(-15, 15)
                m2 = base_pwm + 40.0 * linear_x - 40.0 * linear_y - 100.0 * angular_z + random.uniform(-15, 15)
                m3 = base_pwm - 40.0 * linear_x - 40.0 * linear_y + 100.0 * angular_z + random.uniform(-15, 15)
                m4 = base_pwm - 40.0 * linear_x + 40.0 * linear_y - 100.0 * angular_z + random.uniform(-15, 15)
                
                # Environment variables
                wind_speed_x = 5.0 + random.uniform(-1.0, 1.0)
                ambient_temp_c = 25.0 + random.uniform(-1.5, 1.5)
                
                # Create the physical data packet (Strict 1D API Schema)
                telemetry_payload = {
                    "timestamp": round(t, 1),
                    "drone_id": "drone_00",
                    "linear_x": round(linear_x, 3),
                    "linear_y": round(linear_y, 3),
                    "linear_z": round(linear_z, 3),
                    "angular_z": round(angular_z, 3),
                    "wind_speed_x": round(wind_speed_x, 2),
                    "ambient_temp_c": round(ambient_temp_c, 2),
                    "motor_1_pwm": int(max(1000, min(2000, m1))),
                    "motor_2_pwm": int(max(1000, min(2000, m2))),
                    "motor_3_pwm": int(max(1000, min(2000, m3))),
                    "motor_4_pwm": int(max(1000, min(2000, m4))),
                    "total_current_amps": round(total_current_amps, 2),
                    "battery_voltage_v": round(battery_voltage_v, 3)
                }
                # Queue the packet up directly in the pipeline tube
                pipeline.xadd(stream_key, telemetry_payload, id='*')
            
            # WHOOSH! Push all queued packets through the tube into Redis instantly
            pipeline.execute()
            print(f"Published {i + current_batch_size} / {TOTAL_RECORDS} records...")

        print(f"Successfully published {TOTAL_RECORDS} data points to Redis!")

    except KeyboardInterrupt:
        print("\nMock publisher gracefully stopped.")

if __name__ == "__main__":
    main()
```

---

## Step 4: Create the Telemetry Logger

This script acts as the receiver. It catches the data from Redis and safely writes it to a `.csv` spreadsheet so your ML models can use it.

Inside the `ml_pipeline/` folder, create a new file named `telemetry_logger.py` and paste the following code:

import redis
import json
import csv
import os

def main():
    try:
        # Step 1: Connect to the SAME Redis "Memory Bucket" the drone is talking to
        r = redis.Redis(host='localhost', port=6379, decode_responses=True, protocol=2)
        r.ping() # Knock on the door to see if it's there
        print("Connected to Redis Broker.")
    except redis.ConnectionError:
        print("Error: Could not connect to Redis.")
        return

    # We want to read from the 'velocity' stream, starting from the very beginning (offset "0")
    streams = {"telemetry:drone_00:velocity": "0"} 
    
    # This is the Excel-style file we are going to create
    csv_filename = "drone_telemetry_dataset.csv"
    file_exists = os.path.isfile(csv_filename)
    
    # These are the column names at the top of our spreadsheet
    headers = [
        "timestamp", "drone_id", "linear_x", "linear_y", "linear_z", "angular_z",
        "wind_speed_x", "ambient_temp_c",
        "motor_1_pwm", "motor_2_pwm", "motor_3_pwm", "motor_4_pwm",
        "total_current_amps", "battery_voltage_v"
    ]

    print(f"Starting ML Data Pipeline Logger...")
    print(f"Logging normalized data to {csv_filename} at 10Hz. Press Ctrl+C to stop.")

    # Open our CSV file so we can start writing rows into it
    with open(csv_filename, mode='a', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=headers)
        
        # If the file is brand new, write the column names at the top
        if not file_exists:
            writer.writeheader()
        
        try:
            while True:
                # Step 2: Ask Redis to give us 5,000 records at a time (so we don't crash the computer)
                # If there's no data, wait up to 100 milliseconds for it to arrive
                response = r.xread(streams, count=5000, block=100)
                
                if response:
                    for stream_name, messages in response:
                        for message_id, message_data in messages:
                            # Step 3: We got a record! Pull out the flat dictionary directly from Redis
                            if message_data:
                                writer.writerow({
                                    "timestamp": message_data.get("timestamp"),
                                    "drone_id": message_data.get("drone_id"),
                                    "linear_x": message_data.get("linear_x"),
                                    "linear_y": message_data.get("linear_y"),
                                    "linear_z": message_data.get("linear_z"),
                                    "angular_z": message_data.get("angular_z"),
                                    "wind_speed_x": message_data.get("wind_speed_x"),
                                    "ambient_temp_c": message_data.get("ambient_temp_c"),
                                    "motor_1_pwm": message_data.get("motor_1_pwm"),
                                    "motor_2_pwm": message_data.get("motor_2_pwm"),
                                    "motor_3_pwm": message_data.get("motor_3_pwm"),
                                    "motor_4_pwm": message_data.get("motor_4_pwm"),
                                    "total_current_amps": message_data.get("total_current_amps"),
                                    "battery_voltage_v": message_data.get("battery_voltage_v")
                                })
                                
                                # Force the computer to save it to the hard drive immediately
                                csv_file.flush()
                                
                                # Remember our place in line so we don't read the same record twice!
                                streams[stream_name] = message_id
                                    
        except KeyboardInterrupt:
            # If the user presses Ctrl+C, stop safely without breaking anything
            print("\nTelemetry logger gracefully stopped. Dataset saved.")

if __name__ == "__main__":
    main()
```

---

## Step 5: Ignore Temporary Files in Git

If you are using GitHub, you do not want to upload the heavy Redis `.exe` application. 

Open your `.gitignore` file (or create one in the root folder) and add these lines:

```gitignore
# Redis Server & Database files (Temporary/Local)
redis.zip
redis_win/
dump.rdb

# Python Caches
__pycache__/
*.pyc
```

---

## Step 6: How to Run the Pipeline!

Whenever you want to generate a fresh dataset, follow these exact steps:

1. **Start Redis:** Make sure your Redis server is running in its own terminal window (`./redis-server.exe`).
2. **Generate Data:** Open a *new* terminal window, go into the folder, and run the publisher:
   ```powershell
   cd ml_pipeline
   
   # Run with the default 200,000 records:
   python mock_telemetry_publisher.py
   
   # Or run with a custom amount of records using the --records argument:
   python mock_telemetry_publisher.py --records 500
   ```
3. **Save Data:** In the same terminal, run the logger to compile the CSV spreadsheet:
   ```powershell
   python telemetry_logger.py
   ```
   *(You will now see a `drone_telemetry_dataset.csv` file appear in your folder containing all the data!)*

---

## Step 7: How to Flush and Stop the Redis Server

To clean up your Redis instance or shut down the server when you are done generating data:

### 1. Flush (Clear) the Redis Database
If you want to clear old telemetry stream data and start fresh, run:
```powershell
cd redis_win
./redis-cli.exe FLUSHALL
```

### 2. Stop the Redis Server
To stop the Redis server, you can either:
* **Interactive Method:** Go to the terminal window where `./redis-server.exe` is running and press `Ctrl + C`.
* **Command Line Method:** Run the shutdown command using the CLI:
  ```powershell
  cd redis_win
  ./redis-cli.exe shutdown
  ```

