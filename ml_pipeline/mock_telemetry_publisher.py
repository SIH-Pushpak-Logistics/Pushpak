"""
=============================================================================
MOCK DRONE TELEMETRY PUBLISHER (Data Generation Script)
=============================================================================

WHAT IS THIS SCRIPT FOR?
-----------------------------------------------------------------------------
In a real-world scenario, a physical drone flies around and its internal sensors 
(like GPS and accelerometers) constantly broadcast its speed and location. 

Because we don't have a physical drone flying right now, this script acts as a 
"fake drone." Its only job is to generate massive amounts of fake, randomized 
flight data so that we have something to train our Machine Learning algorithms on.

HOW DOES IT WORK?
-----------------------------------------------------------------------------
1. Connection: It connects to a "Redis" server. Think of Redis as a super-fast 
   temporary memory bucket that sits between this script and the logger script.
2. The Data: It creates a fake data packet containing:
   - timestamp: The exact time the data was created.
   - drone_id: The name of the fake drone (drone_00).
   - linear_x / linear_y / linear_z: The forward/sideways/up-down speed.
   - angular_z: How fast the drone is rotating.
3. The Speed Trick (Pipelining): Instead of putting one piece of data into the 
   bucket at a time (which is slow), this script gathers 2,500 pieces of data 
   at a time, and dumps them all into Redis at once. It does this until it 
   reaches 200,000 records.

HOW TO USE IT:
-----------------------------------------------------------------------------
Make sure your Redis server is running first. Then simply run this script:
`python mock_telemetry_publisher.py`
It will instantly generate and send 200,000 rows of fake data to the broker.
"""

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
