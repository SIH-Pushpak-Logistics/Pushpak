import time
import json
import random
import redis
import argparse
import math

def main():
    try:
        r = redis.Redis(host='localhost', port=6379, decode_responses=True, protocol=2)
        r.ping()
        print("Connected to Redis successfully.")
    except redis.ConnectionError:
        print("Error: Could not connect to Redis. Ensure your Redis server is running.")
        return

    parser = argparse.ArgumentParser(description="Generate fake drone telemetry data.")
    parser.add_argument('--records', type=int, default=200000, help='Total number of fake records to generate')
    args = parser.parse_args()

    # QUARANTINE: Hardcoded to the mock namespace
    stream_key = "mock_telemetry:drone_00:velocity"
    print(f"Publishing strict JSON telemetry to '{stream_key}'. Press Ctrl+C to stop.")

    try:
        TOTAL_RECORDS = args.records
        BATCH_SIZE = min(2500, TOTAL_RECORDS)
        
        print("Pre-calculating total capacity drawn...")
        temp_capacity = 0.0
        for idx in range(TOTAL_RECORDS):
            t = idx * 0.1
            p = idx / max(1, TOTAL_RECORDS - 1)
            
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

        capacity_drawn = 0.0
        print(f"Generating {TOTAL_RECORDS} records as fast as possible...")
        
        # LOGIC FIX: Anchor to current time, but maintain the 10Hz synthetic progression
        base_time = time.time()

        for i in range(0, TOTAL_RECORDS, BATCH_SIZE):
            current_batch_size = min(BATCH_SIZE, TOTAL_RECORDS - i)
            pipeline = r.pipeline()
            
            for j in range(current_batch_size):
                idx = i + j
                t = idx * 0.1
                p = idx / max(1, TOTAL_RECORDS - 1)
                
                linear_x = 2.5 * math.sin(0.05 * t) + 1.5 * math.sin(0.15 * t + 1.0) + 1.0 * math.sin(0.3 * t + 2.0)
                linear_y = 2.5 * math.sin(0.04 * t + 0.5) + 1.5 * math.sin(0.12 * t + 1.5) + 1.0 * math.sin(0.25 * t + 3.0)
                
                if p < 0.05:  
                    linear_z = 1.2 * math.sin(math.pi * (p / 0.05))
                elif p > 0.95:  
                    linear_z = -0.8 * math.sin(math.pi * ((1.0 - p) / 0.05))
                else:  
                    linear_z = 0.05 * math.sin(0.02 * t) + random.normalvariate(0.0, 0.015)
                
                angular_z = 0.02 * math.sin(0.01 * t) + random.normalvariate(0.0, 0.01)
                maneuver_interval = 1000.0
                time_since_maneuver = t % maneuver_interval
                center_of_maneuver = 500.0
                if abs(time_since_maneuver - center_of_maneuver) < 20.0:
                    direction = 1 if int(t / maneuver_interval) % 2 == 0 else -1
                    pulse = math.exp(-((time_since_maneuver - center_of_maneuver) ** 2) / (2 * (5.0 ** 2)))
                    angular_z += direction * 0.5 * pulse

                base_current = 15.0 + 2.0 * abs(linear_x) + 2.0 * abs(linear_y) + 3.0 * linear_z + 1.0 * abs(angular_z) + random.uniform(-1.0, 1.0)
                total_current_amps = max(5.0, min(30.0, base_current))
                
                capacity_drawn += total_current_amps * (0.1 / 3600.0)
                v_logistic = 21.0 + (4.2 / (1.0 + math.exp(k * (capacity_drawn - cliff_capacity))))
                
                voltage_sag = 0.01 * total_current_amps
                sensor_noise = random.normalvariate(0.0, 0.01)
                battery_voltage_v = max(20.0, min(26.0, v_logistic - voltage_sag + sensor_noise))
                
                base_pwm = 1100.0 + (total_current_amps - 5.0) * (800.0 / 25.0)
                m1 = base_pwm + 40.0 * linear_x + 40.0 * linear_y + 100.0 * angular_z + random.uniform(-15, 15)
                m2 = base_pwm + 40.0 * linear_x - 40.0 * linear_y - 100.0 * angular_z + random.uniform(-15, 15)
                m3 = base_pwm - 40.0 * linear_x - 40.0 * linear_y + 100.0 * angular_z + random.uniform(-15, 15)
                m4 = base_pwm - 40.0 * linear_x + 40.0 * linear_y - 100.0 * angular_z + random.uniform(-15, 15)
                
                wind_speed_x = 5.0 + random.uniform(-1.0, 1.0)
                ambient_temp_c = 25.0 + random.uniform(-1.5, 1.5)
                
                telemetry_payload = {
                    "timestamp": round(base_time + t, 3),
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
                pipeline.xadd(stream_key, telemetry_payload, id='*')
            
            pipeline.execute()
            print(f"Published {i + current_batch_size} / {TOTAL_RECORDS} records...")

        print(f"Successfully published {TOTAL_RECORDS} data points to Redis!")

    except KeyboardInterrupt:
        print("\nMock publisher gracefully stopped.")

if __name__ == "__main__":
    main()