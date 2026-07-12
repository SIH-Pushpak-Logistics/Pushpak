import redis
import json
import csv
import os

def main():
    try:
        r = redis.Redis(host='localhost', port=6379, decode_responses=True, protocol=2)
        r.ping()
        print("Connected to Redis Broker.")
    except redis.ConnectionError:
        print("Error: Could not connect to Redis.")
        return

    # QUARANTINE: Strict V1 Isolation
    streams = {"mock_telemetry:drone_00:velocity": "0"}
    
    csv_filename = "drone_telemetry_dataset.csv"
    file_exists = os.path.isfile(csv_filename)
    
    headers = [
        "timestamp", "drone_id", "linear_x", "linear_y", "linear_z", "angular_z",
        "wind_speed_x", "ambient_temp_c",
        "motor_1_pwm", "motor_2_pwm", "motor_3_pwm", "motor_4_pwm",
        "total_current_amps", "battery_voltage_v"
    ]

    print(f"Starting ML Data Pipeline Logger...")
    print(f"Logging normalized data to {csv_filename} at 10Hz. Press Ctrl+C to stop.")

    with open(csv_filename, mode='a', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=headers)
        
        if not file_exists:
            writer.writeheader()
        
        try:
            while True:
                response = r.xread(streams, count=5000, block=100)
                
                if response:
                    for stream_name, messages in response:
                        for message_id, message_data in messages:
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
                                
                                csv_file.flush()
                                streams[stream_name] = message_id
                                    
        except KeyboardInterrupt:
            print("\nTelemetry logger gracefully stopped. Dataset saved.")

if __name__ == "__main__":
    main()