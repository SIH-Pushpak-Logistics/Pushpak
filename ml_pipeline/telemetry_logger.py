"""
=============================================================================
TELEMETRY DATA LOGGER (Data Ingestion Script)
=============================================================================

WHAT IS THIS SCRIPT FOR?
-----------------------------------------------------------------------------
While the publisher script acts like a drone broadcasting data, this script acts 
like a data scientist taking notes. Its only job is to constantly listen to the 
"Redis" memory bucket, grab all the raw data that the drone is throwing in there, 
and organize it neatly into a CSV spreadsheet.

This CSV spreadsheet is the final product that will be fed into our Machine Learning 
models (like our Battery Degradation Model).

HOW DOES IT WORK?
-----------------------------------------------------------------------------
1. Connection: It connects to the exact same "Redis" memory bucket as the publisher.
2. Setup: It creates (or opens) a file called 'drone_telemetry_dataset.csv' and 
   sets up neat columns (headers) like timestamp, linear_x, etc.
3. Batch Processing: If it tried to pull 200,000 records out of the bucket at once, 
   your computer might freeze. So instead, it pulls out safe "chunks" of 5,000 
   records at a time.
4. Normalization: It takes the raw, messy JSON data it pulled from Redis, extracts 
   exactly the numbers we care about, and writes them row-by-row into the spreadsheet.

HOW TO USE IT:
-----------------------------------------------------------------------------
Run this script while (or after) the mock publisher script is running:
`python telemetry_logger.py`
Press Ctrl+C to stop listening once it has finished processing the records.
"""

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
