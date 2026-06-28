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
        print(f"Generating {TOTAL_RECORDS} records as fast as possible...")
        
        # We loop over the total records in chunks
        for i in range(0, TOTAL_RECORDS, BATCH_SIZE):
            
            # If the final batch is smaller than BATCH_SIZE, only do the remainder
            current_batch_size = min(BATCH_SIZE, TOTAL_RECORDS - i)
            
            # Open a "pipeline" (a tube that lets us send lots of data at once instead of one-by-one)
            pipeline = r.pipeline()
            
            # Create fake readings...
            for j in range(current_batch_size):
                # This is the fake data packet (timestamp, drone name, and random speeds)
                telemetry_payload = {
                    "timestamp": time.time(),
                    "drone_id": "drone_00",
                    "linear_x": round(random.uniform(-5.0, 5.0), 3),
                    "linear_y": round(random.uniform(-5.0, 5.0), 3),
                    "linear_z": 0.0,
                    "angular_z": 0.0
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
