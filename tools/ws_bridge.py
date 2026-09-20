#!/usr/bin/env python3
import asyncio
import json
import redis.asyncio as aioredis
import websockets

PORT = 8765
DRONE_ID = 'drone_00'

STREAMS = [
    f'telemetry:{DRONE_ID}:pose',
    f'telemetry:{DRONE_ID}:altitude',
    f'telemetry:{DRONE_ID}:velocity',
    f'telemetry:{DRONE_ID}:flow_debug',
    f'link:{DRONE_ID}:status'
]
VICTIMS_KEY = f'victims:{DRONE_ID}'

clients = set()
r_client = None

def parse_val(v):
    if v == 'True': return True
    if v == 'False': return False
    try:
        return float(v) if ('.' in v or 'e' in v.lower()) else int(v)
    except (ValueError, TypeError):
        return v

async def broadcast(msg_type, data):
    if not clients:
        return
    payload = json.dumps({'type': msg_type, 'data': data})
    dead = set()
    for client in list(clients):
        try:
            await client.send(payload)
        except Exception:
            dead.add(client)
    clients.difference_update(dead)

async def victim_poller():
    while True:
        try:
            if clients:
                raw_victims = await r_client.hgetall(VICTIMS_KEY)
                if raw_victims:
                    victims = [json.loads(v) for v in raw_victims.values()]
                    await broadcast('victims', victims)
        except Exception:
            pass
        await asyncio.sleep(1.0)

async def stream_poller():
    last_ids = {s: '$' for s in STREAMS}
    while True:
        try:
            res = await r_client.xread(last_ids, block=300, count=10)
            if res:
                for stream_name, messages in res:
                    for msg_id, fields in messages:
                        last_ids[stream_name] = msg_id
                        parsed = {k: parse_val(v) for k, v in fields.items()}
                        msg_type = stream_name.split(':')[-1]
                        await broadcast(msg_type, parsed)
        except Exception:
            await asyncio.sleep(0.05)

async def handler(websocket):
    clients.add(websocket)
    print(f"[WS] Client connected. Total: {len(clients)}", flush=True)
    try:
        # Immediate victim state push on connect
        raw_victims = await r_client.hgetall(VICTIMS_KEY)
        if raw_victims:
            victims = [json.loads(v) for v in raw_victims.values()]
            await websocket.send(json.dumps({'type': 'victims', 'data': victims}))
        
        # Keep connection alive
        async for _ in websocket:
            pass
    except Exception:
        pass
    finally:
        clients.discard(websocket)
        print(f"[WS] Client disconnected. Total: {len(clients)}", flush=True)

async def main():
    global r_client
    r_client = aioredis.Redis(host='localhost', port=6379, decode_responses=True)
    await r_client.ping()
    print("[WS] Connected to Redis on 6379", flush=True)

    asyncio.create_task(victim_poller())
    asyncio.create_task(stream_poller())

    async with websockets.serve(handler, "0.0.0.0", PORT):
        print(f"[WS] Bridge active on ws://0.0.0.0:{PORT}", flush=True)
        await asyncio.get_running_loop().create_future()

if __name__ == '__main__':
    asyncio.run(main())
