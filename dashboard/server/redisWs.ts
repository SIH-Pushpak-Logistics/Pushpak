import { WebSocketServer, WebSocket } from 'ws';
import { createClient } from 'redis';

const PORT = 8765;
const REDIS_HOST = 'localhost';
const REDIS_PORT = 6379;
const DRONE_ID = 'drone_00';

const streams = [
    `telemetry:${DRONE_ID}:pose`,
    `telemetry:${DRONE_ID}:altitude`,
    `telemetry:${DRONE_ID}:velocity`,
    `telemetry:${DRONE_ID}:flow_debug`,
    `link:${DRONE_ID}:status`
];

const hashName = `victims:${DRONE_ID}`;

// Parse a redis string value to a number or boolean if applicable
function parseValue(val: string): any {
    if (val === 'True') return true;
    if (val === 'False') return false;
    const num = Number(val);
    if (!isNaN(num)) return num;
    return val;
}

async function startServer() {
    const wss = new WebSocketServer({ port: PORT });
    console.log(`WebSocket server started on port ${PORT}`);

    const redisSubscriber = createClient({ url: `redis://${REDIS_HOST}:${REDIS_PORT}` });
    const redisClient = createClient({ url: `redis://${REDIS_HOST}:${REDIS_PORT}` }); // For HGETALL

    redisSubscriber.on('error', err => console.error('Redis Subscriber Error', err));
    redisClient.on('error', err => console.error('Redis Client Error', err));

    await redisSubscriber.connect();
    await redisClient.connect();
    console.log('Connected to Redis');

    // Keep track of clients
    const clients = new Set<WebSocket>();

    wss.on('connection', (ws) => {
        clients.add(ws);
        console.log('Client connected');
        ws.on('close', () => {
            clients.delete(ws);
            console.log('Client disconnected');
        });
        
        // When client connects, immediately send current victims
        sendVictims();
    });

    const broadcast = (type: string, data: any) => {
        const message = JSON.stringify({ type, data });
        for (const client of clients) {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        }
    };

    // Send victims periodically or on change
    async function sendVictims() {
        try {
            const victimsData = await redisClient.hGetAll(hashName);
            const victimsList = Object.values(victimsData).map(v => JSON.parse(v));
            broadcast('victims', victimsList);
        } catch (e) {
            console.error('Error fetching victims', e);
        }
    }

    setInterval(sendVictims, 1000);

    let lastIds = Object.fromEntries(streams.map(s => [s, '$']));

    async function pollStreams() {
        try {
            const readArgs = streams.map(s => ({ key: s, id: lastIds[s] }));
            const results = await redisSubscriber.xRead(
                redisClient.commandOptions({ isolated: true }),
                readArgs,
                { BLOCK: 500 }
            );

            if (results) {
                for (const streamResult of results) {
                    const stream = streamResult.name;
                    for (const message of streamResult.messages) {
                        lastIds[stream] = message.id;
                        
                        const obj: Record<string, any> = {};
                        for (const key in message.message) {
                             obj[key] = parseValue(message.message[key]);
                        }
                        
                        const type = stream.split(':').pop(); 
                        broadcast(type!, obj);
                    }
                }
            }
        } catch (e) {
            console.error('Error reading stream', e);
        }
        
        setImmediate(pollStreams);
    }

    pollStreams();
}

startServer().catch(console.error);
