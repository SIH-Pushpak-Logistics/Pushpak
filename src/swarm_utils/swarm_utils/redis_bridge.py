import redis
import queue
import threading
import time

class RedisTelemetryPublisher:
    def __init__(self, host='localhost', port=6379, stream_name='', logger=None):
        self.stream_name = stream_name
        self.logger = logger
        
        try:
            self.client = redis.Redis(host=host, port=port, decode_responses=True)
            self.client.ping()
            if self.logger:
                self.logger.info(f"Redis Bridge connected. Stream: {self.stream_name}")
        except redis.RedisError as e:
            if self.logger:
                self.logger.error(f"FATAL: Redis connection failed: {e}")
            raise SystemExit

        self._queue = queue.Queue(maxsize=5)
        self._worker = threading.Thread(target=self._publish_loop, daemon=True)
        self._worker.start()

    def _publish_loop(self):
        while True:
            try:
                payload = self._queue.get(block=True, timeout=1.0)
            except queue.Empty:
                continue

            try:
                self.client.xadd(self.stream_name, payload, maxlen=100, approximate=True)
            except redis.RedisError as e:
                if self.logger:
                    self.logger.error(f"Redis publish failed: {e}")
            finally:
                self._queue.task_done()

    def send_velocity_vector(self,
        drone_id,
        timestamp_sec,
        vx,
        vy,
        vz,
        wz,
        cut_motors=False,
        is_valid=True,
        features=0
    ):
        """
        Enforces the data contract. Coerces all floats to strict strings.
        """
        payload = {
            'timestamp': str(timestamp_sec),
            'drone_id': str(drone_id),
            'linear_x': f"{float(vx):.4f}",
            'linear_y': f"{float(vy):.4f}",
            'linear_z': f"{float(vz):.4f}",
            'angular_z': f"{float(wz):.4f}",
            'cut_motors': str(cut_motors),
            'is_valid': str(is_valid),
            'features': str(features)
        }
        
        # This belongs HERE. Not at the bottom of the file.
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            pass # Drop frame to maintain real-time edge


class RedisTelemetrySubscriber:
    def __init__(self, streams, host='localhost', port=6379, logger=None):
        self.streams = streams
        self.logger = logger
        
        try:
            self.client = redis.Redis(host=host, port=port, decode_responses=True)
            self.client.ping()
        except redis.RedisError as e:
            if self.logger:
                self.logger.error(f"FATAL: Redis connection failed: {e}")
            raise SystemExit

        # Thread-safe local storage for the 20Hz loop to read instantly
        self._lock = threading.Lock()
        self._latest_data = {stream: None for stream in streams}

        # Background worker that polls the indexed message queue at 100Hz
        self._worker = threading.Thread(target=self._poll_loop, daemon=True)
        self._worker.start()

    def _poll_loop(self):
        """ Runs infinitely in the background, hammering Redis for updates. """
        while True:
            try:
                for stream in self.streams:
                    # Pull only the absolute newest record from the stream
                    data = self.client.xrevrange(stream, max='+', min='-', count=1)
                    if data:
                        # ACQUIRE LOCK: Stop the state machine from reading while we write
                        with self._lock:
                            self._latest_data[stream] = data[0][1]
            except redis.RedisError as e:
                if self.logger:
                    self.logger.error(f"Redis read failed: {e}")
            
            # Prevent the daemon from maxing out the CPU core
            time.sleep(0.01)

    def get_latest(self, stream_name):
        """ Returns the latest payload in sub-millisecond time. No network I/O. """
        # ACQUIRE LOCK: Stop the background thread from overwriting while we read
        with self._lock:
            return self._latest_data.get(stream_name)