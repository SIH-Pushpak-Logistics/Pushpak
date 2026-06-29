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

    def send_velocity_vector(self, drone_id, timestamp_sec, vx, vy, vz, wz, cut_motors=False):
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
            'cut_motors': str(cut_motors)
        }
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            pass # Drop frame to maintain real-time edge