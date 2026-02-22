import re
import subprocess
import threading
import time
import os
import sys
from dataclasses import dataclass
from typing import Optional

LASER_TOPIC = "/front_laser"
CMD_VEL_TOPIC = "/cmd_vel"

# Regex for laser data
RANGE_RE = re.compile(r"ranges:\s*\[?([0-9]+(?:\.[0-9]+)?)")

class AsyncPublisher:
    """Sends velocity commands without blocking the main loop."""
    def __init__(self):
        self._proc = None

    def publish(self, linear_x: float, angular_z: float) -> None:
        if self._proc is not None and self._proc.poll() is None:
            return
        msg = f"linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}"
        self._proc = subprocess.Popen(
            ["gz", "topic", "-t", CMD_VEL_TOPIC, "-m", "gz.msgs.Twist", "-p", msg],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

publisher = AsyncPublisher()

def publish_cmd(linear_x: float, angular_z: float) -> None:
    publisher.publish(linear_x, angular_z)

@dataclass
class LatestRange:
    value_m: Optional[float] = None
    updated_at: float = 0.0

class LaserStreamReader:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.latest = LatestRange()
        self._stop = False
        self._thread: Optional[threading.Thread] = None

        # 'script' forces gz topic to output line-by-line instead of buffering
        self._proc = subprocess.Popen(
            ["script", "-q", "/dev/null", "gz", "topic", "-e", "-t", self.topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop = True
        try:
            self._proc.terminate()
        except:
            pass

    def _run(self) -> None:
        if not self._proc.stdout:
            return
        for line in self._proc.stdout:
            if self._stop: break
            if "ranges:" in line:
                match = RANGE_RE.search(line)
                if match:
                    try:
                        self.latest.value_m = float(match.group(1))
                        self.latest.updated_at = time.monotonic()
                    except ValueError:
                        pass

def stop_robot() -> None:
    for _ in range(5):
        publish_cmd(0.0, 0.0)
        time.sleep(0.01)

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

def main() -> None:
    # 1. FIX: Reset terminal settings immediately
    os.system("stty sane")
    
    stop_distance_m = 0.10
    max_speed = 0.15
    slow_zone_m = 0.50
    stale_timeout_s = 0.5
    k = max_speed / max(1e-6, (slow_zone_m - stop_distance_m))

    reader = LaserStreamReader(LASER_TOPIC)
    reader.start()

    # 2. FIX: Use end='\r\n' and flush=True for all prints
    print("Wall-stop controller running.", end='\r\n', flush=True)

    last_print = 0.0
    
    try:
        while True:
            now = time.monotonic()
            dist = reader.latest.value_m
            age = now - reader.latest.updated_at

            if dist is None or age > stale_timeout_s:
                if now - last_print > 1.0:
                    print(f"Lidar stale (age: {age:.2f}s)", end='\r\n', flush=True)
                    last_print = now
                stop_robot()
                time.sleep(0.05)
                continue

            # Check stop condition
            if dist <= stop_distance_m + 0.002:
                print(f"Wall reached at {dist:.4f} m. STOP.", end='\r\n', flush=True)
                break

            # Calculate speed
            desired_speed = clamp(k * (dist - stop_distance_m), 0.0, max_speed)

            # Print status at 10Hz
            if now - last_print >= 0.1:
                print(f"Dist: {dist:.3f} m | Speed: {desired_speed:.3f}", end='\r\n', flush=True)
                last_print = now

            publish_cmd(desired_speed, 0.0)
            time.sleep(0.005)

    finally:
        stop_robot()
        reader.close()
        print("Done.", end='\r\n', flush=True)
        # Final reset to ensure your terminal is usable after exit
        os.system("stty sane")

if __name__ == "__main__":
    main()