import re
import subprocess
import threading
import time
import os
import sys
import math
import random
from dataclasses import dataclass
from typing import Optional
from enum import Enum, auto

LASER_TOPIC = "/front_laser"
CMD_VEL_TOPIC = "/cmd_vel"

RANGE_RE = re.compile(r"ranges:\s*\[?(inf|infinity|nan|[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

class RobotState(Enum):
    CRUISING = auto()
    SCANNING_360 = auto()
    ALIGNING = auto()
    STOPPED = auto()

class AsyncPublisher:
    """Publishes continuously but guarantees force_stop cannot be dropped."""
    def __init__(self):
        self._proc = None
        self._lock = threading.Lock()

    def publish(self, linear_x: float, angular_z: float) -> None:
        with self._lock:
            # If previous command is still being sent, don't overlap (prevents CPU lag)
            if self._proc is not None and self._proc.poll() is None:
                return
            
            msg = f"linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}"
            self._proc = subprocess.Popen(
                ["gz", "topic", "-t", CMD_VEL_TOPIC, "-m", "gz.msgs.Twist", "-p", msg],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

    def force_stop(self) -> None:
        with self._lock:
            # 1. Brutally kill any forward driving command currently processing
            if self._proc is not None and self._proc.poll() is None:
                try:
                    self._proc.terminate()
                except:
                    pass
            self._proc = None

        # 2. Synchronously blast stop commands so Gazebo is forced to listen BEFORE we move on
        msg = "linear: {x: 0.0}, angular: {z: 0.0}"
        for _ in range(3):
            subprocess.run(
                ["gz", "topic", "-t", CMD_VEL_TOPIC, "-m", "gz.msgs.Twist", "-p", msg],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

@dataclass
class LatestRange:
    value_m: Optional[float] = None
    updated_at: float = 0.0

class LaserReader:
    def __init__(self, topic: str):
        self.topic = topic
        self.latest = LatestRange()
        self._stop_event = threading.Event()
        self._proc = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        self._stop_event.set()
        # Immediately kill the bash process so Ctrl+C closes instantly
        if self._proc is not None:
            try:
                self._proc.terminate()
            except:
                pass

    def _run(self):
        cmd = ["script", "-q", "/dev/null", "gz", "topic", "-e", "-t", self.topic]
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1
        )
        try:
            for line in self._proc.stdout:
                if self._stop_event.is_set():
                    break
                if "ranges:" in line:
                    match = RANGE_RE.search(line)
                    if match:
                        try:
                            val_str = match.group(1).lower()
                            if 'inf' in val_str or 'nan' in val_str:
                                self.latest.value_m = 10.0
                            else:
                                self.latest.value_m = float(val_str)
                            self.latest.updated_at = time.monotonic()
                        except ValueError:
                            pass
        except Exception:
            pass
        finally:
            if self._proc:
                try:
                    self._proc.terminate()
                except:
                    pass

class ScoutRoomba:
    # --- Tunable Parameters ---
           
    SCAN_ANGULAR_VEL = 1    
    SCAN_DURATION = (2 * math.pi) / SCAN_ANGULAR_VEL  
    
    SPEED_CRUISE = 0.3      # You can safely increase this now   
    SPEED_ALIGN = 1         
    STALE_TIMEOUT = 1.0     # Increased to ignore brief 0.000m simulator lag spikes

    STOP_DIST = SPEED_CRUISE * 1.25

    def __init__(self):
        self.pub = AsyncPublisher()
        self.laser = LaserReader(LASER_TOPIC)
        self.state = RobotState.STOPPED
        self.state_timer = 0.0
        self.last_print_time = 0.0

        self.scan_readings: list[tuple[float, float]] = []
        self.scan_direction = 1
        self.align_direction = 1
        self.align_duration = 0.0

    def get_dist(self) -> float:
        now = time.monotonic()
        if (self.laser.latest.value_m is None or
                now - self.laser.latest.updated_at > self.STALE_TIMEOUT):
            return 0.0
        return self.laser.latest.value_m
        
    def log(self, text: str):
        sys.stdout.write(f"{text}\r\n")
        sys.stdout.flush()

    def set_state(self, new_state: RobotState) -> None:
        if self.state == new_state:
            return
        self.log(f"\r\n--- State Change: {self.state.name} -> {new_state.name} ---")
        self.state = new_state
        self.state_timer = time.monotonic()

        if new_state == RobotState.SCANNING_360:
            self.scan_readings = []
            self.scan_direction = random.choice([-1, 1])

    def analyze_scan(self) -> tuple[int, float]:
        if not self.scan_readings:
            self.log("No scan data! Picking random direction.")
            return random.choice([-1, 1]), 1.0

        smoothed = []
        window = 5
        for i in range(len(self.scan_readings)):
            start = max(0, i - window // 2)
            end = min(len(self.scan_readings), i + window // 2 + 1)
            avg_dist = sum(d for _, d in self.scan_readings[start:end]) / (end - start)
            smoothed.append((self.scan_readings[i][0], avg_dist))

        best_angle, best_dist = max(smoothed, key=lambda x: x[1])

        self.log(f"\r\nANGLE WITH FARTHEST DISTANCE: {math.degrees(best_angle):.1f}, {best_dist:.3f}\r\n")

        forward_angle = best_angle
        backward_angle = 2 * math.pi - best_angle

        if forward_angle <= backward_angle:
            direction = self.scan_direction
            duration = forward_angle / self.SPEED_ALIGN
        else:
            direction = -self.scan_direction
            duration = backward_angle / self.SPEED_ALIGN

        return direction, duration

    def run(self) -> None:
        os.system("stty sane")
        self.laser.start()
        time.sleep(1.0)

        self.set_state(RobotState.CRUISING)

        try:
            while True:
                now = time.monotonic()
                dist = self.get_dist()

                # --- 1. STOP LOGIC ---
                if 0.01 < dist < self.STOP_DIST and self.state == RobotState.CRUISING:
                    self.pub.force_stop()
                    # Physical brake time (stops the car completely before spinning)
                    time.sleep(0.3)
                    self.set_state(RobotState.SCANNING_360)

                # --- 2. PRINTING LOGIC ---
                if now - self.last_print_time >= 0.2:
                    if self.state == RobotState.SCANNING_360:
                        elapsed = now - self.state_timer
                        current_angle = elapsed * self.SCAN_ANGULAR_VEL
                        deg = math.degrees(current_angle)
                        self.log(f"Angle: {deg:5.1f}°, Distance: {dist:.3f}m")
                    elif self.state == RobotState.CRUISING:
                        self.log(f"Distance: {dist:.3f}m")
                    
                    self.last_print_time = now

                # --- 3. STATE MACHINE ---
                if self.state == RobotState.CRUISING:
                    # Braking Zone: Slow down if we are 30cm away from the STOP limit
                    if 0.01 < dist <= (self.STOP_DIST + 0.30):
                        self.pub.publish(self.SPEED_CRUISE / 4.0, 0.0)
                        print("VERY CLOSE")
                    elif 0.01 < dist <= (self.STOP_DIST + 0.60):
                        self.pub.publish(self.SPEED_CRUISE / 2.0, 0.0)
                        print("CLOSE")

                    else:
                        self.pub.publish(self.SPEED_CRUISE, 0.0)

                elif self.state == RobotState.SCANNING_360:
                    elapsed = now - self.state_timer
                    
                    if dist > 0.01:
                        angle = elapsed * self.SCAN_ANGULAR_VEL
                        self.scan_readings.append((angle, dist))

                    if elapsed >= self.SCAN_DURATION:
                        self.pub.force_stop()
                        self.align_direction, self.align_duration = self.analyze_scan()
                        self.set_state(RobotState.ALIGNING)
                    else:
                        self.pub.publish(0.0, self.scan_direction * self.SCAN_ANGULAR_VEL)

                elif self.state == RobotState.ALIGNING:
                    elapsed = now - self.state_timer

                    if elapsed >= self.align_duration:
                        self.pub.force_stop()
                        time.sleep(0.15)
                        self.set_state(RobotState.CRUISING)
                    else:
                        self.pub.publish(0.0, self.align_direction * self.SPEED_ALIGN)

                time.sleep(0.02)

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self.pub.force_stop()
        self.laser.close()
        time.sleep(0.2)
        os.system("stty sane")
        self.log("\r\nShutdown complete.\r\n")

if __name__ == "__main__":
    ScoutRoomba().run()
