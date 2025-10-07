#!/usr/bin/env python3
"""
Cute Robot Motion & Face Display

This script controls a robot's movement using ROS 2 and displays an animated
"cute" face in a fullscreen pygame window. It blends robot motion phases with
facial expressions, including blinking, reacting to hazards, and different
"story" behaviours.
"""

import os
import sys
import time
import random
import threading

import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector, HazardDetection

SECOND = 1.0  # seconds (convenience multiplier)

# ────────────────────────────────────────────────────────────────────────────────
# Helpers
# ────────────────────────────────────────────────────────────────────────────────
class PhaseController:
    """
    Handles sequential motion "phases" (e.g., forward, turn, stop)
    Each phase has a name and a duration.
    The controller advances through phases in a loop.
    """
    def __init__(self, phases):
        self.phases = phases      # list of (phase_name, duration)
        self.idx = 0              # current phase index
        self.started_at = None    # time phase started
        self.finished = False

    def start(self, now: float):
        """Start from first phase at given timestamp."""
        self.idx = 0
        self.started_at = now
        self.finished = False

    def current_phase(self):
        """Return the name of the current phase."""
        return self.phases[self.idx][0]

    def update(self, now: float):
        """Advance to next phase if current phase duration has elapsed."""
        if self.started_at is None:
            self.started_at = now
            return
        _, dur = self.phases[self.idx]
        if dur != float("inf") and (now - self.started_at) >= dur:
            if self.idx == len(self.phases) - 1:
                self.finished = True
            # Move to next phase, loop around
            self.idx = (self.idx + 1) % len(self.phases)
            self.started_at = now

    def skip(self, now: float):
        """Skip to next phase immediately."""
        self.idx = (self.idx + 1) % len(self.phases)
        self.started_at = now


def _phases(seq):
    """
    Convert sequence of (name, duration_in_seconds) into phase list.
    Multiplies finite durations by SECOND constant for consistency.
    """
    out = []
    for name, dur in seq:
        out.append((name, dur if dur == float("inf") else dur * SECOND))
    return out


class BlinkManager:
    """
    Randomly triggers a "blink" animation after a gap between blinks.
    """
    def __init__(self, min_gap=3, max_gap=6, duration=0.15):
        self.min_gap = min_gap
        self.max_gap = max_gap
        self.duration = duration
        self.next_blink = time.time() + random.uniform(min_gap, max_gap)
        self.active_since = None  # None = not blinking

    def trigger(self):
        """Manually trigger a blink."""
        self.active_since = time.time()
        self.next_blink = time.time() + random.uniform(self.min_gap, self.max_gap)

    def update(self):
        """Check if blink should start/stop based on timers."""
        now = time.time()
        # Start blink if enough time has passed
        if self.active_since is None and now >= self.next_blink:
            self.active_since = now
            self.next_blink = now + random.uniform(self.min_gap, self.max_gap)
        # End blink if blink duration exceeded
        if self.active_since and (now - self.active_since) > self.duration:
            self.active_since = None

    def blinking(self):
        """Return True if currently blinking."""
        return self.active_since is not None


# ────────────────────────────────────────────────────────────────────────────────
# Behaviour config (states → phases; phase → speeds)
# ────────────────────────────────────────────────────────────────────────────────
STATES = ["STRAIGHT", "DIZZY", "YAWN", "SINEWAVE"]

# Each state has a "story" → sequence of motion phases
STORY = {
    "STRAIGHT": _phases([("forward", 5.0), ("turn_180_right", 1.0), ("forward", 6.0)]),
    "DIZZY":    _phases([("spin_fast_right", 2.0), ("spin_fast_right_dizzy", 3.0), ("dizzy", 2.0)]),
    "YAWN":     _phases([("stop", 1.0), ("yawn", 3.0), ("stop", 1.0)]),
    "SINEWAVE": _phases([
        ("forward",1.0),("right",1.0),("forward",1.0), ("left",1.0),
        ("forward",1.0),("right",1.0),("forward",1.0),("left",1.0),
        ("forward",1.0),("right",1.0),("forward",1.0), ("left",1.0),
    ]),
    "AVOID":    _phases([("avoid", float("inf")), ("stop", 0.1)]),
}

# Mapping: phase_name → (linear_speed_m/s, angular_speed_rad/s)
SPEED = {
    "forward": (0.30, 0.00),
    "reverse": (-0.10, 0.00),
    "stop": (0.00, 0.00),
    "yawn": (0.00, 0.00),
    "dizzy": (0.00, 0.00),
    
    "spin_fast_right": (0.50, 20.0),
    "spin_fast_right_dizzy": (0.50, 20.0),
    "turn_180_right":  (0.02, 3.14159),
    "turn_180_left":  (0.02, -3.14159),
    "left": (0.10, -1.0),
    "right": (0.10, 1.0),
    "avoid": (-0.10, 1.0),

}


# ────────────────────────────────────────────────────────────────────────────────
# ROS 2 Motion Node — handles robot motion & hazard reactions
# ────────────────────────────────────────────────────────────────────────────────
class MotionNode(Node):
    def __init__(self):
        super().__init__("cute_motion_node")

        # Publisher: robot velocity commands
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriptions: hazards & IR readings (best-effort QoS)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(HazardDetectionVector, "/hazard_detection", self.on_hazard, qos)
        self.create_subscription(IrIntensityVector, "/ir_intensity", self.on_ir, qos)

        # 10Hz update loop for motion
        self.timer = self.create_timer(0.1, self.tick)

        # Controllers: one per state
        self.controllers = {name: PhaseController(STORY[name]) for name in STORY}
        self.state = STATES[0]
        self.controllers[self.state].start(time.time())

        self.prev_state = None
        self.bumped = False
        self.paused = False
        self.ir_values = [0] * 7  # latest IR readings

    def on_ir(self, msg: IrIntensityVector):
        """React to IR readings (proximity sensors)."""
        self.ir_values = [r.value for r in msg.readings]
        front = sum(self.ir_values[0:2])
        right = sum(self.ir_values[2:4])
        left  = sum(self.ir_values[5:7])
        # Trigger avoidance if something is close enough
        if (front > 150 or right > 200 or left > 200) and self.state != "AVOID":
            self.enter_avoid("ir")

    def on_hazard(self, msg: HazardDetectionVector):
        """React to hazard messages (like bumps)."""
        if not msg.detections:
            return
        for d in msg.detections:
            if d.type == HazardDetection.BUMP and self.state != "AVOID":
                self.bumped = True
                self.enter_avoid("bump")
                break

    def enter_avoid(self, reason: str):
        """Switch to AVOID state and log reason."""
        self.prev_state = self.state
        self.state = "AVOID"
        self.controllers["AVOID"].start(time.time())
        self.get_logger().warn(f"Entering AVOID due to {reason}")

    def tick(self):
        """Main loop — updates phase, sends velocity, handles state switching."""
        now = time.time()
        
        if self.paused:
            # Stop the robot immediately
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return  # skip everything else
        
        ctrl = self.controllers[self.state]
        ctrl.update(now)  # Advance phase if duration elapsed
        phase = ctrl.current_phase()

        # Handle AVOID state separately
        if self.state == "AVOID":
            if phase == "avoid":
                # Continue spinning until IR sensors detect clearance
                if all(v < 100 for v in self.ir_values):
                    ctrl.skip(now)  # advance to "stop"
                    phase = ctrl.current_phase()
            elif phase == "stop":
                # Resume a random normal state
                self.state = random.choice(STATES)
                ctrl = self.controllers[self.state]
                ctrl.start(now)
                phase = ctrl.current_phase()
                self.bumped = False

        else:
            # Normal states: check if all phases finished
            if ctrl.finished:
                # Switch to a new random state
                self.state = random.choice(STATES)
                ctrl = self.controllers[self.state]
                ctrl.start(now)
                phase = ctrl.current_phase()

        # Publish velocity for the current phase
        lin, ang = SPEED.get(phase, (0.0, 0.0))
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.pub.publish(cmd)



    def ui_snapshot(self):
        """Return snapshot of current state for UI."""
        c = self.controllers[self.state]
        return self.state, c.current_phase(), self.bumped


# ────────────────────────────────────────────────────────────────────────────────
# Pygame face UI
# ────────────────────────────────────────────────────────────────────────────────
def load_faces(screen):
    """Load and scale face images from folder."""
    w, h = screen.get_size()
    folder = "/home/cuterobot/robotFaces2.0"  # Adjust path as needed

    def L(name):
        return pygame.transform.scale(pygame.image.load(os.path.join(folder, name)), (w, h))

    faces = {
        "normal": L("Normal.png"),
        "blink": L("Blinking.png"),
        "confused": L("Confused.png"),
        "dazed": L("Dazed.png"),
        "sleepy": L("Sleepy.png"),
        "dizzy": L("Dizzy.png"),
    }
    # Mapping from robot state to default face
    face_map = {
        "STRAIGHT": faces["normal"],
        "DIZZY": faces["dizzy"],
        "YAWN": faces["sleepy"],
        "SINEWAVE": faces["normal"],
        "AVOID_confused": faces["confused"],
        "AVOID_dazed": faces["dazed"],
        "NORMAL": faces["normal"],
    }
    return faces, face_map


def spin_executor(executor):
    """Run ROS executor loop until shutdown."""
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass


def main():
    # ────────── ROS setup ──────────
    rclpy.init()
    node = MotionNode()
    exec_ = rclpy.executors.SingleThreadedExecutor()
    exec_.add_node(node)
    # Run ROS in a background thread so pygame loop stays responsive
    t = threading.Thread(target=lambda: spin_executor(exec_), daemon=True)
    t.start()

    # ────────── Pygame setup ──────────
    pygame.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.display.set_caption("Cute Robot Face")
    clock = pygame.time.Clock()
    faces, face_map = load_faces(screen)
    blink = BlinkManager(min_gap=3, max_gap=6, duration=0.15)

    w, h = screen.get_size()
    font = pygame.font.SysFont(None, 60)
    def btn(text, rect, col=(100,200,100)):
        """Draw rounded rectangle button with text."""
        pygame.draw.rect(screen, col, rect, border_radius=15)
        label = font.render(text, True, (0,0,0))
        screen.blit(label, label.get_rect(center=rect.center))

    paused = False
    running = True
    resume_rect = pygame.Rect(w//2 - w//4, 100, w//2, 100)
    exit_rect   = pygame.Rect(w//2 - w//4, h - 200, w//2, 100)

    # ────────── Main UI loop ──────────
    while running:
        clock.tick(60)  # 60 FPS

        # Handle events
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                running = False
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                if paused:
                    if resume_rect.collidepoint(e.pos): 
                        paused = False
                        node.paused = False
                    elif exit_rect.collidepoint(e.pos): running = False
                else:
                    paused = True
                    node.paused = True

        # Get robot state for face selection
        state, phase, bumped = node.ui_snapshot()

        # Select face based on state
        if state == "AVOID":
            face = face_map["AVOID_confused"]
        elif phase == "spin_fast_right_dizzy" or phase == "dizzy":
            face = face_map["DIZZY"]
        elif phase == "yawn":
            face = face_map["YAWN"]
        else:
            face = face_map["NORMAL"]

        # Handle blinking animation
        if state == "STRAIGHT" or state == "SINEWAVE":
            blink.update()
            if blink.blinking():
                face = faces["blink"]

        # Draw face
        screen.blit(face, (0, 0))

        # Pause overlay UI
        if paused:
            overlay = pygame.Surface((w, h), pygame.SRCALPHA)
            overlay.fill((0,0,0,180))
            screen.blit(overlay, (0,0))
            btn("Resume", resume_rect)
            btn("Exit", exit_rect)

        pygame.display.flip()

    # ────────── Cleanup ──────────
    exec_.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
