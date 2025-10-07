#!/usr/bin/env python3

# Core modules for system operations, timing, randomness, and threading
import os
import sys
import time
import random
import threading

# External dependencies: Pygame for face display; ROS 2 (rclpy) for robot control
import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector, HazardDetection

# Basic constant for readability
SECOND = 1.0 

# Bump behaviour parameters
BUMP_TURN_RATE = 0.75   
BUMP_CLEAR_HOLD = 1.0 

# ────────────────────────────────────────────────────────────────────────────────
# Helpers
# ────────────────────────────────────────────────────────────────────────────────

# Controls time-based transitions between motion phases within a behaviour state
class PhaseController:
    def __init__(self, phases):
        self.phases = phases
        self.idx = 0
        self.started_at = None
        self.finished = False

    # Start the first phase and reset timers
    def start(self, now: float):
        self.idx = 0
        self.started_at = now
        self.finished = False

    # Return the name of the current phase
    def current_phase(self):
        return self.phases[self.idx][0]

    # Progress through phases based on elapsed time
    def update(self, now: float):
        if self.started_at is None:
            self.started_at = now
            return
        _, dur = self.phases[self.idx]
        if dur != float("inf") and (now - self.started_at) >= dur:
            if self.idx == len(self.phases) - 1:
                self.finished = True
            self.idx = (self.idx + 1) % len(self.phases)
            self.started_at = now

    # Manually skip to the next phase
    def skip(self, now: float):
        self.idx = (self.idx + 1) % len(self.phases)
        self.started_at = now


# Converts a sequence of (phase, duration) pairs into time-adjusted tuples
def _phases(seq):
    out = []
    for name, dur in seq:
        out.append((name, dur if dur == float("inf") else dur * SECOND))
    return out


# Handles randomised blink timing for the robot face display
class BlinkManager:
    def __init__(self, min_gap=3, max_gap=6, duration=0.15):
        self.min_gap = min_gap
        self.max_gap = max_gap
        self.duration = duration
        now = time.time()
        self.next_blink = now + random.uniform(min_gap, max_gap)
        self.active_since = None

    # Manually trigger a blink
    def trigger(self):
        self.active_since = time.time()
        self.next_blink = time.time() + random.uniform(self.min_gap, self.max_gap)

    # Update blink timing and reset after blink duration
    def update(self):
        now = time.time()
        if self.active_since is None and now >= self.next_blink:
            self.active_since = now
            self.next_blink = now + random.uniform(self.min_gap, self.max_gap)
        if self.active_since and (now - self.active_since) > self.duration:
            self.active_since = None

    # Return True if currently blinking
    def blinking(self):
        return self.active_since is not None


# ────────────────────────────────────────────────────────────────────────────────
# Behaviour config
# ────────────────────────────────────────────────────────────────────────────────

# Defines all possible behaviour states
STATES = [ "SWERVE"]

# Defines ordered phase sequences (behaviour scripts)
STORY = {
    "ZIGZAG": _phases([("move", 1.5), ("pause", 0.4), ("blink", 0.1), ("turn_zigzag", 0.6)]),
    "JERK":   _phases([("move", 0.5), ("pause", 0.2), ("blink", 0.1)]),
    "TWITCH": _phases([("move", 3.0), ("turn_twitch", 2.0), ("move2", 3.0), ("pause", 0.5), ("twitch", 3.0)]),
    "SWERVE": _phases([("turn_swerve", 3.0), ("move", 2.0), ("twitch_swerve", random.choice([3.0, 5.0]))]),
    "BUMP":   _phases([("pause1", 0.5), ("blink1", 0.1), ("mad", 0.6), ("turn_bump", float("inf")), ("blink2", 0.1), ("resume", 0.1)]),
}

# Defines linear (x) and angular (z) speed values for each phase
SPEED = {
    "move":   (0.20, 0.00),
    "move2":  (0.40, 0.00),
    "pause":  (0.00, 0.00),
    "blink":  (0.00, 0.00),
    "turn_zigzag":   (0.00, 4.00),
    "turn_twitch":   (0.2,  random.choice([-0.5, 0.5])),  
    "turn_swerve":   (0.2, 0.5),  
    "twitch": (0.00, 0.00),
    "twitch_swerve": (0.00, random.choice([1.0, 3.0])),

    # Bump behaviour
    "pause1": (0.00, 0.00),
    "blink1": (0.00, 0.00),
    "mad":    (0.00, 0.00),
    "blink2": (0.00, 0.00),
    "resume": (0.00, 0.00),
    "turn_bump": (0.00, 0.00),
}

# ────────────────────────────────────────────────────────────────────────────────
# ROS 2 Motion Node
# ────────────────────────────────────────────────────────────────────────────────

# Node controlling robot motion, hazard response, and state transitions
class MotionNode(Node):
    def __init__(self):
        super().__init__("creepy_motion_node")

        # Publisher for velocity commands
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriptions for hazard and IR sensors
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(HazardDetectionVector, "/hazard_detection", self.on_hazard, qos)
        self.create_subscription(IrIntensityVector, "/ir_intensity", self.on_ir, qos)

        # Timer loop for periodic updates
        self.timer = self.create_timer(0.1, self.tick)

        # Behaviour controllers per state
        self.controllers = {name: PhaseController(STORY[name]) for name in STORY}
        self.state = "ZIGZAG"
        self.controllers[self.state].start(time.time())

        # Internal state tracking
        self.prev_state = None
        self.bumped = False
        self.paused = False

        # Sensor and direction variables
        self.ir_values = [0] * 7
        self.zigzag_dir = 1.0
        self.swerve_dir = random.choice([-1.0, 1.0])
        self.turn_direction = None
        self._last_clear_check = 0.0
        self._current_phase = None

    # Process IR proximity readings
    def on_ir(self, msg: IrIntensityVector):
        self.ir_values = [r.value for r in msg.readings]
        front = sum(self.ir_values[0:2])
        right = sum(self.ir_values[2:4])
        left  = sum(self.ir_values[5:7])
        if (front > 150 or right > 200 or left > 200) and self.state != "BUMP":
            self.enter_bump("ir")

    # Process physical bump events
    def on_hazard(self, msg: HazardDetectionVector):
        if not msg.detections:
            return
        for d in msg.detections:
            if d.type == HazardDetection.BUMP and self.state != "BUMP":
                self.bumped = True
                self.enter_bump("bump")
                break

    # Enter bump recovery behaviour
    def enter_bump(self, reason: str):
        self.prev_state = self.state
        self.state = "BUMP"
        self.controllers["BUMP"].start(time.time())
        self.get_logger().warn(f"Entering BUMP due to {reason}")
        self.turn_direction = None
        self._last_clear_check = 0.0

    # Main update loop for motion
    def tick(self):
        now = time.time()

        # Stop when paused
        if self.paused:
            self.pub.publish(Twist())
            return

        ctrl = self.controllers[self.state]
        ctrl.update(now)
        phase = ctrl.current_phase()

        # Detect if entering a new phase
        entered_phase = (phase != self._current_phase)
        self._current_phase = phase

        # Handle bump state separately
        if self.state == "BUMP":
            self.get_logger().warn(f"Tick bump")
            cmd = self._tick_bump(now, ctrl, phase)
            self.pub.publish(cmd)
            return

        # Create motion command
        cmd = Twist()
        lin, ang = SPEED.get(phase, (0.0, 0.0))
        cmd.linear.x = lin
        cmd.angular.z = ang

        # Apply direction toggling for zigzag motion
        if self.state == "ZIGZAG":
            if phase == "turn_zigzag" and entered_phase:
                self.zigzag_dir *= -1
            if phase == "turn_zigzag":
                cmd.angular.z = abs(SPEED["turn_zigzag"][1]) * self.zigzag_dir
                
        # Apply swerve oscillation
        elif self.state == "SWERVE":
            if phase == "turn_swerve":
                cmd.angular.z *=  self.swerve_dir
            elif phase == "twitch_swerve":
                cmd.angular.z = random.choice([1.0,3.0]) * self.swerve_dir
                self.swerve_dir *= -1

        # Send command to robot
        self.pub.publish(cmd)

    # Handles bump recovery phases and transitions
    def _tick_bump(self, now, ctrl: PhaseController, phase: str) -> Twist:
        cmd = Twist()
        self.get_logger().warn(f"phase {phase}")

        # Stop for non-turn phases
        if phase in ("pause1", "blink1", "mad", "blink2", "resume"):
            if phase == "resume":
                self.state = random.choice(STATES)
                self.controllers[self.state].start(now)
                self.get_logger().warn(f"next phase {self.state}")
                self.bumped = False
            return cmd

        # Handle spin recovery
        if phase == "turn_bump":
            if self.turn_direction is None:
                self.turn_direction = random.choice([-1.0, 1.0])
                self._last_clear_check = 0.0

            cmd.angular.z = self.turn_direction * BUMP_TURN_RATE

            # Brief reverse after bump
            if self.bumped:
                cmd.linear.x = -0.1
                self.bumped = False
            else:
                cmd.linear.x = 0.0

            # Check if area is clear before resuming
            if all(v < 100 for v in self.ir_values):
                if self._last_clear_check == 0.0:
                    self._last_clear_check = now
                elif (now - self._last_clear_check) > BUMP_CLEAR_HOLD:
                    ctrl.skip(now)
                    self.turn_direction = None
                    self._last_clear_check = 0.0
            else:
                self._last_clear_check = 0.0

        return cmd

    # Used by UI for debugging and display
    def ui_snapshot(self):
        c = self.controllers[self.state]
        return self.state, c.current_phase(), self.bumped


# ────────────────────────────────────────────────────────────────────────────────
# Pygame Face UI
# ────────────────────────────────────────────────────────────────────────────────

# Load and scale all face images for display
def load_faces(screen):
    w, h = screen.get_size()
    folder = os.path.join(os.path.dirname(__file__), "faces/final_faces")
    def L(name): return pygame.transform.scale(pygame.image.load(os.path.join(folder, name)), (w, h))
    faces = {
        "reg":            L("creepy_face_reg.png"),
        "blink":          L("creepy_face_blink.png"),
        "roll_eyes":      L("creepy_face_roll_eyes.png"),
        "sinister":       L("creepy_face_sinister.png"),
        "sinister_blink": L("creepy_face_sinister_blink.png"),
        "twitch":         L("creepy_face_twitch.png"),
        "side_eye":       L("creepy_face_side_eye.png"),
    }
    face_map = {
        "ZIGZAG": faces["reg"],
        "JERK":   faces["roll_eyes"],
        "TWITCH": faces["reg"],
        "SWERVE": faces["reg"],
        "BUMP_mad": faces["sinister"],
    }
    return faces, face_map


# Keeps ROS2 executor spinning on a background thread
def spin_executor(executor):
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass


# Main entry point for the full ROS2 + Pygame loop
def main():
    # Initialise ROS and node
    rclpy.init()
    node = MotionNode()
    exec_ = rclpy.executors.SingleThreadedExecutor()
    exec_.add_node(node)
    threading.Thread(target=lambda: spin_executor(exec_), daemon=True).start()

    # Initialise Pygame fullscreen window
    pygame.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.display.set_caption("Creepy Robot Face")
    clock = pygame.time.Clock()
    faces, face_map = load_faces(screen)
    blink = BlinkManager(min_gap=3, max_gap=6, duration=0.1)

    # Layout constants
    w, h = screen.get_size()
    font = pygame.font.SysFont(None, 60)
    def btn(text, rect, col=(100,200,100)):
        pygame.draw.rect(screen, col, rect, border_radius=15)
        label = font.render(text, True, (0,0,0))
        screen.blit(label, label.get_rect(center=rect.center))

    paused = False
    running = True
    resume_rect = pygame.Rect(w//2 - w//4, 100, w//2, 100)
    exit_rect   = pygame.Rect(w//2 - w//4, h - 200, w//2, 100)

    # Twitch flicker timing
    TWITCH_INTERVAL = 0.25
    last_twitch_flip = 0.0
    twitch_on = False

    # Main event + render loop
    while running:
        clock.tick(60)

        # Handle user input and window events
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                running = False
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                if paused:
                    if resume_rect.collidepoint(e.pos):
                        paused = False; node.paused = False
                    elif exit_rect.collidepoint(e.pos):
                        running = False
                else:
                    paused = True; node.paused = True

        # Sync face display with robot state
        state, phase, bumped = node.ui_snapshot()

        # Choose base face by state
        if state == "BUMP" and (phase in ("mad", "turn_bump")):
            face = face_map["BUMP_mad"]
        elif state == "JERK":
            face = face_map["JERK"]
        elif state == "ZIGZAG":
            face = face_map["ZIGZAG"]
        elif state == "SWERVE":
            face = face_map["SWERVE"]
        else:
            face = face_map.get(state, faces["reg"])

        # Apply special face overlays for blink/twitch events
        if state == "BUMP" and phase in ("blink1", "blink2"):
            face_to_draw = faces["sinister_blink"]
        else:
            # Handle twitch flicker toggling
            if state in ("TWITCH", "SWERVE") and (phase == "twitch" or phase == "twitch_swerve"):
                nowt = time.time()
                if (nowt - last_twitch_flip) >= TWITCH_INTERVAL:
                    twitch_on = not twitch_on
                    last_twitch_flip = nowt
                face_to_draw = faces["twitch"] if twitch_on else face
            else:
                twitch_on = False
                face_to_draw = face

            # Random blinks in normal motion states
            if state in ("ZIGZAG", "JERK", "TWITCH", "SWERVE"):
                blink.update()
                if blink.blinking():
                    face_to_draw = faces["sinister_blink"] if face_to_draw == faces["sinister"] else faces["blink"]

        # Render face and overlay buttons if paused
        screen.blit(face_to_draw, (0, 0))
        if paused:
            overlay = pygame.Surface((w, h), pygame.SRCALPHA)
            overlay.fill((0,0,0,180))
            screen.blit(overlay, (0,0))
            btn("Resume", resume_rect)
            btn("Exit", exit_rect)

        pygame.display.flip()

    # Cleanup on exit
    exec_.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()
    sys.exit(0)


# Run main loop when executed directly
if __name__ == "__main__":
    main()
