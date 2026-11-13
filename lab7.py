"""
Lab 7: More PID
===============

Built on the provided skeleton labs/lab6/code.py, with its TODOs filled in,
the missing helpers (reset_odometry / reset_controls) added, and the
wall-following code added at the bottom. Copy this file to the CIRCUITPY
drive as `code.py`, along with `irsensor.py` and lib/ (adafruit_motor,
neopixel). Set PART below:

  PART = 1  -> PID drive straight 200 mm and stop            [Checkoff #2.1]
  PART = 2  -> turn_left() 90°, pause 1 s, repeat             [Checkoff #2.2-2.3]
  PART = 3  -> forward() 180 mm wall following, every 1 s     [Checkoff #3]

Before running PART 3, put YOUR IR calibration constants from Lab 4 into
the `ir_dist = Distance(...)` call below.

-----------------------------------------------------------------------------
Checkoff #1 answers (no code)
  1. Integral: use a Riemann sum, i.e. add up the error every loop
     iteration (e_sum += e). Strictly it's e * dt, but with a fixed loop
     period dt is a constant that gets absorbed into Ki.
  2. Derivative: use a finite difference, (e - e_prev). Strictly it's
     (e - e_prev) / dt, and again dt gets absorbed into Kd.
  3. Ways to prevent integral windup: clamp the accumulated sum (what we do
     here), stop integrating while the output is saturated, reset the
     integral when a new move starts (reset_controls), or decay/leak the
     integral over time. A good clamp is whatever sum alone would command
     full motor power.

Checkoff #2 answers
  1-3. See PART 1 and PART 2.
  4. The integral term adds e to a running sum every loop (e_ang_sum,
     e_lin_sum), and Ki * sum is added to the output. Windup is handled by
     clamping the sum to [-20, 20] (angular) and [-100, 100] (linear), and
     by resetting it after every move with reset_controls().

Checkoff #3 answers
  1. See PART 3. forward() drives 180 mm while holding 45 mm from the left
     wall, then stops.
  2. Between two walls: instead of a fixed 45 mm target, drive the
     difference between the left and right distances to 0 (target
     left_dist == right_dist, i.e. stay centered). Use the angle estimates
     from both sides, and fall back to one wall or plain encoder odometry
     when a wall is missing (IR reading too far / no wall detected).
"""

PART = 1  # <-- change to 1, 2, or 3

import board
import time
from math import pi

import digitalio
import neopixel
import rotaryio
import pwmio
import adafruit_motor.motor as motor

""" Constants """

ENCODER_TICKS_PER_REVOLUTION = 217
WHEELBASE_DIAMETER = 78.0 # mm
WHEEL_DIAMETER = 34.0 # mm

""" Peripherals """

# debug
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

but = digitalio.DigitalInOut(board.GP3)
but.pull = digitalio.Pull.UP

rgb = neopixel.NeoPixel(board.GP4, 1)

# encoders
lenc = rotaryio.IncrementalEncoder(board.GP12, board.GP13)
renc = rotaryio.IncrementalEncoder(board.GP19, board.GP18)

# motors
lmot = motor.DCMotor(
    pwmio.PWMOut(board.GP16, frequency=20000),
    pwmio.PWMOut(board.GP17, frequency=20000)
)
rmot = motor.DCMotor(
    pwmio.PWMOut(board.GP15, frequency=20000),
    pwmio.PWMOut(board.GP14, frequency=20000)
)
lmot.decay_mode = motor.SLOW_DECAY
rmot.decay_mode = motor.SLOW_DECAY

""" Main """

def constrain(val, min_val, max_val):
    return min(max_val, max(val, min_val))

# compute dist and theta
def compute_odometry():
    MM_PER_TICK = pi * WHEEL_DIAMETER / ENCODER_TICKS_PER_REVOLUTION

    # compute left dist
    left_dist = lenc.position * MM_PER_TICK
    # compute right dist
    right_dist = renc.position * MM_PER_TICK

    #compute dist and theta
    dist  = (left_dist + right_dist) / 2                  # forward distance (mm)
    theta = (right_dist - left_dist) / WHEELBASE_DIAMETER # heading (rad), + = left/CCW

    # what should you be returning here? ... , ...
    return dist, theta


# compute correction and error terms for a target theta
e_ang_sum, e_ang_prev = 0, 0
def compute_u_ang(theta, theta_target):
    Kp, Ki, Kd = 0.1, 0.005, 0.01 # Proportional (Kp), Integral (Ki), and Derivative (Kd)

    global e_ang_sum, e_ang_prev
    #whats the _error_ of our angle?
    e_ang = theta_target - theta

    # Update i
    e_ang_sum += e_ang  # Riemann sum of the error (dt folded into Ki)
    e_ang_sum = constrain(e_ang_sum, -20, 20) # windup

    # update d
    e_ang_deriv = e_ang - e_ang_prev  # finite difference (dt folded into Kd)
    e_ang_prev  = e_ang

    u_ang = Kp * e_ang + Ki * e_ang_sum + Kd * e_ang_deriv # you should be using Kp Ki and Kd here, use the PID formula
    return u_ang, e_ang

# compute correction and error terms for a target distance
e_lin_sum, e_lin_prev = 0, 0
def compute_u_lin(dist, dist_target):
    # follow a similar process as above, set the wind up constraint to (-100, 100)
    Kp, Ki, Kd = 0.01, 0.0005, 0.005  # tune linear first; Ki/Kd ~10x smaller than Kp

    global e_lin_sum, e_lin_prev
    e_lin = dist_target - dist

    # update i
    e_lin_sum += e_lin
    e_lin_sum = constrain(e_lin_sum, -100, 100) # windup

    # update d
    e_lin_deriv = e_lin - e_lin_prev
    e_lin_prev  = e_lin

    u_lin = Kp * e_lin + Ki * e_lin_sum + Kd * e_lin_deriv
    u_lin = constrain(u_lin, -0.3, 0.3)  # leave headroom for the angular correction
    return u_lin, e_lin

def run_control_loop(theta_target, dist_target):
    dist, theta = compute_odometry()
    u_ang, e_ang = compute_u_ang(theta, theta_target)
    u_lin, e_lin = compute_u_lin(dist, dist_target)
    lmot.throttle = constrain(u_lin - u_ang, -1, 1)
    rmot.throttle = constrain(u_lin + u_ang, -1, 1)
    return e_ang, e_lin

# --- helpers used by turn_left() (given in docs/lab7.md / added) -------------

def reset_odometry():
    # zero the encoders so every move starts from dist = 0, theta = 0
    lenc.position, renc.position = 0, 0

def reset_controls():
    # clear integral sums and previous errors so the next move starts fresh
    # (also prevents windup carrying over between moves)
    global e_ang_sum, e_ang_prev, e_lin_sum, e_lin_prev
    e_ang_sum, e_ang_prev, e_lin_sum, e_lin_prev = 0, 0, 0, 0

def turn_left():
    reset_odometry()
    # target +90° (pi/2 rad) heading and 0 mm forward -> turn in place.
    # Loop until the angular error is within 0.05 rad (~3°).
    while abs(run_control_loop(pi/2, 0)[0]) > 0.05:
        time.sleep(0.02)
    reset_controls()
    lmot.throttle, rmot.throttle = 0, 0

# =============================================================================
# Wall following (Checkoff #3)
# =============================================================================
# The angular controller is redefined to use the IR sensors:
#   theta        = (left sensor a distance) - (left sensor b distance)
#                  i.e. the difference between the two left sensors. It's 0
#                  when the mouse is parallel to the wall, so it measures the
#                  angle to the wall.
#   theta_target = Kp_theta * (45mm - left distance)
#                  i.e. aim at a small angle toward/away from the wall based
#                  on the distance error, and let forward motion close the gap.
if PART == 3:
    from irsensor import IRSensors

    # IR sensors
    ir = IRSensors(
        board.GP7,  board.GP5,  board.GP6,  board.GP28, # left
        board.GP9,  board.GP10, board.GP11, board.GP26, # center
        board.GP21, board.GP20, board.GP22, board.GP27, # right
        avg = 10
    )

    # Distance class from Lab 4 (labs/lab4/distance.py)
    class Distance():
        def __init__(self,
            ir,
            la_a, la_b, lb_a, lb_b,
            ca_a, ca_b, cb_a, cb_b,
            ra_a, ra_b, rb_a, rb_b,
        ):
            # store constants
            self.ir = ir
            self.la_a, self.la_b, self.lb_a, self.lb_b = la_a, la_b, lb_a, lb_b
            self.ca_a, self.ca_b, self.cb_a, self.cb_b = ca_a, ca_b, cb_a, cb_b
            self.ra_a, self.ra_b, self.rb_a, self.rb_b = ra_a, ra_b, rb_a, rb_b

            # allocate memory for distances
            self.la, self.lb = 0, 0
            self.ca, self.cb = 0, 0
            self.ra, self.rb = 0, 0

        def scan(self):
            ir.scan()
            self.la = self.la_a * ir.lir_a + self.la_b
            self.lb = self.lb_a * ir.lir_b + self.lb_b
            self.ca = self.ca_a * ir.cir_a + self.ca_b
            self.cb = self.cb_a * ir.cir_b + self.cb_b
            self.ra = self.ra_a * ir.rir_a + self.ra_b
            self.rb = self.rb_a * ir.rir_b + self.rb_b

    # TODO: replace with YOUR constants from Lab 4 calibration
    ir_dist = Distance(ir,
        0.0299, -63.6, 0.0195, -39.1,
        0.0300, -60.4, 0.0251, -47.9,
        0.0258, -50.5, 0.0292, -54.0,
    )

    WALL_DIST_TARGET = 45  # mm from the left wall
    FORWARD_DIST = 180     # mm per forward() call

    def compute_u_ang_wall(wall_dist, wall_dist_target):
        # P-only is enough here
        Kp_theta = 0.01  # distance error (mm) -> desired angle
        Kp_ang = 0.01    # angle error -> steering power

        # distance error from the wall
        e_theta = wall_dist_target - wall_dist

        # new definitions of theta / theta_target (see block comment above)
        theta_target = Kp_theta * e_theta
        theta = ir_dist.la - ir_dist.lb

        e_ang = theta_target - theta

        u_ang = Kp_ang * e_ang
        u_ang = constrain(u_ang, -0.3, 0.3)
        # NOTE: if your mouse steers AWAY from the wall instead of toward the
        # target distance, your a/b sensors are mounted the other way. Flip
        # the sign of u_ang (or swap la/lb above).
        return u_ang, e_ang

    def run_wall_control_loop(dist_target):
        dist, theta = compute_odometry()  # encoder theta unused; IR gives the angle
        ir_dist.scan()
        u_ang, e_ang = compute_u_ang_wall(ir_dist.la, WALL_DIST_TARGET)
        u_lin, e_lin = compute_u_lin(dist, dist_target)  # same linear PID as before
        lmot.throttle = constrain(u_lin - u_ang, -1, 1)
        rmot.throttle = constrain(u_lin + u_ang, -1, 1)
        return e_ang, e_lin

    def forward():
        reset_odometry()
        # drive until within 3 mm of the 180 mm target
        while abs(run_wall_control_loop(FORWARD_DIST)[1]) > 3:
            time.sleep(0.02)
        reset_controls()
        lmot.throttle, rmot.throttle = 0, 0

if __name__ == "__main__":
    #Try using this to turn left instead
    if PART == 1:
        # Checkoff #2.1: PID drive straight 200mm and hold there
        while True:
            e_ang, e_lin = run_control_loop(0, 200)
            print(e_ang, e_lin)
            time.sleep(0.02) # ~50Hz loop

    elif PART == 2:
        # Checkoff #2.2 / #2.3: turn 90° in place, stop 1s, repeat
        while True:
            turn_left()
            time.sleep(1)

    elif PART == 3:
        # Checkoff #3: wall-follow forward 180mm once every second
        while True:
            forward()
            time.sleep(1)
