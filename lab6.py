"""
Lab 6: Intro to (P) of PID
==========================

Built on the provided skeleton labs/lab5/code.py (constants + peripherals),
with the Lab 5 odometry code and the P controllers from docs/lab6.md filled
in. Copy this file to the CIRCUITPY drive as `code.py` (needs
lib/adafruit_motor and lib/neopixel). Set PART below:

  PART = 1  -> Drive Straight (angular P control + 0.2 base) [Checkoff #1]
  PART = 2  -> Go the Distance (angular + linear P, 200 mm)  [Checkoff #2]

Sign convention: theta = (right - left) / wheelbase, so theta > 0 means the
mouse turned LEFT. A positive steer correction u slows the left motor and
speeds up the right (lmot = base - u, rmot = base + u), which turns the
mouse left, i.e. it increases theta.

-----------------------------------------------------------------------------
Checkoff #1 answers
  1. See PART 1. It prints the angular error.
  2. error = theta_target - theta, with theta_target = 0 for straight. The
     error doesn't have to reach exactly 0, it just has to stay stable.
  3. u = Kp * e, so 1 = Kp * 2 rad and Kp = 0.5.
  4. If Kp is too low, the mouse corrects too slowly (or not at all) and
     drifts. If Kp is too high, it overcorrects, wobbles side to side, and
     can go unstable. This file uses 0.1; tune it on your own mouse.

Checkoff #2 answers
  1. See PART 2. It prints the angular and linear errors.
  2. lin_error = dist_target - dist, with dist = (left + right) / 2.
  3. u = Kp * e, so 1 = Kp * 100 mm and Kp = 0.01.
"""

PART = 1  # <-- change to 1 or 2

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

# =============================================================================
# PART 1: Drive Straight
# =============================================================================
if PART == 1:
    # clamp to the valid throttle range [-1, 1]
    def constrain(val):
        return min(1, max(val, -1))
    if __name__ == "__main__":
        MM_PER_TICK = pi * WHEEL_DIAMETER / ENCODER_TICKS_PER_REVOLUTION

        while True:
            # Calculate the left distance value
            left_dist = lenc.position * MM_PER_TICK
            # Calculate the right distance value
            right_dist = renc.position * MM_PER_TICK
            # Get both theta and distance from lab
            dist = (left_dist + right_dist) / 2
            theta = (right_dist - left_dist) / WHEELBASE_DIAMETER

            # set Kp to a low value to begin, 0.1 etc, what do you notice when you vary this?
            k_p = 0.1
            # What is our target theta value?
            theta_target = 0  # driving straight = no rotation
            # whats the error of our target and the theta?
            angular_error = theta_target - theta

            # define the steering correction, Our correction should be proporitonal to our angular error
            # the action
            steer_correction = k_p * angular_error

            #set the throttles of the left/right motor.
            # Lets constrain it to 0.2 plus or minus our steering correction depending on the motor
            # If we drifted right (theta < 0 -> error > 0), slow left / speed up right to turn back left.
            lmot.throttle = constrain(0.2 - steer_correction)
            rmot.throttle = constrain(0.2 + steer_correction)

            print(angular_error)

            time.sleep(0.05)

# =============================================================================
# PART 2: Go the Distance
# =============================================================================
if PART == 2:
    # clamp val into [min_val, max_val]
    def constrain(val, min_val, max_val):
        return min(max_val, max(val, min_val))

    if __name__ == "__main__":
        MM_PER_TICK = pi * WHEEL_DIAMETER / ENCODER_TICKS_PER_REVOLUTION

        while True:
            left_dist  = lenc.position * MM_PER_TICK
            right_dist = renc.position * MM_PER_TICK

            dist  = (left_dist + right_dist) / 2
            theta = (right_dist - left_dist) / WHEELBASE_DIAMETER

            # angular P control (keeps us driving straight)
            k_p = 0.3
            theta_target = 0.0

            angular_error = theta_target - theta
            steer_correction = k_p * angular_error

            # linear P control (gets us to the target distance)
            Kp_lin = 0.01
            dist_target = 200 # in mm

            # linear error?  positive = still short of target, negative = overshot
            lin_error = dist_target - dist
            # cap at +/-0.3 to leave headroom for the steering correction
            lin_correction = constrain(Kp_lin * lin_error, -0.3, 0.3)

            # combine linear correction and steer correction (action terms) to ensure that our mouse's motor
            # differentials are corrected, just as the slides say.
            # linear term = common speed for both wheels, steer term = difference between them
            lmot.throttle = constrain(lin_correction - steer_correction, -1, 1)
            rmot.throttle = constrain(lin_correction + steer_correction, -1, 1)

            print(angular_error, lin_error)

            time.sleep(0.05)
