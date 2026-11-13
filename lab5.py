"""
Lab 5: Odometry and Motor Control
=================================

Copy this file to the CIRCUITPY drive as `code.py` (PART 3 also needs
lib/adafruit_motor). Set PART below:

  PART = 1  -> Odometry (dist + theta from encoders)          [Checkoff #1]
  PART = 2  -> Raw PWM motor control, left wheel 25% forward  [Checkoff #2]
  PART = 3  -> Motor control with the adafruit_motor library  [Checkoff #3]

Odometry equations (differential drive):
  d_f   = (d_l + d_r) / 2          forward distance of the mouse's center
  theta = (d_r - d_l) / d          rotation in radians, d = wheelbase

-----------------------------------------------------------------------------
Checkoff #1 answers
  1. See PART 1. Push the mouse straight to check that dist matches a ruler,
     then rotate it in place to check that theta matches the angle.
  2. mm per tick = pi * WHEEL_DIAMETER / ENCODER_TICKS_PER_REVOLUTION
                 = pi * 34 / 217 ~= 0.49 mm (about 0.5 mm per tick).

Checkoff #2 answers
  1. See PART 2. From the DRV8833 table: IN1=1, IN2=0 is forward and
     IN1=1, IN2=1 is brake (slow decay). Holding IN1 high and PWM-ing IN2
     switches between forward and brake. The mouse only moves forward while
     IN2 is LOW, so for 25% speed IN2 has to be HIGH 75% of the time:
     duty = 0.75 * 65535 ~= 49151. As IN2's duty cycle goes UP, the motor
     slows down.
  2. Swapping the two PWM pins reverses the motor direction.

Checkoff #3 answers
  1. See PART 3.
  2. A throttle above 1.0 or below -1.0 raises a ValueError, so later labs
     clamp throttle to [-1, 1] with a constrain() helper.
  3. Save this code, you'll need it for the next lab.
"""

PART = 1  # <-- change to 1, 2, or 3

import board
import time

# =============================================================================
# PART 1: Odometry
# =============================================================================
if PART == 1:
    import rotaryio
    from math import pi

    lenc = rotaryio.IncrementalEncoder(board.GP12, board.GP13)
    renc = rotaryio.IncrementalEncoder(board.GP19, board.GP18)

    ENCODER_TICKS_PER_REVOLUTION = 217 # TODO from previous lab
    WHEELBASE_DIAMETER = 78.0 # mm     # TODO measure this in mm (distance between the two wheels)
    WHEEL_DIAMETER = 34.0 # mm

    # distance one tick moves the wheel = wheel circumference / ticks per rev
    MM_PER_TICK = pi * WHEEL_DIAMETER / ENCODER_TICKS_PER_REVOLUTION

    while True:
        left_dist  = lenc.position * MM_PER_TICK   # TODO
        right_dist = renc.position * MM_PER_TICK   # TODO

        dist  = (left_dist + right_dist) / 2                 # TODO average of the wheels
        theta = (right_dist - left_dist) / WHEELBASE_DIAMETER # TODO radians, + = turned left (CCW)

        print(dist, theta)
        time.sleep(0.05)

# =============================================================================
# PART 2: Motor Control (raw PWM)
# =============================================================================
if PART == 2:
    import pwmio

    lmot_in1 = pwmio.PWMOut(board.GP16, frequency=20000) # TODO create PWMOut object on pin GP16 at 20kHz
    lmot_in2 = pwmio.PWMOut(board.GP17, frequency=20000) # TODO create PWMOut object on pin GP17 at 20kHz

    while True:
        # IN1 always high; IN2 switches between 0 (forward) and 1 (brake).
        # Forward only while IN2 is low: 25% forward -> IN2 high 75% of the time.
        lmot_in1.duty_cycle = 65535 # TODO set to 25% speed forward should be an int
        lmot_in2.duty_cycle = 49151 # TODO set to 25% speed forward should be an int  (0.75 * 65535)

# =============================================================================
# PART 3: Motor Control w/ Library
# =============================================================================
if PART == 3:
    import pwmio
    import adafruit_motor.motor as motor

    # TODO don't forget to make PWMOut objects first
    lmot_in1 = pwmio.PWMOut(board.GP16, frequency=20000)
    lmot_in2 = pwmio.PWMOut(board.GP17, frequency=20000)
    rmot_in1 = pwmio.PWMOut(board.GP15, frequency=20000)
    rmot_in2 = pwmio.PWMOut(board.GP14, frequency=20000)

    lmot = motor.DCMotor(lmot_in1, lmot_in2) # TODO create DCMotor object on pins GP16 and GP17
    rmot = motor.DCMotor(rmot_in1, rmot_in2) # TODO create DCMotor object on pins GP15 and GP14

    # TODO set lmot and rmot to SLOW_DECAY
    lmot.decay_mode = motor.SLOW_DECAY
    rmot.decay_mode = motor.SLOW_DECAY

    # throttle: 1.0 = full forward, 0 = brake, -1.0 = full reverse
    while True:
        # TODO move full speed forward for 1s
        lmot.throttle, rmot.throttle = 1, 1
        time.sleep(1)

        # TODO brake for 1s
        lmot.throttle, rmot.throttle = 0, 0
        time.sleep(1)

        # TODO move 25% speed backward for 1s
        lmot.throttle, rmot.throttle = -0.25, -0.25
        time.sleep(1)

        # TODO brake for 1s
        lmot.throttle, rmot.throttle = 0, 0
        time.sleep(1)
