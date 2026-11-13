"""
Lab 3: Encoders
===============

Copy this file to the CIRCUITPY drive as `code.py`. Set PART below to the
section you want to run:

  PART = 1  -> Waveform Viewing (raw Hall sensor values)      [Checkoff #1]
  PART = 2  -> Polling (count rising edges of channel A)      [Checkoff #2]
  PART = 3  -> Interrupts (rotaryio.IncrementalEncoder)       [Checkoff #3]

-----------------------------------------------------------------------------
Checkoff #1 answers
  1. When stationary the output is constant: each channel sits at either 0
     or 1 (whichever pole is nearest), and it doesn't change.
  2. Spinning faster makes both channels toggle between 0 and 1 faster
     (higher frequency square waves).
  3. One sensor only tells you THAT the wheel moved, not WHICH WAY, since the
     waveform looks the same forwards and backwards. With two sensors 90° out
     of phase (quadrature), which channel leads the other gives the direction.

Checkoff #2 answers
  1. See PART 2 below.
  2. About 217 ticks per wheel revolution. That's the gearbox ratio times the
     number of north poles on the motor magnet (we count one rising edge of A
     per north pole that passes the sensor).
  3. Polling can miss edges if the wheel spins fast or the loop is slow, and
     it burns all of the CPU's time just checking the pin.
  4. Velocity = change in position / change in time. Either sample position
     at a fixed interval and take the difference, or use a timer to measure
     the time between edges.
  5. Swapping A and B just reverses the sign: forward counts down and
     backward counts up.

Checkoff #3 answers
  1. See PART 3 below. The position goes up and down as you spin the wheel.
  2. Interrupts run only when an edge actually happens, so they don't miss
     edges the way polling can and they leave the CPU free for other work
     (e.g. the control loop). There's still a top speed, set by the overhead
     of handling each interrupt.
"""

PART = 1  # <-- change to 1, 2, or 3

import board
import time
import digitalio

# =============================================================================
# PART 1: Waveform Viewing
# =============================================================================
if PART == 1:
    # a and b Hall effect sensors of left encoder
    a = digitalio.DigitalInOut(board.GP12)
    b = digitalio.DigitalInOut(board.GP13)

    # Print both channels as 0/1 so you can watch the quadrature pattern
    # (e.g. 00 -> 10 -> 11 -> 01 -> 00 in one direction, reversed in the other).
    while True:
        print(int(a.value), int(b.value))
        time.sleep(0.001)

# =============================================================================
# PART 2: Polling
# =============================================================================
if PART == 2:
    # a and b Hall effect sensors of left encoder
    a = digitalio.DigitalInOut(board.GP12)
    b = digitalio.DigitalInOut(board.GP13)

    counter = 0
    position = 0
    a_prev = a.value

    def leftEncoderRisingEdge():
        """TODO increment or decrement position depending on which way the motor is spinning
        Try experimenting with the instance variables of a and b (we only need either a or b). Refer to the
        waveform code from earlier for an idea, think about rising edges"""
        global position # access global position
        # When A has a rising edge, B's level tells us the direction:
        #   B high -> B is leading A -> one direction  (count +1)
        #   B low  -> A is leading B -> other direction (count -1)
        # One-line version: position += 1 if b.value else -1
        if b.value == True : # can be simplified lol
            position += 1
        else:
            position -= 1

    while True:
        a_val = a.value
        if a_val and not a_prev: # rising edge of a
            leftEncoderRisingEdge()
        a_prev = a_val

        # only print every 1000 iterations so printing doesn't slow polling
        if counter % 1000 == 0:
            print(position)
        counter += 1

# =============================================================================
# PART 3: Interrupts
# =============================================================================
if PART == 3:
    import rotaryio

    # rotaryio handles the edges with hardware/interrupts for us, so we just
    # read .position whenever we want it.
    enc = rotaryio.IncrementalEncoder(board.GP12, board.GP13)

    while True:
        print(enc.position)
        time.sleep(0.001)
