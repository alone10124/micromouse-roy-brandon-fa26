"""
Lab 4: IR Sensors
=================

Copy this file to the CIRCUITPY drive as `code.py`, along with `irsensor.py`
from labs/sanity/ (needed for PARTs 2-4). Set PART below:

  PART = 1  -> Read one sensor pair by hand (left a/b)        [Checkoff #1]
  PART = 2  -> Read all sensors with the IRSensors library    [Checkoff #2]
  PART = 3  -> Calibration (least squares fit, labs/lab4/code.py)    [Checkoff #3.1]
  PART = 4  -> Distance readings (labs/lab4/distance.py)             [Checkoff #3.2]

How it works: 6 IR detectors share 3 ADC pins (2 per pin). Each detector
has an enable pin in open-drain mode. Driving it LOW connects that detector
to ground (selects it); setting it HIGH leaves it high-Z (disconnected).

Checkoff answers
  #1: Show one working sensor pair (PART 1).
  #2: Show all six sensors responding (PART 2).
  #3: Run PART 3, save the 12 constants it prints, paste them into the
      Distance(...) call in PART 4, and check that the distances look right.
      Typical values are around a = 0.03 and b = -50.
"""

PART = 1  # <-- change to 1, 2, 3, or 4

import board
import time

# =============================================================================
# PART 1: Read One Sensor
# =============================================================================
if PART == 1:
    import digitalio
    from analogio import AnalogIn

    # adc
    l_adc = AnalogIn(board.GP28) # TODO call AnalogIn on GP28

    # emitter
    l_en = digitalio.DigitalInOut(board.GP7) # TODO create DigitalInOut output on GP7
    l_en.direction = digitalio.Direction.OUTPUT
    l_en.value = False

    # sensors
    lir_a = digitalio.DigitalInOut(board.GP5)
    lir_a.direction = digitalio.Direction.OUTPUT
    lir_a.drive_mode = digitalio.DriveMode.OPEN_DRAIN
    lir_a.value = True # high Z mode

    lir_b = digitalio.DigitalInOut(board.GP6) # TODO create DigitalInOut on GP6 in open-drain mode, use lir_a as direct inspiration
    lir_b.direction  = digitalio.Direction.OUTPUT
    lir_b.drive_mode = digitalio.DriveMode.OPEN_DRAIN
    lir_b.value = True # high Z mode

    while True:
        # TODO enable IR emitters using l_en set one of its variables to a (boolean)
        l_en.value = True

        # TODO enable chosen sensor lir_a or lir_b by setting one of its variable to false
        lir_a.value = False          # connect lir_a to ground -> it drives the ADC
        # TODO wait a bit, 1ms should do time. ...
        time.sleep(0.001)            # let the voltage settle
        # TODO take analog reading for future printing print(l_adc. , end=" ")
        print(l_adc.value, end=" ")  # 16-bit reading (0-65535)
        # TODO disable chosen sensor lir_a or lir_b
        lir_a.value = True           # back to high-Z

        # TODO try for lir_b or lir_a whichever one you didnt use instead, enable chosen sensor etc
        # Repeat above for this sensor
        lir_b.value = False
        time.sleep(0.001)
        print(l_adc.value)
        lir_b.value = True

        # TODO disable IR emitters using l_en
        l_en.value = False           # emitters draw ~50mA each, so turn them off
        time.sleep(0.05)

# =============================================================================
# PART 2: Read All the Sensors
# =============================================================================
if PART == 2:
    from irsensor import IRSensors

    # Per side: emitter enable, sensor a enable, sensor b enable, ADC pin.
    # ir.scan() runs the same enable -> select -> read -> deselect -> disable
    # steps as PART 1, for all three pairs at once, averaging `avg` samples.
    ir = IRSensors(
        board.GP7,  board.GP5,  board.GP6,  board.GP28, # left
        board.GP9,  board.GP10, board.GP11, board.GP26, # center
        board.GP21, board.GP20, board.GP22, board.GP27  # right
    )

    while True:
        ir.scan()
        print("lir_a:", ir.lir_a, "\t", "lir_b:", ir.lir_b, "\t",
                "cir_a:", ir.cir_a, "\t", "cir_b:", ir.cir_b, "\t",
                "rir_a:", ir.rir_a, "\t", "rir_b:", ir.rir_b)
        time.sleep(0.05)

# =============================================================================
# PART 3: Calibration (provided code: labs/lab4/code.py)
# =============================================================================
# Place a maze wall at 0, 10, ..., 60 mm from each sensor pair and press
# <enter> in the serial monitor at each step. Least squares then fits
# distance = a * sensor + b for each sensor.
if PART == 3:
    import ulab.numpy as np

    from irsensor import IRSensors

    """ Peripherals """

    # IR sensors
    ir = IRSensors(
        board.GP7,  board.GP5,  board.GP6,  board.GP28, # left
        board.GP9,  board.GP10, board.GP11, board.GP26, # center
        board.GP21, board.GP20, board.GP22, board.GP27  # right
    )

    """ Main """

    MAX_DIST = 60 # mm
    STEP = 10 # mm

    # Normal-equation least squares: x = (A^T A)^-1 A^T b
    def lstsq(A, b):
        At = A.transpose()
        return np.dot(np.dot(np.linalg.inv(np.dot(At, A)), At), b)

    def calibrate(a, b):
        # x_* = raw sensor readings, d_* = true distances (mm)
        x_a, d_a, x_b, d_b = [], [], [], []
        for d in np.arange(0, MAX_DIST+1e-3, STEP):
            print("Place wall at", d, "\bmm and press <enter> to record", end="")
            input()
            ir.scan()
            d_a.append(d); d_b.append(d)
            x_a.append(getattr(ir, a)); x_b.append(getattr(ir, b))

        # Build A = [sensor, 1] so that A @ [a, b] ~= distance
        A_a = np.concatenate((
            np.array(x_a).reshape((len(x_a),1)),
            np.ones(len(x_a)).reshape((len(x_a),1))
        ), axis=1)
        d_a = np.array(d_a)

        A_b = np.concatenate((
            np.array(x_b).reshape((len(x_b),1)),
            np.ones(len(x_b)).reshape((len(x_b),1))
        ), axis=1)
        d_b = np.array(d_b)

        return lstsq(A_a, d_a), lstsq(A_b, d_b)

    if __name__ == "__main__":
        print("Calibrating left sensors...")
        k_a, k_b = calibrate("lir_a", "lir_b")
        print()
        print("lir_a constants:", "a =", k_a[0], "\tb =", k_a[1])
        print("lir_b constants:", "a =", k_b[0], "\tb =", k_b[1])
        print()

        print("Calibrating center sensors...")
        k_a, k_b = calibrate("cir_a", "cir_b")
        print()
        print("cir_a constants:", "a =", k_a[0], "\tb =", k_a[1])
        print("cir_b constants:", "a =", k_b[0], "\tb =", k_b[1])
        print()

        print("Calibrating right sensors...")
        k_a, k_b = calibrate("rir_a", "rir_b")
        print()
        print("rir_a constants:", "a =", k_a[0], "\tb =", k_a[1])
        print("rir_b constants:", "a =", k_b[0], "\tb =", k_b[1])
        print()

# =============================================================================
# PART 4: Distance (provided code: labs/lab4/distance.py)
# =============================================================================
if PART == 4:
    from irsensor import IRSensors

    """ Extra Code """
    # Converts raw IR readings to mm using distance = a * sensor + b
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

    """ Peripherals """

    # IR sensors
    ir = IRSensors(
        board.GP7,  board.GP5,  board.GP6,  board.GP28, # left
        board.GP9,  board.GP10, board.GP11, board.GP26, # center
        board.GP21, board.GP20, board.GP22, board.GP27, # right
        avg = 10
    )

    """ Main """

    ''' Please ensure you change these to your own, these are for another mouse'''
    # TODO: replace these placeholder constants with the ones PART 3 printed
    #       for YOUR mouse (lir_a a/b, lir_b a/b, cir_a a/b, ...).
    dist = Distance(ir,
        0.0299, -63.6, 0.0195, -39.1,
        0.0300, -60.4, 0.0251, -47.9,
        0.0258, -50.5, 0.0292, -54.0,
    )

    """Make Sure to allign the constants in the right place:
            la_a, la_b, lb_a, lb_b,
            ca_a, ca_b, cb_a, cb_b,
            ra_a, ra_b, rb_a, rb_b,
    """

    if __name__ == "__main__":
        while True:
            dist.scan()
            print("lir_a:", dist.la, "\t", "lir_b:", dist.lb, "\t",
                    "cir_a:", dist.ca, "\t", "cir_b:", dist.cb, "\t",
                    "rir_a:", dist.ra, "\t", "rir_b:", dist.rb)
            time.sleep(0.05)
