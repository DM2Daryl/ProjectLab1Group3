"""
minimu9.py  –  MicroPython driver for the Pololu MinIMU-9 v6
Sensors: LSM6DSO (accel + gyro)  |  LIS3MDL (magnetometer)

Dead-reckoning position tracker:
  • Start the rover at a known point (default 0, 0, 0).
  • Call imu.update() in a tight loop; it integrates acceleration
    twice to produce X/Y/Z displacement in metres.
  • Heading is computed from the magnetometer (yaw, degrees).

Wiring (I²C):
  SDA  →  board SDA
  SCL  →  board SCL
  VIN  →  3.3 V or 5 V   (level-shifters on board handle it)
  GND  →  GND
  SA0  leave floating (pulled HIGH on board) for default addresses

Default I²C addresses (SA0 high / default):
  LSM6DSO  0x6B  (binary 1101011)
  LIS3MDL  0x1E  (binary 0011110)

Usage example
-------------
from machine import I2C, Pin
from minimu9 import MinIMU9

i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=400_000)
imu = MinIMU9(i2c)

while True:
    imu.update()
    x, y, z = imu.position          # metres from start
    heading  = imu.heading_deg      # compass degrees (0 = North)
    ax, ay, az = imu.accel          # raw accel  (m/s²)
    gx, gy, gz = imu.gyro           # raw gyro   (°/s)
    mx, my, mz = imu.mag            # raw mag    (gauss)
    print(f"pos ({x:.3f}, {y:.3f}, {z:.3f}) m  heading {heading:.1f}°")
"""

import time
import math
import struct


# ── LSM6DSO register addresses ────────────────────────────────────────────────
_LSM6_WHO_AM_I   = 0x0F   # expected: 0x6C
_LSM6_CTRL1_XL   = 0x10   # accelerometer control
_LSM6_CTRL2_G    = 0x11   # gyroscope control
_LSM6_CTRL3_C    = 0x12   # common control (IF_INC lives here)
_LSM6_OUTX_L_G   = 0x22   # gyro X low byte  (6 bytes: X, Y, Z)
_LSM6_OUTX_L_A   = 0x28   # accel X low byte (6 bytes: X, Y, Z)

# ── LIS3MDL register addresses ────────────────────────────────────────────────
_MAG_WHO_AM_I    = 0x0F   # expected: 0x3D
_MAG_CTRL_REG1   = 0x20   # performance / ODR
_MAG_CTRL_REG2   = 0x21   # full-scale
_MAG_CTRL_REG3   = 0x22   # operating mode
_MAG_CTRL_REG4   = 0x23   # Z-axis performance
_MAG_OUT_X_L     = 0x28   # X low byte (6 bytes: X, Y, Z)

# LIS3MDL: set MSB of sub-address to enable auto-increment over I²C
_MAG_AUTO_INC    = 0x80

# ── Default I²C slave addresses ───────────────────────────────────────────────
_LSM6_ADDR_DEFAULT = 0x6B   # SA0 high
_LSM6_ADDR_SA0_LOW = 0x6A
_MAG_ADDR_DEFAULT  = 0x1E   # SA1 high
_MAG_ADDR_SA1_LOW  = 0x1C


class MinIMU9:
    """
    Driver for Pololu MinIMU-9 v6.

    Parameters
    ----------
    i2c         : machine.I2C instance (already initialised)
    sa0_high    : True  → use default addresses (SA0 pin floating/high)
                  False → use alternate addresses (SA0 driven low)
    accel_range : ±g full-scale for accelerometer. Options: 2, 4, 8, 16
    gyro_range  : ±dps full-scale for gyroscope.   Options: 125, 250, 500, 1000, 2000
    mag_range   : ±gauss full-scale for magnetometer. Options: 4, 8, 12, 16
    """

    def __init__(
        self,
        i2c,
        sa0_high=True,
        accel_range=4,
        gyro_range=500,
        mag_range=4,
        calibration_samples=1000, #original 1000
        still_accel_threshold=0.15,
        still_gyro_threshold=1.5,
    ):
        """
        Extra parameters
        ----------------
        calibration_samples   : number of samples averaged on startup to measure
                                sensor bias (keep the board perfectly still during
                                the first ~2 seconds after creating this object).
        still_accel_threshold : m/s² — if the net horizontal acceleration is below
                                this, the rover is considered stationary and velocity
                                is zeroed.  Raise if you get false-still detections
                                while moving slowly; lower to reduce drift when parked.
        still_gyro_threshold  : °/s — gyro magnitude must also be below this for a
                                stillness detection.
        """
        self._i2c = i2c

        # I²C addresses
        self._lsm_addr = _LSM6_ADDR_DEFAULT if sa0_high else _LSM6_ADDR_SA0_LOW
        self._mag_addr = _MAG_ADDR_DEFAULT  if sa0_high else _MAG_ADDR_SA1_LOW

        # Raw sensor outputs (SI units)
        self.accel = (0.0, 0.0, 0.0)   # m/s²
        self.gyro  = (0.0, 0.0, 0.0)   # °/s
        self.mag   = (0.0, 0.0, 0.0)   # gauss

        # Integration state
        self._vel  = [0.0, 0.0, 0.0]   # m/s
        self._pos  = [0.0, 0.0, 0.0]   # m  (relative to start)
        self._last_t = None             # timestamp of last update (µs)

        # Heading (degrees, 0 = magnetic north)
        self.heading_deg = 0.0

        # Stillness detection thresholds
        self._still_accel = still_accel_threshold
        self._still_gyro  = still_gyro_threshold

        # Bias offsets (measured during calibration)
        self._accel_bias = [0.0, 0.0, 0.0]
        self._gyro_bias  = [0.0, 0.0, 0.0]

        # Whether the rover was still on the last update (exposed for debugging)
        self.is_still = False

        # Initialise hardware
        self._init_lsm6(accel_range, gyro_range)
        self._init_lis3mdl(mag_range)

        # Measure sensor bias with the board sitting still
        self._calibrate(calibration_samples)

    # ── Low-level I²C helpers ─────────────────────────────────────────────────

    def _write_reg(self, addr, reg, value):
        self._i2c.writeto(addr, bytes([reg, value]))

    def _read_reg(self, addr, reg):
        # Try repeated-start (no STOP between write and read).
        # Fall back to full STOP/START if the board doesn't support it.
        try:
            self._i2c.writeto(addr, bytes([reg]), False)
            return self._i2c.readfrom(addr, 1)[0]
        except (OSError, TypeError):
            self._i2c.writeto(addr, bytes([reg]))
            return self._i2c.readfrom(addr, 1)[0]

    def _read_bytes(self, addr, reg, length):
        try:
            self._i2c.writeto(addr, bytes([reg]), False)
            return self._i2c.readfrom(addr, length)
        except (OSError, TypeError):
            self._i2c.writeto(addr, bytes([reg]))
            return self._i2c.readfrom(addr, length)

    # ── LSM6DSO initialisation ────────────────────────────────────────────────

    def _init_lsm6(self, accel_range, gyro_range):
        """Wake up and configure the LSM6DSO accelerometer + gyroscope."""

        who = self._read_reg(self._lsm_addr, _LSM6_WHO_AM_I)
        if who != 0x6C:
            raise RuntimeError(
                f"LSM6DSO not found at 0x{self._lsm_addr:02X} "
                f"(WHO_AM_I=0x{who:02X}, expected 0x6C)"
            )

        # CTRL3_C: BDU=1 (block data update), IF_INC=1 (auto-increment, default on)
        self._write_reg(self._lsm_addr, _LSM6_CTRL3_C, 0x44)

        # ── Accelerometer: CTRL1_XL ──────────────────────────────────────────
        # ODR = 104 Hz (normal), full-scale selectable
        # Bits[7:4] ODR_XL  = 0100 → 104 Hz
        # Bits[3:2] FS_XL   = see below
        # Bit[1]    LPF2_XL = 0
        _accel_fs = {2: 0b00, 4: 0b10, 8: 0b11, 16: 0b01}
        if accel_range not in _accel_fs:
            raise ValueError("accel_range must be 2, 4, 8, or 16 (g)")
        fs_bits = _accel_fs[accel_range]
        ctrl1 = (0b0100 << 4) | (fs_bits << 2)
        self._write_reg(self._lsm_addr, _LSM6_CTRL1_XL, ctrl1)

        # Sensitivity in m/s² per LSB
        # Full-scale ±Ng spans 2N g → 2N * 9.80665 / 65536 m/s² per LSB
        self._accel_sens = accel_range * 9.80665 / 32768.0

        # ── Gyroscope: CTRL2_G ───────────────────────────────────────────────
        # ODR = 104 Hz, full-scale selectable
        # Bits[7:4] ODR_G = 0100 → 104 Hz
        # Bits[3:1] FS_G  = see below  (bit 0 = 125 dps flag)
        _gyro_fs = {
            125:  (0b000, 1),
            250:  (0b000, 0),
            500:  (0b010, 0),
            1000: (0b100, 0),
            2000: (0b110, 0),
        }
        if gyro_range not in _gyro_fs:
            raise ValueError("gyro_range must be 125, 250, 500, 1000, or 2000 (dps)")
        fs_g, fs_125 = _gyro_fs[gyro_range]
        ctrl2 = (0b0100 << 4) | (fs_g << 1) | fs_125
        self._write_reg(self._lsm_addr, _LSM6_CTRL2_G, ctrl2)

        # Sensitivity in °/s per LSB
        self._gyro_sens = gyro_range / 32768.0

    # ── LIS3MDL initialisation ────────────────────────────────────────────────

    def _init_lis3mdl(self, mag_range):
        """Wake up and configure the LIS3MDL magnetometer."""

        who = self._read_reg(self._mag_addr, _MAG_WHO_AM_I)
        if who != 0x3D:
            raise RuntimeError(
                f"LIS3MDL not found at 0x{self._mag_addr:02X} "
                f"(WHO_AM_I=0x{who:02X}, expected 0x3D)"
            )

        # CTRL_REG1: TEMP_EN=0, OM=11 (ultra-high), ODR=100 (10 Hz), FAST_ODR=0, ST=0
        # OM[1:0] at bits [6:5], ODR[2:0] at bits [4:2]
        # 0b01110000 = TEMP_EN=0, OM=11, DO=100 (10 Hz)
        self._write_reg(self._mag_addr, _MAG_CTRL_REG1, 0b01110000)

        # CTRL_REG2: full-scale
        _mag_fs = {4: 0b00, 8: 0b01, 12: 0b10, 16: 0b11}
        if mag_range not in _mag_fs:
            raise ValueError("mag_range must be 4, 8, 12, or 16 (gauss)")
        fs_bits = _mag_fs[mag_range]
        self._write_reg(self._mag_addr, _MAG_CTRL_REG2, fs_bits << 5)

        # Sensitivity in gauss per LSB (from datasheet Table 3)
        _mag_sens = {4: 1/6842, 8: 1/3421, 12: 1/2281, 16: 1/1711}
        self._mag_sens = _mag_sens[mag_range]

        # CTRL_REG3: continuous-conversion mode (MD[1:0] = 00)
        self._write_reg(self._mag_addr, _MAG_CTRL_REG3, 0x00)

        # CTRL_REG4: Z-axis ultra-high performance (OMZ=11), little-endian
        self._write_reg(self._mag_addr, _MAG_CTRL_REG4, 0b00001100)

    # ── Read raw sensor data ──────────────────────────────────────────────────

    def _read_lsm6(self):
        """Return accel (m/s²) and gyro (°/s) as tuples."""

        # Gyro: 6 bytes starting at OUTX_L_G (auto-increment enabled by default)
        raw_g = self._read_bytes(self._lsm_addr, _LSM6_OUTX_L_G, 6)
        gx, gy, gz = struct.unpack_from("<hhh", raw_g)

        # Accel: 6 bytes starting at OUTX_L_A
        raw_a = self._read_bytes(self._lsm_addr, _LSM6_OUTX_L_A, 6)
        ax, ay, az = struct.unpack_from("<hhh", raw_a)

        return (
            (ax * self._accel_sens, ay * self._accel_sens, az * self._accel_sens),
            (gx * self._gyro_sens,  gy * self._gyro_sens,  gz * self._gyro_sens),
        )

    def _read_lis3mdl(self):
        """Return magnetometer reading as tuple (gauss)."""

        # Set MSB of register address to enable auto-increment on LIS3MDL
        raw = self._read_bytes(self._mag_addr, _MAG_OUT_X_L | _MAG_AUTO_INC, 6)
        mx, my, mz = struct.unpack_from("<hhh", raw)

        return (mx * self._mag_sens, my * self._mag_sens, mz * self._mag_sens)

    # ── Main update call ──────────────────────────────────────────────────────

    def _calibrate(self, n_samples):
        """
        Average n_samples readings with the board perfectly still to measure
        the accelerometer and gyro zero-rate offsets (bias).
        Called automatically from __init__ — keep the board still for ~2 s.
        """
        print(f"IMU calibrating ({n_samples} samples) — keep board still...")
        ax_sum = ay_sum = az_sum = 0.0
        gx_sum = gy_sum = gz_sum = 0.0
        for _ in range(n_samples):
            (ax, ay, az), (gx, gy, gz) = self._read_lsm6()
            ax_sum += ax;  ay_sum += ay;  az_sum += az
            gx_sum += gx;  gy_sum += gy;  gz_sum += gz
            time.sleep_ms(5)
        self._accel_bias[0] = ax_sum / n_samples
        self._accel_bias[1] = ay_sum / n_samples
        # Z bias: sensor reads ~+9.81 when flat. Remove only the offset above gravity.
        self._accel_bias[2] = (az_sum / n_samples) - 9.80665
        self._gyro_bias[0]  = gx_sum / n_samples
        self._gyro_bias[1]  = gy_sum / n_samples
        self._gyro_bias[2]  = gz_sum / n_samples
        print(f"  Accel bias: ({self._accel_bias[0]:+.4f}, {self._accel_bias[1]:+.4f}, {self._accel_bias[2]:+.4f}) m/s²")
        print(f"  Gyro  bias: ({self._gyro_bias[0]:+.4f}, {self._gyro_bias[1]:+.4f}, {self._gyro_bias[2]:+.4f}) °/s")
        print("  Calibration done.")

    def update(self):
        """
        Read all sensors, update position estimate and heading.
        Call this as frequently as possible (ideally in a tight loop).

        After calling update():
          imu.accel        → bias-corrected accel  (m/s²)
          imu.gyro         → bias-corrected gyro   (°/s)
          imu.mag          → magnetometer reading  (gauss)
          imu.heading_deg  → compass heading in degrees (0 = magnetic north)
          imu.position     → (x, y, z) in metres from origin
          imu.is_still     → True if rover is detected as stationary
        """

        now = time.ticks_us()

        raw_accel, raw_gyro = self._read_lsm6()
        self.mag = self._read_lis3mdl()

        # ── Apply bias correction ─────────────────────────────────────────────
        ax = raw_accel[0] - self._accel_bias[0]
        ay = raw_accel[1] - self._accel_bias[1]
        az = raw_accel[2] - self._accel_bias[2]
        gx = raw_gyro[0]  - self._gyro_bias[0]
        gy = raw_gyro[1]  - self._gyro_bias[1]
        gz = raw_gyro[2]  - self._gyro_bias[2]
        self.accel = (ax, ay, az)
        self.gyro  = (gx, gy, gz)

        # ── Heading from magnetometer (yaw only, flat surface assumed) ────────
        mx, my, _ = self.mag
        heading_rad = math.atan2(-my, mx)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360.0
        self.heading_deg = heading_deg

        # ── Dead-reckoning position integration ──────────────────────────────
        if self._last_t is not None:
            dt = time.ticks_diff(now, self._last_t) / 1_000_000.0  # seconds

            # Gravity is already removed via the Z bias above.
            # Use only horizontal (X/Y) acceleration for stillness check —
            # Z always has some residual gravity noise.
            horiz_accel = math.sqrt(ax * ax + ay * ay)
            gyro_mag    = math.sqrt(gx * gx + gy * gy + gz * gz)

            self.is_still = (horiz_accel < self._still_accel and
                             gyro_mag    < self._still_gyro)

            if self.is_still:
                # Zero-velocity update: if we know we're not moving,
                # reset velocity so bias can't accumulate further.
                self._vel = [0.0, 0.0, 0.0]
            else:
                # Integrate acceleration → velocity → position
                az_for_integration = az - 9.80665  # remove gravity for Z
                for i, a in enumerate((ax, ay, az_for_integration)):
                    self._vel[i] += a * dt
                    self._pos[i] += self._vel[i] * dt

        self._last_t = now

    @property
    def position(self):
        """Current estimated position (x, y, z) in metres from origin."""
        return tuple(self._pos)

    def reset_position(self, x=0.0, y=0.0, z=0.0):
        """
        Reset the dead-reckoning origin.
        Call this when the rover is placed at a known point.
        """
        self._pos = [x, y, z]
        self._vel = [0.0, 0.0, 0.0]
        self._last_t = None

    # ── Convenience: pretty-print one line of readings ────────────────────────

    def __str__(self):
        ax, ay, az = self.accel
        gx, gy, gz = self.gyro
        mx, my, mz = self.mag
        px, py, pz = self.position
        return (
            f"Accel  ({ax:+7.3f}, {ay:+7.3f}, {az:+7.3f}) m/s²\n"
            f"Gyro   ({gx:+8.2f}, {gy:+8.2f}, {gz:+8.2f}) °/s\n"
            f"Mag    ({mx:+6.4f}, {my:+6.4f}, {mz:+6.4f}) gauss\n"
            f"Heading {self.heading_deg:6.1f}°\n"
            f"Pos    ({px:+7.3f}, {py:+7.3f}, {pz:+7.3f}) m"
        )


def diagnose_i2c(i2c):
    """
    Run this BEFORE creating a MinIMU9 object to pinpoint I2C issues.
    Usage:
        from minimu9 import diagnose_i2c
        from machine import I2C, Pin
        i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
        diagnose_i2c(i2c)
    """
    LSM6_ADDR = 0x6B
    MAG_ADDR  = 0x1E
    WHO_AM_I  = 0x0F

    print("=== MinIMU-9 v6 I2C Diagnostics ===")

    # 1. Bus scan
    found = i2c.scan()
    print(f"Bus scan: {[hex(d) for d in found]}")
    for addr, name in [(LSM6_ADDR, 'LSM6DSO'), (MAG_ADDR, 'LIS3MDL')]:
        print(f"  {name} (0x{addr:02X}): {'FOUND' if addr in found else 'MISSING'}")

    # 2. Try each read strategy
    strategies = [
        ("writeto(stop=False) + readfrom",
         lambda i, a, r: (i.writeto(a, bytes([r]), False), i.readfrom(a, 1))[1][0]),
        ("writeto(stop=True)  + readfrom",
         lambda i, a, r: (i.writeto(a, bytes([r])),        i.readfrom(a, 1))[1][0]),
        ("readfrom_mem",
         lambda i, a, r: i.readfrom_mem(a, r, 1)[0]),
    ]

    for addr, name in [(LSM6_ADDR, 'LSM6DSO'), (MAG_ADDR, 'LIS3MDL')]:
        print(f"\n--- {name} (0x{addr:02X}) ---")
        for label, fn in strategies:
            try:
                val = fn(i2c, addr, WHO_AM_I)
                print(f"  [{label}]  WHO_AM_I = 0x{val:02X}  {'OK' if val in (0x6C, 0x3D) else 'UNEXPECTED'}")
            except Exception as e:
                print(f"  [{label}]  FAILED: {e}")

    print("\n=== Done ===")

