
"""
Library to fuse values from an MPU9250 9 DOF accelerometer, magnetometer and gyroscope
to produce useful outputs
"""

from mpu9250 import MPU9250
from iir import iir
import math

# Constant Definitions
RS2DS = 57.295779578552 # 1 rad/s = 57.295779578552 deg/s

class tilt:
    """ Class which gives the roll, pitch and yaw of the sensor"""
    def __init__(self, i2c, ak8963=None):
        # Initialise the sensor
        self.sensor = MPU9250(i2c, ak8963=ak8963)

        # Initialise the class variables
        self.G_roll = 0
        self.G_pitch = 0
        self.G_yaw = 0

        # Initialise the filter
        _a = [1.0000,   -1.8879,    0.8946] # Denominator coefficients
        # Gain
        _K = sum(_a) / 4
        _b = [1.0000*_K,    1.9701*_K,    0.9703*_K] # Numerator coefficients

        self.accel_filter_x = iir(_a, _b)
        self.accel_filter_y = iir(_a, _b)
        self.accel_filter_z = iir(_a, _b)

    """ Function to update the entire class, should be called each measurement cycle
    This function gets the raw values from the sensors, updates the filters, then updates
    the value for the class outputs"""
    def update(self):
        # Measure raw values
        self.measure()

        # Filter raw values
        self.filter()

    """ Function to get the current filtered accelerometer readings"""
    def get_accel(self):
        _Ax = self.accel_filter_x.get()
        _Ay = self.accel_filter_y.get()
        _Az = self.accel_filter_z.get()
        return _Ax, _Ay, _Az


    """ Calculate the roll and pitch using the accelerometer values"""
    def accel_rp(self):
        # roll = atan(y / z)
        self.rad_roll = math.atan2(self.accel[1], self.accel[2])

        # pitch = atan(-x / sqrt(y^2 + z^2))
        self.rad_pitch = math.atan2(-self.accel[0], math.sqrt(self.accel[1]*self.accel[1] + self.accel[2]*self.accel[2]))

    """ Calculate yaw using the roll, pitch and magnetometer readings"""
    # NOTE: This function must be called AFTER the accel_rp function in order to have updated roll and pitch values
    def yaw(self):
        # Combine roll and pitch to adjust the magnetometer readings for the tilt
        # Mx = mx * cos(rad_pitch) + mz * sin(rad_pitch)
        Mx = self.mag[0] * math.cos(self.rad_pitch) + self.mag[2] * math.sin(self.rad_pitch)

        # My = mx * sin(rad_roll) * sin(rad_pitch) + my * cos(rad_roll) - mz * sin(rad_roll) * cos(rad_pitch)
        My = self.mag[0] * math.sin(self.rad_roll) * math.sin(self.rad_pitch) + self.mag[1] * math.cos(self.rad_roll) - self.mag[2] * math.sin(self.rad_roll) * math.cos(self.rad_pitch)

        # yaw = atan(My / Mx)
        self.rad_yaw = math.atan2(-My, Mx)

    """ Calculate roll, pitch and yaw using gyroscope readings"""
    def gyro_vals(self, dt):
        # dt - Change in time since the last measurement
        # Gives the values in radians

        _temp = self.gyro[0] * RS2DS + 60

        # roll = Gx * delta_t
        self.G_roll += _temp * dt
        self.G_pitch += self.gyro[1] * dt
        self.G_yaw += self.gyro[2] * dt

    """ Function to get the raw measurements from the sensor"""
    def measure(self):
        # Doesn't return any values, but sets the class variables to the current values from the sensor
        self.accel = self.sensor.acceleration # (ax, ay, az)
        self.gyro = self.sensor.gyro # (Gx, Gy, Gz)
        self.mag = self.sensor.magnetic # (mx, my, mz)

    """ Function to filter the measured values"""
    def filter(self):
        # Update the filters with the new measurements
        self.accel_filter_x.update(self.accel[0])
        self.accel_filter_y.update(self.accel[1])
        self.accel_filter_z.update(self.accel[2])
