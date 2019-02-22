
"""
Library to fuse values from an MPU9250 9 DOF accelerometer, magnetometer and gyroscope
to produce useful outputs
"""

from mpu9250 import MPU9250
from iir import iir
from kalman import eulerKalman
import matrixfunctions as matrix
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

        # Offset values for the gyroscope output
        self._gyroOffset = (-83.53642,78.94066,-69.39014)
        self._gyroScale = (0.1, 0.1, 0.1)

        # Initialise the filters for the accelerometer and gyroscope
        _a = [1.0000,   -1.8879,    0.8946] # Denominator coefficients
        # Gain
        _K = sum(_a) / 4
        _b = [1.0000*_K,    1.9701*_K,    0.9703*_K] # Numerator coefficients

        self.accel_filter_x = iir(_a, _b)
        self.accel_filter_y = iir(_a, _b)
        self.accel_filter_z = iir(_a, _b)

        self.mag_filter_x = iir(_a, _b)
        self.mag_filter_y = iir(_a, _b)
        self.mag_filter_z = iir(_a, _b)

        self.gyro_filter_x = iir(_a, _b)
        self.gyro_filter_y = iir(_a, _b)
        self.gyro_filter_z = iir(_a, _b)

        # Initialise Kalman filter matrices
        _q = 0.0001 # Process noise
        _r = 10 # Signal noise
        _Q = matrix.matrix(4, 4, data=[[_q, 0, 0, 0],
        [0, _q, 0, 0],
        [0, 0, _q, 0],
        [0, 0, 0, _q]])
        _R = matrix.matrix(4, 4, data=[[_r, 0, 0, 0],
        [0, _r, 0, 0],
        [0, 0, _r, 0],
        [0, 0, 0, _r]])

        # Create the Kalman filter
        self.kalman_filter = eulerKalman(_Q, _R)

    """ Function to update the entire class, should be called each measurement cycle
    This function gets the raw values from the sensors, updates the filters, then updates
    the value for the class outputs"""
    def update(self):
        # Measure raw values
        self.measure()

        # Convert gyroscope values to quaternion matrix
        (_p, _q, _r) = self.filter_gyro()

        _p = _p * 0.01 / 2
        _q = _q * 0.01 / 2
        _r = _r * 0.01 / 2

        _A = matrix.matrix(4, 4, data=[[1, -_p, -_q, -_r],
        [_p, 1, _r, -_q],
        [_q, -_r, 1, _p],
        [_r, _q, -_p, 1]])

        # Filter raw accelerometer values
        (_Ax, _Ay, _Az) = self.filter_accel()

        # Filter raw magnetometer values
        (_Mx, _My, _Mz) = self.filter_mag()

        # Calculate roll, pitch and yaw
        (_roll, _pitch, _yaw) = self.attitude(_Ax, _Ay, _Az, _Mx, _My, _Mz)

        # Convert the filtered values to quaternion
        _z = self.quatern(_roll, _pitch, _yaw)

        # Filter the values
        self.kalman_filter.update(_z, _A)

        # Return the current filtered values
        return self.kalman_filter.get()

    """ Function to get the current filtered accelerometer readings"""
    def get_accel(self):
        _Ax = self.accel_filter_x.get()
        _Ay = self.accel_filter_y.get()
        _Az = self.accel_filter_z.get()
        return _Ax, _Ay, _Az

    def get_mag(self):
        _Mx = self.mag_filter_x.get()
        _My = self.mag_filter_y.get()
        _Mz = self.mag_filter_z.get()
        return _Mx, _My, _Mz

    def get_gyro(self):
        _Gx = self.gyro_filter_x.get()
        _Gy = self.gyro_filter_y.get()
        _Gz = self.gyro_filter_z.get()
        return _Gx, _Gy, _Gz

    """ Calculate the roll, pitch and yaw using accelerometer and magnetometer values"""
    def attitude(self, Ax, Ay, Az, Mx, My, Mz):
        # roll = atan(y / z)
        _roll = math.atan2(Ay, Az)

        # pitch = atan(-x / sqrt(y^2 + z^2))
        _pitch = math.atan2(-Ax, math.sqrt(Ay*Ay + Az*Az))

        # Combine roll and pitch to adjust the magnetometer readings for the tilt
        # Mx = mx * cos(pitch) + mz * sin(pitch)
        _M1 = Mx * math.cos(_pitch) + Mz * math.sin(_pitch)

        # My = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
        _M2 = Mx * math.sin(_roll) * math.sin(_pitch) + My * math.cos(_roll) - Mz * math.sin(_roll) * math.cos(_pitch)

        # yaw = atan(M2 / M1)
        _yaw = math.atan2(-_M2, _M1)

        return _roll, _pitch, _yaw

    """ Calculate the quaternion values from the roll, pitch and yaw"""
    def quatern(self, roll=0, pitch=0, yaw=0):
        # Calculate intermediate values
        r1 = math.sin(roll / 2)
        r2 = math.cos(roll / 2)
        p1 = math.sin(pitch / 2)
        p2 = math.cos(pitch / 2)
        y1 = math.sin(yaw / 2)
        y2 = math.cos(yaw / 2)

        # z = [ cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi,
        # sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi,
        # cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi,
        # cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi]
        _data = [[r2*p2*y2 + r1*p1*y1],
        [r1*p2*y2 + r2*p1*y1],
        [r2*p1*y2 + r1*p2*y1],
        [r2*p2*y1 + r1*p1*y2]]

        _z = matrix.matrix(4, 1, _data)

        return _z

    """ Function to get the raw measurements from the sensor"""
    def measure(self):
        # Doesn't return any values, but sets the class variables to the current values from the sensor
        self.accel = self.sensor.acceleration # (ax, ay, az)
        self.gyro = self.sensor.gyro # (Gx, Gy, Gz)
        self.mag = self.sensor.magnetic # (mx, my, mz)

    """ Function to filter the measured values"""
    def filter_accel(self):
        # Update the filters with the new measurements
        _Ax = self.accel_filter_x.update(self.accel[0])
        _Ay = self.accel_filter_y.update(self.accel[1])
        _Az = self.accel_filter_z.update(self.accel[2])
        return _Ax, _Ay, _Az

    def filter_mag(self):
        _Mx = self.mag_filter_x.update(self.mag[0])
        _My = self.mag_filter_y.update(self.mag[1])
        _Mz = self.mag_filter_z.update(self.mag[2])
        return _Mx, _My, _Mz

    def filter_gyro(self):
        # Offset the measurements
        _valx = self.gyro[0] - self._gyroOffset[0]
        _valy = self.gyro[1] - self._gyroOffset[1]
        _valz = self.gyro[2] - self._gyroOffset[2]
        
        # Scale the measurements
        _valx *= self._gyroScale[0]
        _valy *= self._gyroScale[1]
        _valz *= self._gyroScale[2]

        # Update all the gyroscope filters
        _Gx = self.gyro_filter_x.update(_valx)
        _Gy = self.gyro_filter_y.update(_valy)
        _Gz = self.gyro_filter_z.update(_valz)
        
        return _Gx, _Gy, _Gz
