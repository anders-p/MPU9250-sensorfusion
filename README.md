# MPU9250-sensorfusion
Fusion algorithms for an MPU9250 9DOF accelerometer, gyroscope and magnetometer. This library uses a Kalman filter to combine data from an accelerometer, gyroscope and magnetometer to determine the 3-axis attitude of the sensor (the roll, pitch and yaw). It was written for the MPU9250, but the filter could be adapted for any fusion algorithm. There are also classes for 1-dimensional and n-dimensional Kalman filters which can be used for general purpose applications.

Written in micro-python for use with the LoPy Expansion 3.0 board.

# Calibration
The sensor generally needs to be calibrated before use. The accelerometer is usually offset by a constant value, which can be calculated and compensated for easily. The magnetometer has a calibration routine written by Mika Tuupola and provided with the MPU9250 library. The gyroscope calibration currently calculates the offset while held completely still. Possible scaling may also be required and added at a later date.

The modules for communicating directly with the sensor were written by Mika Tuupola under the MIT License.
Project home: https://github.com/tuupola/micropython-mpu9250
