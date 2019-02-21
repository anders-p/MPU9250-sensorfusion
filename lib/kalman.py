### Implementing discrete Kalman filter ###
""" Function for implementing a discrete Kalman filter
A Kalman filter uses a probability distribution to predict the next value from a source,
such as a sensor. It then updates this probability distribution based on the value given
from the source, providing a smoothing effect in a similar way to a moving average filter.
It works in two stages, first prediction, where it guesses what the next value from the sensor
will be, then correction, where it uses the actual value from the source to correct its prediction.

It requires 4 main parameters to be determined externally to use:
    A - The 'prediction matrix', which determines the predicted state of the system, this can
    usually be approximated as 1 for one-dimensional systems, or an identity matrix for higher order
    Q - The covariance of the 'process noise', this is static and represents the idea
    of how quickly the system responds to external input
    R - The covariance of the 'signal noise', this is static and is a measure of how
    much noise is expected in the system
    z - The raw input from the source, usually a sensor

The classes provided are:
- 1-dimensional filter, which can be useful for simple noise reduction, and operates
faster than the other two, as it removes the need for use of the matrix class
- n-dimensional filter, for general use
- euler Kalman filter, a 4-dimensional filter designed specifically for use fusing inputs from a
9 DOF sensor to determine attitude, an example of its use is included at the bottom

NOTE: The classes requiring matrix operations (euler and n-dimensional) require the additional file 'matrixfunctions'
which adds simple matrix operations for micropython

Author: Anders Appel"""

import math, cmath
import matrixfunctions as matrix

class eulerKalman:
    """ 4-dimensional Kalman filter - 4 x 1 state variable etc.
    This is specifically for combining accelerometer, gyroscope and magnetometer values to get roll, pitch and yaw
    NOTE: It is assumed that H is an n x n identity matrix"""
    def __init__(self, Q, R):
        # Q - Process noise variance - should be a 4 x 4 matrix
        # R - Signal noise variance - also a 4 x 4 matrix
        # n - Dimension of the Kalman filter
        self._Q = Q
        self._R = R

        # Initialise the filter variables
        self._x = matrix.matrix(4, 1) # Matrix to hold the quaternion values
        self._x.store(1, 0, 0)
        self._P = matrix.eye(4) # 4 x 4 identity matrix
        self._K = 0 # Shouldn't matter, as it is calculated fresh each iteration

        # Initialise the roll, pitch and yaw
        self._roll = 0
        self._pitch = 0
        self._yaw = 0

    """ Add a new value to the filter - Allows A to be a changing value"""
    def update(self, z, A):
        # z - The new values from the sensor (should be 4 x 1 matrix)
        # A - State matrix - should be 4 x 4

        # Predict the new filter variables
        self._x = matrix.matMult(A, self._x)
        _temp = matrix.matMult(A, self._P)
        self._P = matrix.matMult(_temp, A.transpose) + self._Q

        # Calculate the Kalman gain
        _temp = matrix.matInv(self._P + self._R)
        self._K = matrix.matMult(self._P, _temp)

        # Correct the estimates
        self._x = self._x + matrix.matMult(self._K, (z - self._x))
        self._P = self._P - matrix.matMult(self._K, self._P)

        # Calculate the roll, pitch and yaw
        self.attitude()

        # Return the quaternion values (not the actual output of the filter)
        return self._x

    """ Function to return the current values"""
    def get(self):
        return self._roll, self._pitch, self._yaw

    """ Function to convert the quaternion to roll, pitch and yaw values"""
    def attitude(self):
        # Split the relevant values for readability
        _x1 = self._x.getVal(0, 0)
        _x2 = self._x.getVal(1, 0)
        _x3 = self._x.getVal(2, 0)
        _x4 = self._x.getVal(3, 0)

        self._roll = math.atan2(2*(_x3*_x4 + _x1*_x2), 1 - 2*(_x2*_x2 + _x3*_x3))
        self._pitch = -math.asin(2*(_x2*_x4 - _x1*_x3))
        self._yaw = math.atan2(2*(_x2*_x3 + _x1*_x4), 1 - 2*(_x3*_x3 + _x4*_x4))

class kalman:
    """ n-dimensional Kalman filter - n x 1 state variable etc.
    NOTE: It is assumed that H is an n x n identity matrix, and thus isn't included for simplicity"""
    def __init__(self, Q, R, n=1):
        # Q - Process noise variance - should be a 4 x 4 matrix
        # R - Signal noise variance - also a 4 x 4 matrix
        # n - Dimension of the Kalman filter
        self._Q = Q
        self._R = R

        # Initialise the filter variables
        self._x = matrix.matrix(n, 1) # Matrix to hold the quaternion values
        self._P = matrix.eye(n) # 4 x 4 identity matrix
        self._K = 0 # Shouldn't matter, as it is calculated fresh each iteration

    """ Add a new value to the filter - Allows A to be a changing value"""
    def update(self, z, A):
        # z - The new values from the sensor (should be n x 1 matrix)
        # A - State matrix - should be n x n

        # Predict the new filter variables
        self._x = matrix.matMult(A, self._x)
        _temp = matrix.matMult(A, self._P)
        self._P = matrix.matMult(_temp, A.transpose) + self._Q

        # Calculate the Kalman gain
        _temp = matrix.matInv(self._P + self._R)
        self._K = matrix.matMult(self._P, _temp)

        # Correct the estimates
        self._x = self._x + matrix.matMult(self._K, (z - self._x))
        self._P = self._P - matrix.matMult(self._K, self._P)

        # Return the output of the filter
        return self._x

    """ Function to return the current value"""
    def get(self):
        return self._x

class kalman1:
    """ Class for a one dimensional Kalman Filter"""
    def __init__(self, A, Q, R):
        # A - Filter parameter
        # Q - Process noise variance
        # R - Signal noise variance
        self._A = A
        self._Q = Q
        self._R = R

        # Initialise the filter variables
        self._x = 0
        self._P = 1 # Initial prediction matrix

    """ Function to add a new value to the filter"""
    def update(self, z):
        # z - The new measured value from the sensor

        # Predict the new variables for the new measurement
        self._x = self._x * self._A
        self._P = self._P * self._A + self._Q

        # Correct the new prediction using the measured value
        _K = self._P / (self._P + self._R)
        self._x = self._x + _K * (z - self._x)
        self._P = (1 - _K) * self._P

        return self._x

    """ Function to get the current value from the filter"""
    def get(self):
        return self._x

# Example implementation
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import numpy as np
    import random
    import time

    # Function to give a random noise variable
    def noise(mag):
        return mag * random.gauss(1, 1)

    # Define the Kalman filter parameters
    """ These must be determined beforehand """
    A = 1.01
    Q = 0.01 # Tends to be very small
    R = 0.4 # Depends on the process being measured

    # Initialise the filter
    filter = kalman1(A, Q, R)

    # Create the dummy dataset
    N = 1024 # Number of samples
    Fs = 500 # Sample rate (samples/sec)
    Ts = 1 / Fs # Sample period (sec)

    # Time variable
    t = np.linspace(0.0, N*Ts, N)

    # Example output - two sinusoids
    x = list(np.sin(5.0 * 2.0*np.pi*t) + 0.5*np.sin(2.0 * 2.0*np.pi*t))

    # Add some Gaussian noise
    y = [output + noise(0.1) for output in x]

    # Start an empty list
    filtered = []

    # Filter each value of the noisy signal
    for y_val in y:
        # Update the filter
        filter.update(y_val)

        # Get the new value
        filtered.append(filter.get())

    # Plot the results
    plt.figure(1)
    plt.plot(t, y) # Noisy signal
    plt.plot(t, filtered) # Filtered signal
    plt.show()

    """Test the euler filter"""

    # Function to simulate gyroscope outputs
    def getGyro():
        # Define the output for each axis
        Gx = 0 + noise(0.1)
        Gy = 0 + noise(0.1)
        Gz = 0 + noise(0.1)
        return Gx, Gy, Gz

    # Function to simulate accelerometer outputs
    def getAccel():
        # Define the output for each axis
        Ax = 0 + noise(0.1)
        Ay = 0 + noise(0.1)
        Az = 9.8 + noise(0.1)
        return Ax, Ay, Az

    # Function to get quaternion values
    def getQuatern():
        q1 = 0 + noise(0.1)
        q2 = 0 + noise(0.1)
        q3 = 0 + noise(0.1)
        q4 = 0 + noise(0.1)

        data = [[q1],
        [q2],
        [q3],
        [q4]]

        Q = matrix.matrix(4, 1, data)
        return Q

    # Create the dummy dataset
    N = 1024 # Number of samples
    Fs = 500 # Sample rate (samples/sec)
    Ts = 1 / Fs # Sample period (sec)

    # Time variable
    t = np.linspace(0.0, N*Ts, N)

    # Example output - two sinusoids
    # x = list(np.sin(5.0 * 2.0*np.pi*t) + 0.5*np.sin(2.0 * 2.0*np.pi*t))

    # Initialise Kalman filter matrices
    q = 0.0001 # Process noise
    r = 10 # Signal noise
    Q = matrix.matrix(4, 4, data=[[q, 0, 0, 0],
    [0, q, 0, 0],
    [0, 0, q, 0],
    [0, 0, 0, q]])
    R = matrix.matrix(4, 4, data=[[r, 0, 0, 0],
    [0, r, 0, 0],
    [0, 0, r, 0],
    [0, 0, 0, r]])

    # Initialise the filter
    filter2 = eulerKalman(Q, R)

    # Initialise the lists
    Gx = []
    Gy = []
    Gz = []
    Ax = []
    Ay = []
    Az = []
    R = []
    P = []

    start = time.process_time()

    # Sitting stationary for first test
    for n in range(N):
        # Get the gyroscope values
        (p, q, r) = getGyro()

        # Store the values in the lists
        Gx.append(p)
        Gy.append(q)
        Gz.append(r)

        # Define the state matrix
        A = matrix.matrix(4, 4, data=[[1, -p, -q, -r],
        [p, 1, r, -q],
        [q, -r, 1, p],
        [r, q, -p, 1]])

        # Get the accelerometer values
        (ax, ay, az) = getAccel()

        # Store the values in the lists
        Ax.append(ax)
        Ay.append(ay)
        Az.append(az)

        # Get quaternion values and filter
        filter2.update(getQuatern(), A)

        (roll, pitch, yaw) = filter2.get()

        R.append(roll)
        P.append(pitch)

    # Display time
    time = time.process_time() - start
    print("Time taken: ", time, " s")

    # Plot the results
    plt.figure(2)
    plt.plot(t, P) # Noisy signal
    plt.plot(t, R) # Filtered signal
    plt.show()
