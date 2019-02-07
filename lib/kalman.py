### Implementing discrete Kalman filter ###
""" Function for implementing a discrete Kalman filter
A Kalman filter uses a probability distribution to predict the next value from a source,
such as a sensor. It then updates this probability distribution based on the value given
from the source, providing a smoothing effect in a similar way to a moving average filter.
It works in two stages, first prediction, where it guesses what the next value from the sensor
will be, then correction, where it uses the actual value from the source to correct its prediction.
It requires 3 main parameters to be determined externally to use:
    A - The 'prediction matrix', which determines the predicted state of the system, this can
    usually be approximated as 1 for one-dimensional systems, or an identity matrix for higher order
    Q - The covariance of the 'process noise', this is static and represents the idea
    of how quickly the system responds to external input
    R - The covariance of the 'signal noise', this is static and is a measure of how
    much noise is expected in the system
The remaining 3 inputs are variable, and change as the filter is used:
    z - The input from the source, usually a sensor
    x_prev - The previous output from the filter -- Usually initialised as zero
    P_prev - The previous value of the 'confidence' matrix, or the value of the corrected error
    covariance -- This can be initialised at anything EXCEPT zero, and will correct itself as it's used,
    better initial estimates will result in faster convergence
NOTE: This is still being developed, and can still be vastly improved"""

import math, cmath

# ONE DIMENSIONAL Kalman filter implementation - this assumes all parameters are integers, not matrices
def one_dim(z, x_prev, P_prev, A, Q, R):
    # z - The new measured value from the sensor
    # x_prev - The previous corrected measured value
    # P_prev - The previous corrected value of the confidence
    # A - Filter parameter
    # Q - Process noise variance
    # R - Signal noise variance

    # Predict the next values
    x_predicted = A * x_prev
    P_predicted = A * P_prev + Q

    # Correct the prediction using the measured value
    K = P_predicted / (P_predicted + R)
    x_corrected = x_predicted + K * (z - x_predicted)
    P_corrected = (1 - K) * P_predicted

    return (x_corrected, P_corrected)

# Example implementation
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import numpy as np
    import random

    # Function to give a random noise variable
    def noise(mag):
        return mag * random.gauss(1, 1)

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

    # Define the Kalman filter parameters
    """ These must be determined beforehand """
    A = 1.01
    Q = 0.01 # Tends to be very small
    R = 0.4 # Depends on the process being measured

    # Initialise the variables
    filtered_val = 0
    filtered_P = 1

    # Filter each value of the noisy signal
    for y_val in y:
        filtered_val, filtered_P = one_dim(y_val, filtered_val, filtered_P, A, Q, R)
        filtered.append(filtered_val)

    # Plot the results
    plt.figure(1)
    plt.plot(t, y) # Noisy signal
    plt.plot(t, filtered) # Filtered signal
    plt.show()
