## Implementing discrete IIR ##
"""Functions for first and second order discrete IIR filtering
Coefficients for the numerator and denominator can be determined using various filter
design software, such as Matlab
It is possible to implement higher order filters by chaining multiple first and second order filters
together, the coefficients of these would again need to be calculated manually
The functions are intended to work on microprocessors using micropython, the example
shown below uses Python functions only to create an example dataset and plot the results
NOTE: The filters do introduce a significant time delay in the signal, which needs to be taken
into account when using them. An example of how to delay the output is shown, but is not the most
efficient way of doing this"""

import math, cmath

# Second Order implementation using Direct Form II
def second_order(a, b, x, w):
    # a - Denominator coefficients [a0, a1, a2]
    # b - Numerator coefficients [b0, b1, b2]
    # x - New data point
    # w - Buffer
    # y - New output data

    # Shift the Buffer
    w = rotate(w, -1)

    # Calculate the new buffer value
    w[0] = x - a[1] * w[1] - a[2] * w[2]

    # Calculate the new data value
    y = b[0] * w[0] + b[1] * w[1] + b[2] * w[2]

    # Return the current value and the new buffer
    return (y, w)

# Function to circle an array
def rotate(x, k):
    # Rotate x k times
    return x[k:]+x[:k]

# Example implementation of the filters
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

    # Define the parameters for the second order filter
    """ These must be determined beforehand to obtain the output you want """
    a = [1.0000,   -1.8879,    0.8946] # Denominator coefficients
    # Gain
    K = sum(a) / 4
    b = [1.0000*K,    1.9701*K,    0.9703*K] # Numerator coefficients
    w = [0] * 3

    # Cycle through the output and filter
    for y_val in y:
        filtered_val, w = second_order(a, b, y_val, w)
        filtered.append(filtered_val)

    # Plot the results
    plt.figure(1)
    plt.plot(t, y) # Noisy signal
    plt.plot(t, filtered) # Filtered signal
    plt.show()
