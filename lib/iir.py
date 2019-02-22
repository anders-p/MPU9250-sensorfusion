## Implementing discrete IIR ##
"""Class for second order discrete IIR filtering in micropython

Coefficients for the numerator and denominator can be determined using various filter
design software, such as Matlab

It is possible to implement higher order filters by chaining multiple first and second order filters
together, the coefficients of these would again need to be calculated manually
The functions are intended to work on microprocessors using micropython, the example
shown below uses Python functions only to create an example dataset and plot the results

NOTE: The filters do introduce a significant time delay in the signal, which needs to be taken
into account when using them. An example of how to delay the output is shown, but is not the most
efficient way of doing this

FUTURE UPDATES: first order iir filter, built-in time delay, remove need for rotating lists"""

import math, cmath

# Second Order implementation using Direct Form II
class iir:
    """Class for a second order IIR filter"""
    def __init__(self, a, b):
        # a - Denominator coefficients [a0, a1, a2]
        # b - Numerator coefficients [b0, b1, b2]
        self._a = a
        self._b = b
        
        # Initialise the filter variables
        self._w = [0, 0, 0]
        self._y = 0
        
    """ Function to update the filter with a new value"""
    def update(self, x):
        # x - The new measurement value to be put in the filter
        
        # Rotate the buffer
        self._w = self.rotate(self._w, -1)
        
        # Calculate the new buffer value
        self._w[0] = x - self._a[1] * self._w[1] - self._a[2] * self._w[2]
        
        # Calculate the new output value
        self._y = self._b[0] * self._w[0] + self._b[1] * self._w[1] + self._b[2] * self._w[2]
        
        # Return the new value
        return self._y
        
    """ Function to rotate an array by k """
    def rotate(self, arr, k):
        return arr[k:]+arr[:k]
    
    """ Function to get the current filter value """
    def get(self):
        return self._y

# Example implementation of the filters
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import numpy as np
    import random
    
    # Function to give a random noise variable
    def noise(mag):
        return mag * random.gauss(1, 1)
    
    # Define the parameters for the second order filter
    """ These must be determined beforehand to obtain the output you want """
    a = [1.0000,   -1.8879,    0.8946] # Denominator coefficients
    # Gain
    K = sum(a) / 4
    b = [1.0000*K,    1.9701*K,    0.9703*K] # Numerator coefficients
    
    # Initialise the filter
    filter = iir(a, b)

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

    # Cycle through the output and filter
    for y_val in y:
        # Update the filter
        filter.update(y_val)
        
        # Get and store the new filtered value
        filtered_val = filter.get()
        filtered.append(filtered_val)

    # Plot the results
    plt.figure(1)
    plt.plot(t, y) # Noisy signal
    plt.plot(t, filtered) # Filtered signal
    plt.show()
