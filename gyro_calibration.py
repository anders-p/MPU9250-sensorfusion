""" Script to calibrate the MPU9250 gyroscope
Default settings are 256 samples, with 200ms in between each sample
During the calibration, the sensor should be completely still

The script will save the offset values in a file called 'gyro_calibration.txt' on an SD card
It will also print them to the console for immediate use
These offset values must then be SUBTRACTED from any gyroscope measurements later made"""

import pycom
from machine import I2C
from mpu6500 import MPU6500
import utime
from machine import SD
import os

# Example output:
# Offset = (-83.53642,78.94066,-69.39014) => Offset in form (Gx, Gy, Gz)

# Averaging function
def update_average(avg, val, ind):
    return (1 / (ind + 1)) * (avg * ind + val)

# Mount the SD card for recording data
sd = SD()
os.mount(sd, '/sd')

pycom.heartbeat(False)

# Initialise the file
f = open('/sd/gyro_calibration.txt', 'w')
f.write("Offset Values for MPU9250 gyroscope:\n") # Write an introduction
f.close()

# Initialise the I2C connection
i2c = I2C(0, I2C.MASTER, baudrate=100000)
gyroSensor = MPU6500(i2c)

samples = 256 # Number of samples to average
delay = 200 # Delay between samples (ms)
counter = 1

(Gx, Gy, Gz) = gyroSensor.gyro
ax = Gx
ay = Gy
az = Gz

print(counter)

while counter < samples:
    (Gx, Gy, Gz) = gyroSensor.gyro

    ax = update_average(ax, Gx, counter)
    ay = update_average(ay, Gy, counter)
    az = update_average(az, Gz, counter)

    counter+= 1

    print(counter)

    utime.sleep_ms(delay)

print("Gyroscope offset:")
print("Offset = ({},{},{})".format(ax, ay, az))

# Open the file
f = open('/sd/gyro_calibration.txt', 'a')
# Record the values
f.write("Offset = ({},{},{})\n".format(ax, ay, az))
f.close()
