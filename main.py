import pycom
import utime
import math
import os
import uos
from machine import I2C, Pin, SD
from fusion import tilt
# from ak8963 import AK8963

# Magnetometer Offset Values:
# Offset = (5.19961, -4.03418, -56.78174)
# Scale = (1.067856, 1.114284, 0.8575542)

# Mount the SD card for recording data
sd = SD()
os.mount(sd, '/sd')

pycom.heartbeat(False)

# Set how many samples to take
N = 4096

# Initialise the file
f = open('/sd/MPU9250.txt', 'w')
f.write("Ax, FAx, Gx, Mx\n") # Write an introduction
f.close()

i2c = I2C(0, I2C.MASTER, baudrate=100000)

# Set the magnetometer to have the appropriate offset and scale
# ak8963 = AK8963(
#     i2c,
#     offset=(-136.8931640625, -160.482421875, 59.02880859375),
#     scale=(1.18437220840483, 0.923895823933424, 0.931707933618979)
# )

sensor = tilt(i2c, ak8963=None)

# values = tilt(i2c)

print("MPU9250 id: " + hex(sensor.sensor.whoami))

time = utime.ticks_ms()

# Open the file for appending
f = open('/sd/MPU9250.txt', 'a')

counter = 0

while counter < N:
    # Get new measurements from the sensor
    sensor.update()

    # Get the desired values from the sensor class
    Ax = sensor.accel[0]
    FAx = sensor.accel_filter_x.get()

    Gx = sensor.gyro[0]
    Mx = sensor.mag[0]

    print("Measurement: ", counter)

    # Store the values in the file
    f.write("{},{},{},{}\n".format(Ax, FAx, Gx, Mx))

    counter += 1

# Close the file
f.close()

# Print the elapsed time
total = utime.ticks_ms() - time
print("Total time elapsed is: ", total, "ms")

# Unmount the SD card for further use
uos.unmount('/sd')
