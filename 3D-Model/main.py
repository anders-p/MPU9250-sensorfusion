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

pycom.heartbeat(False)

# Set how many samples to take
N = 2048

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

counter = 0

while True:
    # Commented out commands are for reference, may be used in future

    # Get new measurements from the sensor
    # (roll, pitch, yaw) = sensor.update()
    (q1, q2, q3, q4) = sensor.update()

    # Get some comparison values from the sensor
    (Gx, Gy, Gz) = sensor.get_gyro()
    (Ax, Ay, Az) = sensor.get_accel()
    (Mx, My, Mz) = sensor.get_mag()
    (raw_roll, raw_pitch, raw_yaw) = sensor.attitude(Ax, Ay, Az, Mx, My, Mz)
    # raw_yaw = sensor.____________

    # print(roll, ",", pitch, ",", yaw)
    # print('%.4f' % round(q1, 4), ",", '%.4f' % round(q2, 4), ",", '%.4f' % round(q3, 4), ",", '%.4f' % round(q4, 4))

    q1 *= 1000
    q2 *= 1000
    q3 *= 1000
    q4 *= 1000
    print(round(q1), ",", round(q2), ",", round(q3), ",", round(q4))

    # IMPORTANT
    # Has to send data slow enough for Processing sketch, if this is too low the serial link will keep closing
    utime.sleep_ms(100)

    # counter += 1

# Print the elapsed time
total = utime.ticks_ms() - time
