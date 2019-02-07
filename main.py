import pycom
import utime
import math
from machine import I2C, Pin
from fusion import tilt
from ak8963 import AK8963

# Magnetometer Offset Values:
# Offset = (5.19961, -4.03418, -56.78174)
# Scale = (1.067856, 1.114284, 0.8575542)

pycom.heartbeat(False)

i2c = I2C(0, I2C.MASTER, baudrate=100000)

# Set the magnetometer to have the appropriate offset and scale
ak8963 = AK8963(
    i2c,
    offset=(-136.8931640625, -160.482421875, 59.02880859375),
    scale=(1.18437220840483, 0.923895823933424, 0.931707933618979)
)

values = tilt(i2c, ak8963=ak8963)

# values = tilt(i2c)

print("MPU9250 id: " + hex(values.sensor.whoami))

time = utime.ticks_ms()

dt = 0
counter = 0

while counter < 10:
    values.measure()
    values.accel_rp()
    values.yaw()
    values.gyro_vals(dt/1000)

    dt = utime.ticks_ms() - time

    print("Roll: ", values.rad_roll * 180 / math.pi, "Gyro: ", values.G_roll * 180 / math.pi, "Raw Gyro: ", values.gyro[0])
    print("Pitch: ", values.rad_pitch * 180 / math.pi, "Gyro: ", values.G_pitch)
    print("Yaw: ", values.rad_yaw * 180 / math.pi, "Gyro: ", values.G_yaw)
    print("Dt = ", dt)

    print()

    time = utime.ticks_ms()



    counter += 1
