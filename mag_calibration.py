import pycom
from machine import I2C, Pin
from mpu9250 import MPU9250
from ak8963 import AK8963

# Returned Values:
# Offset = (5.19961, -4.03418, -56.78174)
# Scale = (1.067856, 1.114284, 0.8575542)

i2c = I2C(0, I2C.MASTER, baudrate=100000)

ak8963 = AK8963(i2c)
offset, scale = ak8963.calibrate(count=256, delay=200)

#sensor = MPU9250(i2c, ak8963=ak8963)

print("Offset: ", offset)
print("Scale: ", scale)
