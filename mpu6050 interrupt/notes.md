# Notes

## 09 October 2024
### 9:27

I used the following command from the raspberry pi to set and read the data from the power managament register of the MPU6050 and I was able to read the set data. Here is the input and output

```

>>> import smbus2
>>> bus = smbus2.SMBus(1)
>>> device_address = 0x68
>>> bus.read_byte_data(device_address, 0x6B)
64
>>> bus.write_byte_data(device_address, 0x6B, 1)
>>> bus.read_byte_data(device_address, 0x6B)
1
>>> 

```

SMPLRT\_DIV for a required frequency can be calculated using the formula

SMPLRT\_DIV = $ (gyro\_freq/required\_freq) - 1$
