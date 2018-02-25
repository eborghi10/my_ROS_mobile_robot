# Teapot

From [here](http://arduinoandmpu6050.blogspot.com.ar/2013/11/arduino-due-wiht-mpu6050.html).

In the arduino program:

- comment Line-100 (we do not need YPR), Line-165 (TWBR is not assignable in ARM architecture)

- Uncomment Line-117 (We want teapot output)

- Change Serial rate as required, Line-173 (57600 works fine)

- Not neccesary, change offsets, Line- 201 to 204

- Line-254, change 0=>2 (different for 'due', assign pin-2 as an intterupt detection)

For the Processing code:

- In the Library Manager download the "_Toxi Library_".

- Open the downloaded processing code and change the port name, Line-74 (eg. "COM1" for Windows and "/dev/ttyACM0" for Unix-based).

- change the baud rate, Line-77. (57600 or any other but, should be same as used in arduino) 

**NOTE** Default MPU Address is 0x68. if you dont know device address refer to the web or any other source. if above code doesn't work try change it to 0x69 instead. (Line-61,62).

# IMU examples

Extra notes [here](https://docs.google.com/document/d/1qAH08FEJKtgD55jEGmZ0EA3cySXDe4CYROtKoIuOLTE/edit?usp=sharing).

## MPU6050 calibration

More info [here](https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/).

An auto calibration [here](http://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/).

## mpu6050_serial_to_imu

Arduino ROS node: more info [here](https://github.com/fsteinhardt/mpu6050_serial_to_imu).

## Arduino DUE issues

[Not compile](https://github.com/ros-drivers/rosserial/issues/85).

[Another problem](https://github.com/ros-drivers/rosserial/issues/113).

An [example](http://forum.arduino.cc/index.php?topic=318110.0) with the magnetic encoder AS5048.