# Eddie the Balancing Robot
Eddie is a relatively small robot that balances itself on two wheels. It is controlled by an Arduino board using an accelerometer & gyroscope sensor.

## Depencencies

- PID library (http://playground.arduino.cc/Code/PIDLibrary)
- MPU6050 library for Arduino (https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
- I2Cdev library (https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev)

Dependencies should be installed to Arduino libraries folder (located in /Users/username/Documents/Arduino/libraries on mac)

[Read more about libraries](https://www.arduino.cc/en/Guide/Libraries)

## Main components
- MPU6050 accelerometer & gyroscope sensor
- Two 330 rpm motors with wheel encoders
- L298N dual-H bridge DC motor driver
