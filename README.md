# Madgwick filter for 9 dof MPU6050 + HMC5883l

This is an ESP-IDF project for ESP32-S3 (and similar) which reads the 9dof data from sensors and then interprets them via a Madgwick filter. The purpose of this program is to test and experiment with the Madgwick filter for the purposes of a masters thesis.

## How to use example

The project can be used by compiling the code using ESP-IDF with minimum version 5.2.1

Compile the code and upload it to the board.

The program is made to run with a GY-87 10DOF IMU board, although it can be used with a MPU6050 and HMC5883l connected to the same I2C bus. You have to adjust the pins for I2C according to your connection. 

After the program is running open Teleplot in VScode and you should be able to see graphs and a 3D animation of teh board.

## Credits
https://github.com/nickrehm/dRehmFlight
https://github.com/jango175
