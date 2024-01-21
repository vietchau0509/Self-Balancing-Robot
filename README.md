# Self-Balancing-Robot
This project involves designing a 2-wheeled, self-balancing robot with capabilities for sensor integration. Using 3-axis accelerometers, gyros, and odometry, the robot maintains an upright position while allowing for IR remote-controlled movement.

Key Components
Motion Control: Integrates accelerometers and gyros for balance.
Drive System: Powered by a 4xAA NiMH battery source and controlled via a DRV8833 dual H-bridge motor driver.
Sensors: Features an MPU-6050 6-DOF IMU, optical interrupters for odometry, and a TSOP13438 IR demodulator for remote commands.
Microcontroller: Utilizes an ARM M4F core (TM4C123GH6PMI) for processing.
Software and Interface
Kinematic Software: Enables stable balancing and precise movement control.
Command-Line Interface: Supports commands for movement and status checks via a virtual COM port.
IR Remote Interface: Offers additional control options through an IR remote.
Testing and Validation
Hardware testing is conducted to ensure operational accuracy, focusing on balancing, straight-line movement, and rotation precision.

Development and Learning
This project demonstrates intricate hardware-software integration, offering a hands-on experience in robotic controls, sensor interfacing, and real-time system programming.
