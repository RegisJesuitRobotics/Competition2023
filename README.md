# Competition 2023
## Glossary of Terms
- AHRS
  - Stands for attitude and heading reference system. Kauai Labs uses AHRS as the class to read the gyroscope
- Encoder
  - An encoder is a sensor that can tell the position (and velocity) of a system.
- Kinematics
  - Kinematics are how we can tell what our entire system is doing (drivetrain for example) from the velocities/positions of our encoders and the other way around. For our swerve, for example, we can say we want to drive 1 m/s in the x, 2 m/s in the y, and 1 rad/s rotation and kinematics will tell us what direction each wheel should face and what velocity they should drive at.
- Desaturate
  - TODO
- NavX
  - Used interchangeably with AHRS. It's the actual name for the kauai labs gyro part.
- FGPA
  - The Field programmable gate array. It's a part of RIO. The `Timer.getCurrentFGPATime()` returns the time in seconds according to the FGPA clock.
- CAN
  - TODO
- Closed Loop
  - TODO
- Feedforward/Open Loop
  - TODO
- Characterization
  - Characterization is a process that allows us to easily get the physical constants of our system.
- Tunable Classes
  - The tunable classes allow us to change the gains for our feedforward/feedback controllers without having to redeploy the code.
- PID
- Trapezoidal
  - A trapezoidal motion profile is a way to smooth setpoints for the actual constraints of our system. It also enables usage of velocity setpoints for position goals.
- NT
  - Network tables. A Pub-Sub network protocol that is used to transfer information to/from the rio to a different device (dashboard, coprocessor). It can transfer different types of data such as integers, arrays, strings, or just bytes.
- Snap
  - TODO

## Why Swerve Drive?
- Benefits
  - TODO
- Explain how to use constants.java
  - TODO
