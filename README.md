# Self Balancing Robot [![Build Status](https://travis-ci.org/linorobot/lino_install.svg?branch=master)](https://travis-ci.org/linorobot/lino_install)


This repository contains the firmware, PID Tunning on Simulink and ROS Navigation package for a Self-Balancing Robot.


## Goals <!-- Features --> 

- Self-Balacing
- Localization
- Autonomous Navigation


## Hardware

- Arduino MEGA 2560
- MPU6050 IMU
- A4988 Stepper Motor Drive
- Raspberry Pi 3/B+


## Dependencies

- git
- [Arduino](https://www.arduino.cc/en/Main/Software)
- [ROS](http://wiki.ros.org/kinetic/Installation)


## Installation 

Clone this package in your workspace

```
cd ~/your_workspace
git clone https://github.com/fredvaz/bluerov2.git
```


<!-- ## Running with ROS -->
<!-- ## Creating a Map -->
<!-- ## Diagram of the software components -->


## References

- Zheng, B. G., Huang, Y. B., & Qiu, C. Y. (2014). LQR+ PID control and implementation of two-wheeled self-balancing robot. In Applied Mechanics and Materials (Vol. 590, pp. 399-406). Trans Tech Publications.

- Ding, Y., Gafford, J., & Kunio, M. (2012). Modeling, Simulation and Fabrication of a Balancing Robot. Harvard University, Massachusettes Institute of Technology, 151.


## License

Self Balancing Robot package is open-sourced under the GNU General Public License v3.0. See the
[LICENSE](LICENSE) file for details.
