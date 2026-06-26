# Autonomous Vehicle

Autobus is a project to convert a Tamiya King Yellow 6x6 RC bus into a fully autonomous vehicle.

## Overview

The project centers on a Raspberry Pi 5-based robotics platform with camera input and servo control hardware. The long-term goal is to build toward autonomous navigation, while the immediate focus is on establishing a reliable hardware and software foundation for control, connectivity, and perception.

## Project Goals

Autobus is intended to become an autonomous RC platform that can:
- Control steering and drivetrain components from onboard software
- Connect to a home network for remote operations and telemetry
- Stream camera data for monitoring and future perception pipelines
- Progress toward autonomous navigation using OpenCV-based vision

## Base Platform

- Chassis: Tamiya King Yellow 6x6 RC bus
- Primary compute: Raspberry Pi 5
- Servo controller: Adafruit PCA9685 (I2C)
- Camera: Raspberry Pi Camera Module 3

## Images

![The original bus](docs/images/stock_king_yellow.png)

![Our base model](docs/images/starter_bus.png)

## Project Resources

- [Wiki](https://github.com/gladish/autobus/wiki) for planning notes, architecture decisions, and project journal entries
- [Issues](https://github.com/gladish/autobus/issues) for actionable work items, bugs, and feature tracking
