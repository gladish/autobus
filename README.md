# Autonomous Vehicle

Autobus is a project to convert a Tamiya King Yellow 6x6 RC bus into a fully autonomous vehicle.

The current hardware platform centers around a Raspberry Pi 5 with camera and servo control hardware, with planned networking and computer vision milestones.

## Project Goal

Build an autonomous RC platform that can:
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

## Project Tracking

Detailed planning notes and architecture decisions are maintained in the wiki.
Actionable work items are tracked in GitHub Issues.
