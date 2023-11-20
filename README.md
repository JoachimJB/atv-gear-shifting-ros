# ATV Gear Shifting ROS Package

## Overview

This ROS package addresses the challenge of remotely changing gears in All Terrain Vehicles (ATVs) when the gearbox gets stuck. The system facilitates smooth gear shifts, including handling scenarios where the gears are jammed due to gearbox limitations.

**Author:**
- Joachim Jamtvedt Børresen (LoneWolf Kongsberg 2023 Bachelor thesis)
  - **Main Responsibility:** Project Leader, SCRUM Product Owner and head of Communications.
  - **Secondary Responsibility:** Created and led the development of the head package (and others, which will be pushed to the repo soon).
  - **Team Effort:** Acknowledges the collaborative effort of the team in achieving the project's goals.
  
**TEAM:**
- Joachim Jamtvedt Børresen, Martin Jørgensen and Leif Arne Ulvestad Bastesen (Computer engineers)
- Sigurd Sætherø Spangelo (Electrical engineer) 
- Vegard Skårdal Brenna and Vebjørn Aleksander Østlie (Mechanical engineers)

## Problem Description

When attempting to change gears remotely, the ATV's gearbox may become stuck. To resolve this, the package provides a solution by implementing a back-and-forth movement strategy to engage the selected gear, similar to the manual effort needed to shift a traditional car into reverse.

## Jam Circumvention

The algorithm employed by the package mitigates gear jams by recalling the last successful gear engagement. Since the brake is applied during gear shifting, the system can revert to the last successful throttable gear (R, Low, or High). It then releases the brakes, applies a slight throttle to rotate the gearbox, and repeats this process for up to 3 attempts.

## Implementation Details

- **ROS Callback Functionality:** The package utilizes ROS callback functionality to implement an interrupt-driven system. Upon receiving a relevant message, the system prioritizes identifying the current state in the state machine and takes appropriate actions to complete the stages of the state machine, ensuring a successful gearshift.

## System Requirements

- **ROS:** ROS Noetic and ROS2 Foxy
- **Ubuntu:** 20.04 LTS and 22.04 LTS (Note: ROS-Noetic is chosen for compatibility with Arduino UNO and Mega using the rosserial library)
- **ROS Bridge:** Required to communicate with ROS2 Foxy on the ATV onboard server

## Getting Started

[//]: # (Getting started section details will be added later.)

## Configuration

Ensure that the ROS Bridge is set up to communicate with ROS2 Foxy on the ATV onboard server. Check the configuration files for specific settings.

## Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
