# ATV Gear Shifting ROS Package

## Overview

This ROS package addresses the challenge of remotely changing gears in All Terrain Vehicles (ATVs) when the gearbox gets stuck. The system facilitates smooth gear shifts, including handling scenarios where the gears are jammed due to gearbox limitations.

## Problem Description

When attempting to change gears remotely, the ATV's gearbox may become stuck. To resolve this, the package provides a solution by implementing a back-and-forth movement strategy to engage the selected gear, similar to the manual effort needed to shift a traditional car into reverse.

## Jam Circumvention

The algorithm employed by the package mitigates gear jams by recalling the last successful gear engagement. Since the brake is applied during gear shifting, the system can revert to the last successful throttable gear (R, Low, or High). It then releases the brakes, applies a slight throttle to rotate the gearbox, and repeats this process for up to 3 attempts.

## Implementation Details

- **ROS Callback Functionality:** The package utilizes ROS callback functionality to implement an interrupt-driven system. Upon receiving a relevant message, the system prioritizes identifying the current state in the state machine and takes appropriate actions to complete the stages of the state machine, ensuring a successful gearshift.

## System Requirements

- ROS: Melodic
- Ubuntu: 20.04 (Note: ROS-Noetic is chosen for compatibility with Arduino UNO and Mega using the rosserial library)
- ROS Bridge: Required to communicate with ROS2 Foxy on the ATV onboard server

## Getting Started

1. Clone this repository:

    ```bash
    git clone https://github.com/your-username/atv-gear-shifting-ros.git
    ```

2. Navigate to the ROS workspace:

    ```bash
    cd catkin_ws/src
    ```

3. Build the package:

    ```bash
    catkin build
    ```

4. Source the workspace:

    ```bash
    source devel/setup.bash
    ```

5. Run the ROS node:

    ```bash
    roslaunch atv_gear_shifting gear_shifting.launch
    ```

## Configuration

Ensure that the ROS Bridge is set up to communicate with ROS2 Foxy on the ATV onboard server. Check the configuration files for specific settings.

## Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
