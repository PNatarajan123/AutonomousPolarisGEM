# AutonomousPolarisGEM
Autonomous vehicle lane detection, waypoint navigation, and Monte Carlo Localization for Self-Driving Polaris GEM car

This guide provides instructions on how to set up and run the project. Please ensure you have the required dependencies installed on your system before proceeding.

## Dependencies

- Ubuntu 20.04
- ROS Noetic
- Gazebo Simulator

## Setup Instructions

Follow these steps to set up the project environment:

1. Source the ROS Noetic setup script:

    ```bash
    source /opt/ros/noetic/setup.bash
    ```

2. Install the required Python package:

    ```bash
    python3 -m pip install scipy
    ```

3. Compile the project using catkin:

    ```bash
    catkin_make
    ```

4. Source the project's setup script:

    ```bash
    source ./devel/setup.bash
    ```

## Running the Project

### Section 1: Lane Detection

#### Simulator Setup

1. Open `mp1/src/StudentVision.py` and make the following adjustments:
    - Uncomment the simulator subscriber line.
    - Comment the rosbag simulator subscriber line.

#### Execution

Run the following commands in separate terminals:

1. Launch the ROS project:

    ```bash
    roslaunch mp1 mp1.launch
    ```

2. Start the student vision script:

    ```bash
    python3 studentVision.py
    ```

3. Run the main script:

    ```bash
    python3 main.py
    ```

### Section 2: Waypoint Navigation

1. Launch the waypoint navigation:

    ```bash
    roslaunch mp2 mp2.launch
    ```

2. Run the main script:

    ```bash
    python3 main.py
    ```

## Additional Information

- Ensure that you have all the necessary permissions to execute the scripts.
- For any issues or further assistance, please refer to the project's issue tracker on GitHub.
