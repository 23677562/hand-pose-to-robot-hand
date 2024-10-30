
# Hand Pose To Robotic Hand

## Overview
This project is a ROS 2-based robotic control system that processes video streams for vision-based hand gesture tracking and robot control. It consists of three primary packages:
- **Camera Feed:** Captures and publishes video streams to other nodes.
- **Vision Processing:** Applies pose estimation or other vision processing techniques to the video stream.
- **Control:** Uses processed data to control a robotic hand or arm, depending on the selected control mode.

Currently as an example this project is setup to use the [Allegro Hand](https://www.allegrohand.com/). While the primary vision processing performed in this project is hand pose estimation, a node for gesture recognition (specifically for the allegro hand) was created to experiment with.

## Features
- **Modular Design:** Choose different vision processing methods and control mechanisms for flexible robot manipulation.
- **Camera Integration:** A video feed is captured and shared throughout the ROS workspace for processing and control.
- **Vision and Control Flexibility:** Easily swap between different vision processing techniques (e.g., hand pose estimation) and robotic control commands.
- **Configurable Parameters:** Uses YAML configuration files for setting up camera parameters, vision processing options, and robot control details.
- **Launch File Management:** Conveniently launch multiple nodes using custom ROS 2 launch files.

## Setup

### Requirements
- ROS 2 (Foxy or later)
- Python 3.x
- OpenCV
- MediaPipe (for hand tracking)

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/23677562/hand-pose-to-robot-hand.git
   ```
2. Build the ROS 2 workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

### Configuration

#### Parameter YAML Files
Each package contains its own `params.yaml` file located in the `config/` directory. These files contain parameters for choosing the camera feed source, vision processing logging, and robot configurations.

For example, to modify camera feed parameters:
```yaml
publisher:
  ros_parameters:
   camera_url: "http://xxx.xxx.xxx.xxx:5000/video_feed"
   use_webcam: False
   webcam_device_id: 0
```

You can update these YAML files before running the nodes, depending on your hardware and processing needs.

### Launch Files & Running the Project
Launch files are located in the `launch/` directory of each package. They allow you to launch multiple nodes with predefined configurations.

1. **Launch the Camera Feed:**
   ```bash
   ros2 launch camera_feed camera_launch.py
   ```
2. **Launch Vision Processing:**
   Choose a vision processing option:
   ```bash
   ros2 launch vision_processing hand_pose_estimation.py
   ```
3. **Control the Robotic Hand:**
   Based on processed data, control the robotic hand:
   ```bash
   ros2 launch control hand_pose_to_joint_state.py
   ```

### Customizing Parameters
You can edit the `params.yaml` files in each package to adjust settings such as:
- **Camera Selection**: Choose between using a hardware connection (USB webcam) or a URL stream for the video feed. You can set the device ID for the connected webcam or the URL for the video stream.
- **Vision Processing Logging**: Configure the logging level for vision processing to monitor and debug the system's performance and behavior.
- **Robot Configuration Path**: The control node is designed to be flexible and can be used for different target robots. To change the target robot, provide a JSON file that depicts the relationship between hand landmarks and robotic joints, along with their respective angle limits. Please refere to `src/control/config/allegro_hand.json` for an example.

Once configured, run the corresponding launch file, and it will automatically load the updated parameters.
