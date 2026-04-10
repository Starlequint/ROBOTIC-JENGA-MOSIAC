# **Robotic Jenga Mosaic Project**

Implementation of Gen 3 (kinova) motion to pick and place jenga planks to extend a given, pre-determined pattern.
The **Robotic Jenga Mosaic** project involves using a robotic arm to interact with Jenga planks and perform a variety of tasks such as detection, manipulation, and structural assembly. This project incorporates vision-based processing (with detection of Jenga planks), kinematic motion (both forward and inverse), and robotic arm control (including gripper commands). It integrates Python scripts to enable smooth operations for robotic movement and plank interaction.

---

## **Project Structure**


ROBOTIC-JENGA-MOSIAC/

├── getImage.py # Captures and processes images from a camera.

├── instrutions.md # Instructions or guidelines related to the project.

├── JENGA_detection_v5.py # Detects Jenga planks and processes visual data.

├── main.py # Main script that runs the core robot control and movement.

├── move_angular_and_cartesian.py # Functions for angular and Cartesian movements of the robot arm.

├── robohub_codes.py # Example scripts for forward/inverse kinematics and gripper control.

├── TriangleTessellation.py # Defines a tessellation method for structural planning with planks.

└── utilities.py # Helper functions for device connection management (TCP/UDP).


---

## **Setup and Installation**

1. **Prerequisites**:
   - **Python 3.x**: Ensure that Python 3.x is installed on your system.
   - **Kinova Robot and Kortex API**: You need access to a Kinova robotic arm that supports the Kortex API.
   - **Camera/Computer Vision Tools**: You may also need a camera and image processing tools to perform plank detection (e.g., OpenCV).

2. **Install Dependencies**:
   This project uses several Python libraries. Install them via `pip` as follows:
   
   ```bash
   pip install kortex-api opencv-python matplotlib numpy argparse pillow
Set up the Robot:
Connect your Kinova robotic arm to your network.
Ensure the robot's API is accessible from your development machine (check the IP address and credentials).
Make sure the robot's software is set up according to Kinova's documentation.
Image Capture:
If you plan to use plank detection and vision-based feedback, ensure you have a working camera setup that integrates with OpenCV. The getImage.py file contains functions for capturing images.
Usage
0. Launch the project
The main.py script contains the core of the project and calls to other modules.
```bash
./main.py
```
The following are interesting only if you care about using the modules individually.
1. Move the Robot Arm

The move_angular_and_cartesian.py script contains the core functions for controlling the robot arm.

Home Position:
Moves the robot to a predefined "home" or safe position.
```python
from move_angular_and_cartesian import example_move_to_home_position
example_move_to_home_position(base)
```
Angular Movement:
Move the robot arm in joint space using angular commands.

from move_angular_and_cartesian import example_angular_action_movement
example_angular_action_movement(base)

Cartesian Movement:
Move the robot arm in Cartesian space (x, y, z) and apply orientations (theta_x, theta_y, theta_z).
```python
from move_angular_and_cartesian import example_cartesian_action_movement
example_cartesian_action_movement(base, base_cyclic, x=0.1, y=0.1, z=0.1, ox=0.0, oy=0.0, oz=0.0)
```
2. Forward and Inverse Kinematics

The robohub_codes.py file contains functions for performing forward and inverse kinematics.

Forward Kinematics:
Given joint angles, this function computes the robot's pose in Cartesian space.
```python
from rohub_codes import example_forward_kinematics
pose = example_forward_kinematics(base, angles=[0, 0, 0, 0, 0, 0])
```
Inverse Kinematics:
Given a target Cartesian pose, this function computes the required joint angles to reach that pose.
```python
from rohub_codes import example_inverse_kinematics
joint_angles = example_inverse_kinematics(base, x=0.1, y=0.1, z=0.1, ox=0.0, oy=0.0, oz=0.0)
```
3. Gripper Control

The gripper can be controlled using the GripperCommandExample class from robohub_codes.py. You can open, close, and control the gripper's speed.

Gripper Open:
```python
from rohub_codes import GripperCommandExample
gripper = GripperCommandExample(router)
gripper.open()
```
Gripper Close:
```python
gripper.close(end=1.0)
```
Gripper Speed Move:
```python
gripper.speedMove(speed=-0.1)
```
4. Image Capture for Jenga Detection

The JENGA_detection_v5.py file includes vision-based processing to detect Jenga planks in a scene. This file utilizes OpenCV to identify plank positions and orientations.

Running Image Detection:
```python
from JENGA_detection_v5 import detect_planks
results = detect_planks(
        raw_image_path   = rawImagePath,
        color_image_path = colourImagePath,
        save_path        = 'detected_planks.png',
        show=False
    )
```
Image Capture:

The getImage.py file can be used to capture live images from a camera, which can be processed by the Jenga detection script:
```python
from getImage import get_single_frame
rtsp_url = "rtsp://admin:admin@192.168.1.10/color"
frame_array = get_single_frame(rtsp_url)
5. Triangle Tessellation
```
The TriangleTessellation.py file contains methods for tessellating triangles or other shapes for use in the Jenga mosaic or structural planning.

Running Tessellation:
```python
from TriangleTessellation import patternRecognition
patternRecognition(rawData)
```
6. Device Connection

The utilities.py file provides helper functions to connect to the robot via TCP or UDP. You can connect to the robot using the provided IP address, username, and password.

TCP Connection:
```python
from utilities import DeviceConnection, parseConnectionArguments
args = parseConnectionArguments()
with DeviceConnection.createTcpConnection(args) as router:
    # Interact with the robot
```
UDP Connection:
```python
from utilities import DeviceConnection, parseConnectionArguments
args = parseConnectionArguments()
with DeviceConnection.createUdpConnection(args) as router:
    # Interact with the robot at 1kHz
```
Module's code Breakdown
1. getImage.py

Captures images from the camera for processing. The images can be saved and passed to other scripts for Jenga plank detection.

2. JENGA_detection_v5.py

Detects and processes the Jenga planks from captured images. It processes visual input, identifies plank positions, and computes useful data for robot interaction.

3. TriangleTessellation.py

Contains functions for tessellating triangular or polygonal shapes. This could be used for planning structural patterns or arranging Jenga planks into a mosaic pattern.

4. move_angular_and_cartesian.py

This script contains functions to control the robot's movements, both in joint space (angular) and Cartesian space (x, y, z). It also includes code to handle action notifications, ensuring the arm reaches the desired position.

5. robohub_codes.py

Includes functions for forward and inverse kinematics, allowing the robot to calculate the correct joint angles for a given pose. It also has code for controlling the robot's gripper.

6. utilities.py

Helper functions for setting up TCP/UDP connections to the robot and managing login sessions.

Contributing

If you would like to contribute to the project, feel free to fork the repository, create a new branch, and submit a pull request. Please ensure that any code changes are properly tested.

### **License**

This project is distributed under the following terms:

1. **Kinova Code License**:
   - The code related to **Kinova** (found in files such as `move_angular_and_cartesian.py`, `robohub_codes.py`, and other files that directly interact with the Kinova API) is subject to the **Kinova License**. This code is provided under the **BSD 3-Clause License** as specified by **Kinova Inc.**
   - **BSD 3-Clause License**:
     - The software can be modified and distributed under the terms of the BSD 3-Clause License, which is a permissive open-source license.
     - Please refer to the `LICENSE` file provided in the project repository for full details of the **BSD 3-Clause License**.

2. **Other Code**:
   - The remaining code (e.g., related to image processing, robotics control algorithms, utilities, etc.) is **open source** and can be freely used, modified, and distributed.
   - This code **does not have a specific license** attached, meaning it is provided "as is" with no warranties. It is open for use, but users should be aware that the project author does not provide any formal licensing or support for these parts of the code.
  
### Acknowledgments
Thanks to Kinova for providing the robotic arm and the Kortex API.
This project was inspired by real-world applications in robotics and automation.

### Reference

User guide: https://www.kinovarobotics.com/product/gen3-robots#Product__resources
Note: we are using the 7 DOF one.
