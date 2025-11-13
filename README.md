# Roaming Roombas: DIY Social Robot Platform

Roaming Roombas is a versatile, low-cost platform for building expressive and customizable social robots using the Create3 Roomba as a movement hub. Designed for Human-Robot Interaction (HRI) research and education, it enables rapid prototyping through 3D printing or cardboard crafting, and supports real-time facial expressions and behavior control via Python or ROS2.

## 🚀 Features

- **Modular Design**: Easily adapt robot size and appearance using 3D printed or cardboard components.
- **Expressive Faces**: Display reactive facial expressions on a touchscreen or mobile device.
- **Flexible Behaviors**: Define robot actions using a storyboard-driven software architecture.
- **Open Source**: Hardware designs and software are customizable and extensible.
- **Low-Cost**: Build robots for under \$600 USD using accessible materials and components.

## 🧱 Architecture

### Hardware

- **Movement Hub**: iRobot Create3
- **Controller**: Raspberry Pi 4B
- **Face Display**: 7" touchscreen or mobile device
- **Sensors**: IR proximity and bump sensors
- **Power**: Powered via Create3’s onboard battery

A full price list with links to necessary components is available in the Hardware folder above.

### Software

- **Languages**: Python or ROS2
- **Components**:
  - `MotionNode`: Manages movement phases and sensor input
  - `PhaseController`: Links behaviors to expressions
  - `BlinkManager`: Adds lifelike blinking
  - `Storyboard`: Maps user-defined scenarios to robot actions

## Setup of Create3 Platform with Raspberry Pi

### Prerequisites

1. **Install Ubuntu on your Raspberry Pi**  
   Follow the official installation instructions: [Ubuntu for Raspberry Pi](https://ubuntu.com/download/raspberry-pi)

   > **Recommended**: Ubuntu 22.04 LTS (Jammy) for ROS 2 Humble compatibility

2. **Install ROS 2 on the Raspberry Pi**  
   Follow the binary installation guide: [ROS 2 Humble Binary Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

   > **Important**: Install the full desktop version (`ros-humble-desktop`) or at minimum the base version (`ros-humble-ros-base`)

### Connecting Raspberry Pi and Create3 via USB-C

1. **Physical Connection**  
   Connect your Raspberry Pi to the Create3's USB-C port using a USB-C cable.

2. **Follow Setup Guide**  
   Follow [Jim Bennett's guide](https://jimbobbennett.dev/blogs/irobot-create3-connect-a-pi/) for detailed instructions on configuring USB-C communication between your Raspberry Pi and the Create3.

### Configure ROS 2 Domain ID

The ROS 2 Domain ID must match between your Raspberry Pi and Create3 for them to communicate.

#### On the Raspberry Pi:

1. Open your `.bashrc` file:

```bash
   nano ~/.bashrc
```

2. Add the following line (use domain ID 0 unless you have a specific reason to use another):

```bash
   export ROS_DOMAIN_ID=0
```

3. Save the file (`Ctrl+O`, `Enter`, `Ctrl+X`) and apply the changes:

```bash
   source ~/.bashrc
```

#### On the Create3:

1. **Find the Create3's IP address**:
   
   **If connected via USB-C**, the Create3 has a fixed IP address:
    ```
       192.168.186.2
    ```
   **If connected via WiFi**, find the IP address by:
   - Checking your router's DHCP client list for a device named `iRobot-XXXX` (where XXXX is part of the serial number)
   - Pinging the robot by hostname: `ping iRobot-XXXX.local` (replace XXXX with your robot's identifier)
   - Using the Bluetooth `create3_ip_address.py` example from [python.irobot.com](https://python.irobot.com)

2. **Access the Create3 Web Interface**:
   - Open a web browser and navigate to: `http://<CREATE3_IP_ADDRESS>`
   - For USB connections: `http://192.168.186.2`
   - For WiFi: Use the IP address found in step 1

3. **Configure the Domain ID**:
   - Navigate to the **Application** → **Configuration** section
   - Set the **ROS 2 Domain ID** to match your Pi (e.g., `0`)
   - Optionally set the **ROS 2 Namespace** if needed (default is usually fine)
   - Click **Save** and then **Restart Application**

### Verifying the Connection

After configuration, verify that your Raspberry Pi can see the Create3's ROS 2 topics:

1. **On the Raspberry Pi**, open a terminal and run:

```bash
   ros2 topic list
```

2. You should see Create3-related topics such as:

```
   /battery_state
   /cmd_audio
   /cmd_lightring
   /cmd_vel
   /dock
```

(among others)

> **Troubleshooting**: If you don't see any topics:
>
> - Verify both devices are on the same network (if using WiFi) or properly connected via USB-C
> - Double-check that the ROS_DOMAIN_ID matches on both devices
> - Restart both the Raspberry Pi and Create3
> - Ensure your ROS 2 installation is sourced: `source /opt/ros/humble/setup.bash`

## Setting up ROS2

### ROS2 Workspace

To set up the ROS2 workspace, you have two options:

1. **Official Documentation**: Follow the [Creating a Workspace - ROS2 Documentation](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

2. **Video Tutorial** (Recommended): Watch ["ROS2 Tutorial – ROS2 Humble Crash Course"](https://www.youtube.com/watch?v=Gg25GfA456o) by Robotics Back-End, from **21:29 to 29:18**, which provides a clear walkthrough of setting up a ROS2 workspace.

### Creating Your Robot Controller Package

After setting up your workspace, create a Python package named `my_robot_controller`:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_controller --build-type ament_python
```

You should end up with a `ros2_ws` directory that has the following structure:

```
ros2_ws/
└── src/
    └── my_robot_controller/
        ├── my_robot_controller/
        │   └── __init__.py
        ├── resource/
        │   └── my_robot_controller
        ├── test/
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## Running Example Code

To run the programs provided in the software directory of this repository:

1. **Copy the Python file** (e.g., `creepy_robot.py`) into the **inner** `my_robot_controller` directory (the one containing `__init__.py`).

   ```
   src/
   ├── my_robot_controller/
   │   ├── my_robot_controller/
   │   │   ├── __init__.py
   │   │   ├── creepy_robot.py
   │   ├── resource/
   │   ├── test/
   │   ├── package.xml
   │   ├── setup.cfg
   │   └── setup.py
   ```

2. **Update your package configuration** by ensuring the correct dependencies for the example code are in your `package.xml`:

   ```xml
   <?xml version="1.0"?>
   <package format="3">

   <!-- existing content  -->

   <depend>rclpy</depend>
   <depend>geometry_msgs</depend>
   <depend>irobot_create_msgs</depend>
   <exec_depend>python3-pygame</exec_depend>

   <!-- existing content  -->

   <export>
      <build_type>ament_python</build_type>
   </export>
   </package>
   ```

3. **Add the entry point** in your `setup.py`:

   ```python
   # ... existing imports and content ...

   setup(
      # ... existing parameters ...
      tests_require=['pytest'],
      entry_points={
         'console_scripts': [
               'creepy_robot = my_robot_controller.creepy_robot:main',
         ],
      },
   )

   ```

### Building the Program

1. Navigate to the **top level** of your `ros2_ws` directory:

   ```bash
   cd ~/ros2_ws  # Adjust path if your workspace is located elsewhere
   ```

2. Build your workspace:

   ```bash
   colcon build --symlink-install
   ```

   > **Note**: The `--symlink-install` flag allows you to modify Python scripts without rebuilding.

3. **Source your workspace**:
   ```bash
   source install/setup.bash
   ```

### Running the Program

Now you can run your program:

```bash
ros2 run my_robot_controller creepy_robot
```

> **Troubleshooting**: If you get a "package not found" error, make sure you've sourced the workspace in your current terminal (`source install/setup.bash`).

## 🤖 Example Robots

### Bubbles

<img src="Images/Bubbles.jpeg" alt="Bubbles" width="200"/>
Bubbles is a cute, child-friendly robot with smooth movements and rounded features. Ideal for school and playful environments.

### Spike

<img src="Images/Spike.jpeg" alt="Spike" width="200"/>
Spike is angular and intimidating, designed for security and patrol tasks, featuring jerky movements and grumpy expressions.

### BeachBot

<img src="Images/BeachBot.jpeg" alt="BeachBot" width="200"/>
BeachBot is crafted from cardboard and Lego, designed to assist beachgoers with safety info and navigation.

## 🙏 Acknowledgements

We gratefully acknowledge the contributions of students and participants involved in the co-design workshops that shaped the development of the Roaming Roombas platform. Their creativity, diverse use case ideas, and design insights were instrumental in validating the feasibility and adaptability of the system.

**With special thanks to** the developers of **Bubbles**, **Spike**, and **BeachBot** for their innovative prototypes and commitment to accessible social robots. This project would not have been possible without their enthusiasm and imagination.
