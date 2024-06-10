# px4_driver
ROS2 driver for px4 based drones.

## Prerequisites

- ROS2 Humble installed. Installation steps can be found *[here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)*.
- MicroXRCE-DDS Agent. To do so, run the following commands:
```sh
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

You can find more info and other ways to install it by checking *[this](https://docs.px4.io/main/en/middleware/uxrce_dds.html)* link on PX4 main documentation.

## Installation

Create a directory for ROS2 Workspace.
```sh
mkdir -p ~/px4_ws/src/
cd ~/px4_ws/src/
```

Clone PX4 msgs repo.
```sh
git clone https://github.com/PX4/px4_msgs.git
```

Clone this repo.
```sh
git clone https://github.com/elTobe/px4_driver.git
```

Source your ROS2 installation and build the ROS2 Workspace.
```sh
cd ~/px4_ws/
source /opt/ros/humble/setup.bash
colcon build
```

## Usage

Source your ROS2 installation if you haven't done it before.
```sh
source /opt/ros/humble/setup.bash
```

Run the MicroXRCE-DDS Agent. Assuming you have connected PX4 to Raspberry Pi over UART pins, the command should be : 
```sh
sudo MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

To check that the connection is working fine, you can list topics created by the agent, all of them starting with /fmu/*.
```sh
ros2 topic list
```

Source your local installation.
```sh
cd ~/px4_ws/
source install/local_setup.bash
```

Run the node.
```sh
ros2 run px4_driver px4_driver_node
```

## Useful Links

- *[ROS2 Humble installation.](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)*
- *[ROS2 User Guide on PX4 main documentation.](https://docs.px4.io/main/en/ros/ros2_comm.html)*
- *[Raspberry Pi to PX4, setup and connection.](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html)*