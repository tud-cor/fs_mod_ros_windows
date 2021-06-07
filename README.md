# fs_mod_ros_windows

## Overview

This repository contains ROS nodes needed to run in Windows.


## Requirements

* Windows 10
* Python `3.8.5` (other versions have not been tested)

A ROS installation on Windows is not needed as [rospypi/simple](https://github.com/rospypi/simple) is used.


## Building

Clone the repository or download the `.zip` at any location

```cmd
git clone https://github.com/tud-cor/fs_mod_ros_windows.git
```

#### Creating the symbolic link

In order to exchange data with the Python side of the mod, the script uses a symbolic link to a *named pipe*.
This named pipe needs to be created *only once*. Do not create it every time the FarmSim is rebooted.

Open a `cmd` window and run the following command:

```cmd
mklink "%USERPROFILE%\Documents\My Games\FarmingSimulator2019\mods\modROS\ROS_messages" \\.\pipe\ROS_messages
```

If you get the message *You do not have sufficient privilege to perform this operation*, right-click the Command Prompt shortcut, and select __Run as administrator__ to start an elevated shell. Then try creating the symbolic link again.

#### Setting up Python 3 `virtualenv`

```cmd
REM create a virtual environment
python -m venv venv_modROS

REM activate the venv
venv_modROS\Scripts\activate.bat

REM change the active directory to the location of fs_mod_ros_windows
cd path\to\fs_mod_ros_windows

REM install Python dependencies
pip install -r nodes\requirements.txt
```


## Running

### Types of messages transmitted between FarmSim and ROS
There are four types of data which can be exchanged from farmsim and be published as ROS messages:

 - `rosgraph_msgs/Clock`: in-game simulated clock which is not linked to real-time clock. However, the timescale can be configured within Farming Simulator to be real time or 5x, 15x, 30x, 60x or 120x faster than real-time. The `Clock` message stops being published when the game is paused/exited
 - `nav_msgs/Odometry`: ground-truth `Pose` and `Twist` of vehicles based on the in-game position and orientation
 - `sensor_msgs/LaserScan`: a simulated multi layer lidar
 - `sensor_msgs/Imu`: a simulated IMU


The FarmSim mod subscribes to only a single topic (for now):

 - `geometry_msgs/Twist`: controls the currently active vehicle in FarmSim (this is also used by `move_base` when using the navigation stack)


The following are step-by-step instructions allow you to run ROS nodes in Windows 10. There are two parts of the instructions.
One is for publishing data, the other is for subscribing data.


#### Publishing data

Before starting the node, make sure a `roscore` is running (either locally or remotely)

```cmd
REM open one cmd window, activate the base virtual environment
venv_modROS\Scripts\activate.bat

REM go to the directory where all_in_one_publisher.py is located
cd path\to\fs_mod_ros_windows\nodes
```

If you would like to run the navigation stack, start the script with following remapping argument:

```cmd
python all_in_one_publisher.py tf:=fstf
```

If you are not going to run the navigation stack, simply run:

```cmd
python all_in_one_publisher.py
```


#### Subscribing data

Open another cmd window, activate the base virtual environment you created and run the script:

```cmd
venv_modROS\Scripts\activate.bat
cd path\to\fs_mod_ros_windows\nodes
python cm_vel_subscriber.py
```
