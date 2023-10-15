# Efr-ROS2-autonomous-driving
This is a task repository for the Elbflorace Formula Student team - autonomous driving division.
The following are the required steps in order to set up [ROS2](https://docs.ros.org/en/iron/index.html) along with turtlesim and rqt.

### Contents
- [Setup](#setup)
- [Introduction Task](#introduction)

## Setup
This setup is made primarily for Windows platforms, for Ubuntu WSL and GWSL are **not** required.
### Downloads
| Tool              | Platform |                                                         Link |
| :---------------- | :------: | -----------------------------------------------------------: |
| WSL2              | Windows  | [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) |
| GWSL *(optional)* | Windows  |                      [GWSL](https://opticos.github.io/gwsl/) |

<p> Now that the pre-requisites are done for Windows, we can setup and configure the simulator.

### ROS2
[Link](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) for the official installation guide on Ubuntu systems. 

**Enable** repositories
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
- Add the ROS 2 GPG key with apt.
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
- Add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
- Install development tools *(required in order to build packages)*
```
sudo apt update && sudo apt install ros-dev-tools
```
**Install** ROS2
- Quick apt repository caches update before install
```
sudo apt install ros-iron-desktop
```
**Environment** setup
```
# Possible values are: setup.bash, setup.sh, setup.zsh depending on your install
source /opt/ros/iron/setup.bash
```