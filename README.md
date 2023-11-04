# colcon_ws

Simple overview of use/purpose.

## Description

An in-depth paragraph about your project and overview of use.

## Getting Started

### Dependencies

* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* ex. Windows 10

### Installing VM and ROS2 (foxy)

* follow the ROS-foxy guide at https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

### Setting up

* clone the repository into your $HOME
```
cd ~
git clone <ssh url>
```
* create a new branch for your own work space
  e.g.
```
git checkout -b Alan/workspace
```
* On your branch, run...
```
cd ~/colcon_ws
colcon build
```
> **_NOTE:_** DON'T RUN COLCON BUILD ON THE MAIN BRANCH
