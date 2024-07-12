<h1 align="center">
<img src="./images/splash.png" style="max-width:100%;">
</h1><br>

[![ROS Build](https://github.com/Warwick-Racing/wrai_ws/actions/workflows/ros.yml/badge.svg?branch=main)](https://github.com/Warwick-Racing/wrai_ws/actions/workflows/ros.yml)

The [Warwick Racing](https://warwickracing.org/) AI Team compete in the Formula Student AI (FS-AI) competition.

This repo contains the production-ready code for the AI car. 

## Installation

Prerequisites: 
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
* [Rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)

Clone the repo with `git clone`, then `cd` into the repo directory. Make sure the ROS2 setup script is `source`d. Install dependencies with `rosdep install --from-paths . -y --ignore-src` (making sure `rosdep` has been `init`ed and `update`ed). Build all the packages with `colcon build`.

## Usage

Launch the software stack with `ros2 launch launch/xyz.launch.py`.

## Contributing

Team members should make changes in feature branches (one branch for each major feature you are working on). Once a feature is production-ready, submit a pull request so the rest of the team can review your code and suggest changes.

## License

All rights reserved. 
