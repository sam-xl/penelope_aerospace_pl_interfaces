# PeneloPe Aerospace Pilot Line Interfaces
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)  

## Overview 
ROS 2 packages with action and message definitions, intended for use with the Digital Thread of the PeneloPe Aerospace Pilot Line. This repository also includes an example action server and client for the `InfraredThermographyInspect` action that can serve as a baseline for developing the module action servers. 

**Authors: E. Bernardi (SAM XL, TU Delft), D. Kroezen (SAM XL, TU Delft)**

**Affiliation: [SAM XL](https://www.samxl.com/), [TU Delft](https://www.tudelft.nl/)**

This package hase been developed and tested under `ROS 2 Humble` and `Ubuntu 22.04`.

## Installation

### Building
To build from source, clone the lastest version from this repository into your ROS 2 workspace:

    cd ros2_ws/src
    git clone https://github.com/sam-xl/penelope_aerospace_pl_interfaces.git

Always source `ros2` installation in a new terminal before installaing dependencines and building pkgs:

    source /opt/ros/humble/setup.bash

Install the dependencies of the cloned packages using `rosdep`:

    rosdep install --from-paths src -y --ignore-src

Finally, build all packages in the workspace:

    colcon build

A `stderr` can be shown for the `penelope_aerospace_pl_interfaces_examples` package while building the package which warns for use of a deprecated function in the `setup.py` file. This can be ignored.

## Usage
The `penelope_aerospace_pl_interfaces_msgs` package is used as a description of the interfaces between the digital thread and the pilot line processes. 

**All process partners** need to use and implement the relevant interfaces to create the action servers needed for communication and coordination with the digital thread. SAM XL will implement the client side of the action interface. The action server and client needs to be implemented before the integration phase of the process on the gantry (Q1 2024).

This repository contains an example server and client that demonstrates one of the action interfaces that will be used. This can serve as a baseline for implementation of the interface in the process modules and digital thread. The example focusses only on how to interact with the action interface and send/read the goal, feedback and result. The process logic is not part of the example.

### Run example server and client
Always source the newly compiled pkgs from the local `install` folder before running them:

    source install/setup.bash
    
To run the action server (module side of the interface):

    ros2 run penelope_aerospace_pl_interfaces_examples example_server

To run the action client example (digital thread side of the interface):

    ros2 run penelope_aerospace_pl_interfaces_examples example_client

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/sam-xl/penelope_aerospace_pl_interfaces/issues).

> :warning: Note: Changes to the interfaces can also be requested via the issue tracker. 
