# Uncertainty Aware-Predictive Control Barrier Functions: Safer Human Robot Interaction through Probabilistic Motion Forecasting

This repository contains the code for the UAPCBF project. The project deals with safe interaction between robot and human in the context of a collaborative cell. This includes:

* Tracking the position of the operator's hand through a Leap Motion v2 camera
* Forecasting the future position of the operator's hand through a deep-learning module
* Safely planning the trajectory of the robot's end-effector through a custom Control Barrier Function formulation

The framework and its applications are detailed in a paper: TBD

## Table of contents

- [Uncertainty Aware-Predictive Control Barrier Functions: Safer Human Robot Interaction through Probabilistic Motion Forecasting](#uncertainty-aware-predictive-control-barrier-functions-safer-human-robot-interaction-through-probabilistic-motion-forecasting)
  * [Setup](#setup)
    + [LeapC Python bindings](#leapc-python-bindings)
    + [ROS2](#ros2)
    + [JAKA SDK](#jaka)
  * [Hand mockup experiment](#hand-mockup-experiment)
  * [Operator experiment](#operator-experiment)

## Setup

The framework was developed and tested on a machine running Ubuntu 24.04. 

<b>It is highly suggested to set up a virtual environment to run the following setup</b>, as some packages require specific versions of core packages (eg NumPy).

### LeapC Python bindings

To install LeapC's Python bindings, used in hand tracking and forecasting, follow these steps:

1. Download and install [Gemini Ultraleap Hand Tracking Software](https://www.ultraleap.com/downloads/leap-motion-controller-2/)
2. Clone and cd into the [LeapC Python Bindings](https://github.com/ultraleap/leapc-python-bindings) repository, and execute:
```
pip install -r requirements.txt
pip install -e leapc-python-api
```

### ROS2

Install ROS2 and create a workspace (see [here](https://docs.ros.org/en/kilted/index.html)). Clone this repository into the src folder and build.

Cd into the src folder and install the project's required packages:
```
pip install -r requiremtns.txt
```

### JAKA SDK

The recommended version of JAKA's Python SDK is included in jaka_interface/lib. To make it available, add the path to the following env variables:


```
export PYTHONPATH=$PYTHONPATH:{ros2WorkspaceFolder}/src/jaka_interface/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{ros2WorkspaceFolder}/src/jaka_interface/lib

```


## Hand mockup experiment

TODO

## Operator experiment

TODO

