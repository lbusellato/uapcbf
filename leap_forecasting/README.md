# Forecasting Node for CollaborICE

## Requirements
- ROS2 (tested on jazzy-ros2 on WSL2, Ubuntu 24.04)
- Python3 (with dependencies correctly installed); any version of python3.10+ should work

## Installation
1. Clone the repository
2. Install the python dependencies (`pip install -r requirements.txt`)
3. (optional) if you want install it as global package, put this folder in your ROS2 workspace (usually `~/ros2_ws`) before building with `colcon`
4. Install dependencies (such as rclpy and std_msgs) using rosdep
    `rosdep install -i --from-path src --rosdistro jazzy -y`
4. Open a shell, activate your target python3 envoironment, then cd inside this directory (or ws, see above), and build the package using ros2 
    `colcon build --packages-select collaborice_forecasting_node`
5. Source the workspace 
    `source install/setup.bash`


## Usage
Run the node `ros2 run collaborice_forecasting_node collaborice_forecasting_node`
or
`ros2 launch collaborice_forecasting_node nn_forecasting.launch.py debug:=true`


# PER DUMMY
leggi setup.py e guarda entry_points e console_scripts
poi fai 
```ros2 run collaborice_forecasting_node <nome_script>```
