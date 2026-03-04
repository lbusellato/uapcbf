## Execution and ROS Nodes

This section details the steps required to set up the Leap Motion device in WSL, build the ROS workspace, launch the applications, and monitor the data topics. 

**leap_motion_screen is more commented to help to read the code**

-----

### 1. Setup on Windows Subsystem Linux (WSL)

To bind the Leap Motion USB device to your WSL environment, execute the following commands in **PowerShell as Administrator**:

1.  **List USB Devices** (Identify the correct Bus ID for your Leap Motion):
    ```powershell
    usbipd list
    ```
2.  **Bind the USB Device** (Replace `4-4` with the correct Bus ID):
    ```powershell
    usbipd bind --busid 4-4
    ```
3.  **Attach the Device to WSL** (Ensure the device is accessible within Ubuntu):
    ```powershell
    usbipd attach --wsl --busid 4-4
    ```

-----

### 2. Build and Source the ROS Workspace (Ubuntu)

Navigate to your workspace and run the build and sourcing commands:

1.  Navigate to the **colcon workspace**:
    ```bash
    cd colcon_ws
    ```
2.  **Build the project**:
    ```bash
    colcon build --symlink-install
    ```
3.  **Source the setup file** (Essential for finding executables and packages):
    ```bash
    . install/setup.bash
    ```

-----

### 3. Run the Applications

Launch the main ROS nodes using the package name and the executable name:

```bash
ros2 run leap_gesture_recognition leap_gesture_recognition
ros2 run leap_motion leap_motion
```

-----

### 4. Visualize Topic Output

To verify that the Leap Motion node is streaming data and check the format of the joint coordinates, use the `ros2 topic echo` command:

```bash
ros2 topic echo /leapmotion/joints
```

-----

### Troubleshooting and Dependencies

  * **If the Build/Libraries Break:**
    To clean the workspace and force a complete rebuild, remove the build, install, and log directories before running `colcon build` again:
    ```bash
    cd colcon_ws/
    rm -rf build/ install/ log/
    ```
  * **Required Library Installation:**
    Ensure you have correctly installed the necessary dependencies using the instructions for the LeapC Python Bindings:
    [https://github.com/ultraleap/leapc-python-bindings/tree/main](https://github.com/ultraleap/leapc-python-bindings/tree/main)