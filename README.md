# Centralized-Control-System

### Pre_Requirement
- ROS2 Jazzy Jalisco
- Colcon build
  ```
  sudo apt install python3-colcon-common-extensions
  ```

### How to use this Project
Source ROS2 Humble
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
This make you doesn't need to source ROS2 Humble every time you open terminal
1. Clone this Github to the workspace
  ```bash
  git clone https://github.com/wchawaphon/Centralized-Control-System.git
  cd ~/Centralized-Control-System/centralized_ws
  ```
2. Check Dependencies
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
If it has output like below you can move to the next step
  ```bash
  #Output
  #All required rosdeps installed successfully
  ```
3. Build and Source the packages
  ```bash
  colcon build
  source install/setup.bash
  ```
4. Run all node in separate terminal
  ```bash
  #Terminal 1
  cd ~/Centralized-Control-System/centralized_ws
  source install/setup.bash
  ros2 run central_control central_control_node

  #Terminal 2
  cd ~/Centralized-Control-System/centralized_ws
  source install/setup.bash
  ros2 run manipulator_node manipulator_node

  #Terminal 3
  cd ~/Centralized-Control-System/centralized_ws
  source install/setup.bash
  ros2 run amr_node amr_node

  #Terminal 4
  cd ~/Centralized-Control-System
  uvicorn ros_web_backend:app --reload --port 8000
  ```
For the GUI 
Open file `index.html` in browser

