# dg_description ROS2 Package 

## 📌 Overview

The `dg_description` package provides visualization tools and robot description files (URDF/Xacro). 
This package enables easy visualization of robot models in RViz, allowing users to validate and inspect the robot geometry and joint configurations.



## 🖥️ Visualization Usage

This package includes RViz-based launch files for visualizing the Delto Gripper-5F models. Follow the instructions below to visualize each variant:

### 🚀 Launch Left Delto Gripper-5F Visualization

Run the following command to visualize the Left Delto Gripper-5F:

```bash
ros2 launch dg_description dg5f_left_display.launch.py
```

### 🚀 Launch Right Delto Gripper-5F Visualization

Run the following command to visualize the Right Delto Gripper-5F:

```bash
ros2 launch dg_description dg5f_right_display.launch.py
```


## 📁 Included Files

- **Launch files**: 
  - `dg5f_left_display.launch.py`
  - `dg5f_right_display.launch.py`

- **Robot Description Files (URDF/Xacro)**:

- **Meshes**:
  - Detailed collision and visual mesh files for robot visualization.



## 📄 License

This project is distributed under the BSD-3-Clause license, promoting open-source collaboration and reuse.



## 📧 Contact

For support or inquiries regarding this package, please reach out to [TESOLLO SUPPORT](mailto:support@tesollo.com).
