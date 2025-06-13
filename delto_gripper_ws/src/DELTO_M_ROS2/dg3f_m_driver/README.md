# dg3f_m_driver ROS 2 Package 🚀

## 📌 Overview

The `dg3f_m_driver` ROS 2 package provides a hardware interface leveraging [ros2_control](https://control.ros.org/) for the DG-3F-M grippers, enabling direct robotic control operations.

## 📦 Dependency Installation

### Navigate to Workspace
```bash
cd ~/your_ws
```

### Update rosdep
```bash
apt update
rosdep update
```

### Install Specific Dependencies
```bash
rosdep install --from-paths src/DELTO_M_ROS2/dg3f_m_driver --ignore-src -r -y
```

### Verify Installation by Building
```bash
colcon build --packages-select dg3f_m_driver
```

## 🎛️ Controlling Delto Gripper-3F-M

### 1\. Loading a URDF model into Gazebo

Launch the Left Delto Gripper-3F-M controller with:
```bash
ros2 launch dg3f_m_driver dg3f_m_driver.launch.py
```

### 2\. Here's a simple topic-based control command example:
-  **Python Example**: [dg3f_m_test.py](script/dg3f_m_test.py)

Run the Python test script:
```bash
ros2 run dg3f_m_driver dg3f_m_test.py
```

- 💻 **C++ Example**: [dg3f_m_test.cpp](src/dg3f_m_test.cpp)

Run the CPP test code:
```bash
ros2 run dg3f_m_driver dg3f_m_test_cpp
```

## 🤝 Contributing
Contributions are encouraged:

1. Fork repository
2. Create branch (`git checkout -b feature/my-feature`)
3. Commit changes (`git commit -am 'Add my feature'`)
4. Push (`git push origin feature/my-feature`)
5. Open pull request

## 📄 License
BSD-3-Clause

## 📧 Contact
[TESOLLO SUPPORT](mailto:support@tesollo.com)

