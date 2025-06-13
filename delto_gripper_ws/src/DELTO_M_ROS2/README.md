<img src="/dg_description/image/title.svg"/>

<img src="/dg_description/image/dg5f_1.png" alt="dg5f" width="300px"/> <img src="https://en.tesollo.com/wp-content/uploads/2024/12/DG-5F-6-1.webp" width="300px"/> <img src="https://tesollo.com/wp-content/uploads/2024/12/DG-3F-5-1.webp" width="200px"/> 



The DELTO_M_ROS2 Repository is a comprehensive ROS 2 package designed to support the Delto Gripper-M. This project includes simulations, control interfaces, and visualization tools for the gripper, enabling developers to efficiently develop and test robotic applications.


## 📌 **Supported ROS Distributions**

|  **ROS Version** |  **Ubuntu Version** |  **Branch** | **Build Status** |
|------------------|----------------------|---------------|---------------|
| ROS 2            | 22.04 (Jammy)        | `humble`        | ![Build Status](https://github.com/Tesollo-Delto/DELTO_M_ROS2/actions/workflows/ci.yaml/badge.svg?branch=humble) |


## 📦 Package Structure

The DELTO_M_ROS2 project comprises the following main packages:
- [**dg3f_m_driver**](/dg3f_m_driver/) ros2 control interfaces for the Delto Gripper-3F.
- [**dg3f_m_gz**](/dg3f_m_gz/): Provides Gazebo simulations and control interfaces for the Delto Gripper-3F. 
- [**dg5f_driver**](/dg5f_driver/) ros2 control interfaces for the Delto Gripper-5F.
- [**dg5f_gz**](/dg5f_gz/): Provides Gazebo simulations and control interfaces for the Delto Gripper-5F.
- [**dg_description**](/dg_description/): Contains URDF models and visualization configurations for the Delto Gripper-5F.
- [**dg_isaacsim**](/dg_isaacsim/):  package provides a demonstration of integrating ROS2 with Isaac Sim.


## 🛠️ Installation and Build Instructions

To install and build the DELTO_M_ROS2 project, follow these steps:

1. **Create Workspace and Clone Source Code**

   ```bash
   mkdir -p ~/your_ws/src
   cd ~/your_ws/src
   git clone <DELTO_M_ROS2_repository_URL>
   ```

2. **Install Dependencies**

   ```bash
   cd ~/delto_m_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build Packages**

   ```bash
   colcon build
   ```

4. **Source the Environment**

   ```bash
   source install/setup.bash
   ```


## 🎯 **Performance Demonstrations**

### **Delto Gripper-3F: Advanced Packaging Solutions**

[![Delto Gripper-3F Advanced Packaging Solutions](https://img.youtube.com/vi/x6QCdgl5r8Q/sddefault.jpg)](https://www.youtube.com/watch?v=x6QCdgl5r8Q)  
▶️ *Click the image to watch the video*


### **Delto Gripper-5F: Paper Cup Removal Demonstration**

[![Delto Gripper-5F Paper Cup Removal](https://img.youtube.com/vi/MlUSlto5R9U/sddefault.jpg)](https://www.youtube.com/watch?v=MlUSlto5R9U)  
▶️ *Click the image to watch the video*

## 🤝 Contributing

The DELTO_M_ROS2 project is open-source, and contributions are welcome. To contribute:

1. Fork this repository.
2. Create a new branch (`git checkout -b feature/my-feature`).
3. Commit your changes (`git commit -am 'Add my feature'`).
4. Push to your branch (`git push origin feature/my-feature`).
5. Open a pull request detailing your modifications.


## 📄 License

This project is released under the BSD-3-Clause license.


## 📧 Contact

For additional support or inquiries about this project, please contact [TESOLLO SUPPORT](mailto:support@tesollo.com). 
