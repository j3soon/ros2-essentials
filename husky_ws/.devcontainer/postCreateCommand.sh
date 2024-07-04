sudo apt-get update --fix-missing
sudo rosdep update
# Note: The following commands are commented out to prevent unintended install/builds.
# sudo rosdep install --from-paths src --ignore-src --rosdistro humble -y
# sudo chown -R user /home/ros2-agv-essentials/
# colcon build