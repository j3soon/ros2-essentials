# ROS 2 Essentials Workspace

Please visit <https://j3soon.github.io/ros2-essentials/> for documentation.

# Quick README

## Launch and Development
1. build the workspace at stretch3_ws if you changed anything
```
colcon build
```
> the dependencies of the usda and usdz is very trick, some in install and some in src, **it is very important to not delete any exist file and keep the project structure**
> since I have know idea what is depend on original src what is depend on install, it is better to rebuild the workspace

2. launch via stretch3_bringup
```
ros2 launch stretch3_bringup stretch3_bringup.launch.py
```

3. Do what ever you want in isaacsim
4. Go to `File` -> `Save`, if you see `stretch_movable.usda` is marked `modified` by Git, you have saved your modifiy.

## Important file
1. isaacsim/model/stretch_movable.usda
    - description of the stretch3 isaacsim environment, launch it with isaccsim and you can have a movable stretch3 in a clean playground
    - it is depend on `stretch.project.usdz`
2. isaacsim/model/stretch.project.usdz
    - the base project dependency, I have no idea why.
3. stretch3_bringup/launch/stretch3_bringup.launch.py
    - the launch file of the environment
    - if you want to open other usdz or usda, you can modify line 21-38
    1. if use this line, you will open usd in install
    ```
    stretch3_usd_path = PathJoinSubstitution([FindPackageShare("isaacsim"), "model", "stretch_movable.usda"])
    ```
    2. if use these line, you will open usd in original src
    ```
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.abspath(os.path.join(launch_file_dir, "../../../../../"))
    stretch3_usd_path = os.path.join(workspace_root, "src/isaacsim/model/stretch_movable.usda")
    ```
    **again, I have no idea about the dependency chain, so be careful, always keep the orginal file and backup the file**