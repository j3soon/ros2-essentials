# template_ws

This template will help you set up a ROS-Humble environment quickly.

## 🌱 Structure 🌱

Here is the structure of this template:

```
ros2-agv-essentials
├── scripts
|   └── create_workspace.sh
├── template_ws
|   ├── .devcontainer
|   |   ├── devcontainer.json
|   |   └── postCreateCommand.sh
|   ├── docker
|   |   ├── cache
|   |   |   └── .gazebo
|   |   ├── .bashrc
|   |   ├── compose.yaml
|   |   └── Dockerfile
|   ├── install
|   ├── build
|   ├── log
|   ├── src
|   |   ├── minimal_pkg
|   |   |   ├── include
|   |   |   ├── minimal_pkg
|   |   |   ├── scripts
|   |   |   └── ...
|   |   └── ...
|   └── README.md
```

- `build` / `install` / `log` folders will appear once you've built the packages.
- `minimal_pkg` is the ROS2 package used to create a publisher and subscriber in both Python and C++. You can remove it if you don't need it.

## 🚩 How to use this template 🚩

### 1. Use the script to copy the template workspace.

We have provided a script to create a new workspace. Please use it to avoid potential issues.  

```bash
# Open a terminal, and change the directory to ros2-agv-essentials.
./scripts/create_workspace.sh <new_workspace_name>
```

> To unify the naming style, we will modify the string `<new_workspace_name>` in some files.

### 2. Configure Docker settings.

- `docker/Dockerfile`
    - Check whether the default Dockerfile meets your requirements. If not, feel free to modify the contents. In most cases, you may want to install additional tools for ROS packages.
- `docker/compose.yaml`
    - If you want to access the GPU in the container, please uncomment the lines accordingly.
    - If you want to add any environment variables in the container, you can include them in the `environment` section, or you can use `export VARIABLE=/the/value` in `docker/.bashrc`.
- `.devcontainer/devcontainer.json`
    - Update the `name` to match your project.
    - Add any extensions you want in the `Vscode extensions`.
    - If you change the service name in `compose.yaml`, please update the setting in the `service` section accordingly. 
- `.devcontainer/postCreateCommand.sh`
    - This script will be executed after the container-building process. Feel free to add any desired commands here.

### 3. Open the workspace folder using Visual Studio Code.

> Haven't set up the devcontainer yet ?
> 
> Please refer to the tutorial provided by Visual Studio Code first.  
> You can find it here:  [https://code.visualstudio.com/docs/devcontainers/containers](https://code.visualstudio.com/docs/devcontainers/containers)

Spotting the workspace folder within your Explorer indicates that you've selected the wrong folder.  
You should only observe the `.devcontainer`, `docker` and `src` folders there.

**Note**: Alternatively, you can use `docker compose` instead of `devcontainer` if you are not using VSCode.

### 4. Build the container.

Press `F1` and enter `> Dev Containers: Rebuild Container`.  
Building the images and container will take some time. Please be patient.

You should see the output below.

```
Done. Press any key to close the terminal.
```

### 5. Start to develop with ROS.

You've successfully completed all the instructions.  
Wishing you a productive and successful journey in your ROS development !

## ⚠️ Warning ⚠️

- Do not place your files in any folder named `build`, `install`, or `log`. These folders will not be tracked by Git.
- If you encounter an error when opening Gazebo, consider closing the container and deleting the cache in the `docker/cache` directory. Please note that you should only delete the files inside the folder and not the entire cache folder.
