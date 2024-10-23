# Template

This template will help you set up a ROS-Humble environment quickly.

## 🌱 Structure 🌱

Here is the structure of this template:

```
ros2-essentials
├── scripts
|   └── create_workspace.sh
├── template_ws
|   ├── .devcontainer
|   |   └── devcontainer.json
|   ├── docker
|   |   ├── .bashrc
|   |   ├── .dockerignore
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

## 🚩 How to use this template 🚩

### 1. Use the script to copy the template workspace.

We have provided a script to create a new workspace. Please use it to avoid potential issues.  

```bash
# Open a terminal, and change the directory to ros2-essentials.
./scripts/create_workspace.sh <new_workspace_name>
```

> To unify the naming style, we will modify the string `<new_workspace_name>` in some files.

### 2. Configure settings.

> To help you easily find where changes are needed, we have marked most of the areas you need to adjust with `# TODO:`. Usually, you only need to modify the configurations without removing anything. If you really need to remove something, make sure you clearly understand your goal and proceed with caution.

- `docker/Dockerfile`
    - Add the packages you need according to the comments inside.
- `docker/compose.yaml`
    - By default, the Docker image is built according to your current computer's architecture. If you need to cross-compile, please modify the `platforms` parameter to your desired architecture and set up the basic environment.
    - If you want to access the GPU in the container, please uncomment the lines accordingly.
    - If you want to add any environment variables in the container, you can include them in the `environment` section, or you can use `export VARIABLE=/the/value` in `docker/.bashrc`.
- `docker/.bashrc`
    - We will automatically compile the workspace in .bashrc. If you don't want this to happen, feel free to remove it. If you’re okay with it, remember to adjust the compilation commands according to your packages.
- `src`
    - Add the ros packages you need here.
    - `minimal_pkg` is the ROS2 package used to create a publisher and subscriber in both Python and C++. You can remove it if you don't need it.

### 3. Open the workspace folder using Visual Studio Code.

> Haven't set up the devcontainer yet ?
> 
> Please refer to the tutorial provided by Visual Studio Code first.  
> You can find it here:  [https://code.visualstudio.com/docs/devcontainers/containers](https://code.visualstudio.com/docs/devcontainers/containers)

> We recommend using `VScode` + `devcontainer` for development. This plugin can significantly reduce development costs and can be used on local computers, remote servers, and even embedded systems. If you don't want to use `devcontainer`, you can still use Docker commands for development, such as `docker compose up` and `docker exec`.

Open the workspace folder using Visual Studio Code, spotting the workspace folder within your Explorer indicates that you've selected the wrong folder. You should only observe the `.devcontainer`, `docker` and `src` folders there.

### 4. Build the container.

> We have pre-built some Docker images on Docker Hub. If the building time is too long, you might consider downloading them from Docker Hub instead. For more information, please refer to the `README.md` on the repository's main page.

Press `F1` and enter `> Dev Containers: Rebuild Container`.  
Building the images and container will take some time. Please be patient.

You should see the output below.

```
Done. Press any key to close the terminal.
```

> For non-devcontainer users, please navigate to the `docker` folder and use `docker compose build` to build the container. We have moved all commands that need to be executed into the `.bashrc` file. No further action is needed after creating the Docker container.

### 5. Start to develop with ROS.

You've successfully completed all the instructions.  
Wishing you a productive and successful journey in your ROS development !

## ⚠️ Warning ⚠️

- Do not place your files in any folder named `build`, `install`, or `log`. These folders will not be tracked by Git.
- If you encounter an error when opening Gazebo, consider closing the container and reopen it. Alternatively, you can check the log output in `~/.gazebo`, which may contain relevant error messages. The most common issue is using a duplicate port, which prevents Gazebo from starting. You can use `lsof -i:11345` to identify which process is using the port and then use `kill -9` to terminate it.
- `xhost +local:docker` is required if the container is not in privileged mode.
