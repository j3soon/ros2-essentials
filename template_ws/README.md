# template_ws

This template will help you set up a ROS-Humble environment quickly.

## 🌱 Structure 🌱

Here is the structure of this template:

```
template_ws
├── .devcontainer
|   ├── cache
|   |   └── .gazebo
|   |       ├── .gitkeep
|   |       └── ...
|   ├── .bashrc
|   ├── devcontainer.json
|   ├── postCreateCommand.sh
|   └── Dockerfile
├── install
├── build
├── log
├── src
|   ├── minimal_pkg
|   |   ├── include
|   |   ├── minimal_pkg
|   |   ├── scripts
|   |   └── ...
|   └── ...
└── README.md
```

> ```build``` / ```install``` / ```log``` folders will appear once you've built the packages.

## 🚩 How to use this template 🚩

Copy the ```template_ws``` directory and follow the instructions below.

### 0. Update the folder name to match your project's name.

Ensure you update the folder name ( ```template_ws``` ) after copying the folder immediately.

### 1. Configure `devcontainer`

> File paths:
> - ```.devcontainer/devcontainer.json```
> - ```docker/docker-compose.yaml```

> TODO: Update the instructions below for docker-compose.

- Update the ```"name"``` to match your project.
- Verify that the ```"runArg"``` matches your requirements.
    - These arguments define the container settings and should align with your ```docker run``` command.
- Review the ```"containerEnv"``` settings:
    - These are used to configure environment variables. Just so you know,  
      variables set here cannot be changed once the container is built,  
      so avoid putting variables here if you need to modify them frequently.
    - If you wish to set a variable that you want to change after the container is built,  
      use ```export VARIABLE=/the/value``` in ```.devcontainer/.bashrc```.
- Update the path in ```"workspaceFolder"```
    - Replace the ```.../template_ws``` with the name of the folder you set in section 0.

### 2. Install the tools you want to use in the container

> File path : ```.devcontainer/postCreateCommand.sh```

Note that the script will be executed after the container-building process.  
Feel free to add any desired commands here !!!

Example:

```bash=
sudo apt-get update && apt-get install -y \
    ros-humble-xxx \
    ros-humble-xxx
```

> You can include the commands directly in the Dockerfile as well.

### 3. Open the workspace folder using Visual Studio Code.

> Haven't set up the devcontainer yet ?
> 
> Please refer to the tutorial provided by Visual Studio Code first.
> You can find it here:  [https://code.visualstudio.com/docs/devcontainers/containers](https://code.visualstudio.com/docs/devcontainers/containers)

Spotting the workspace folder within your Explorer indicates that you've selected the wrong folder.  
You should only observe the ```.devcontainer``` and ```src``` folders there.

**Note**: Alternatively, you can use `docker-compose` instead of `devcontainers` if you are not using VSCode.

### 4. Build the container

Press ```F1``` and enter ```> Dev Containers: Rebuild Container.```.  
Building the images and container will take some time. Please be patient.

You should see the output below.

```
Done. Press any key to close the terminal.
```

**Note**: To save time, you can pull the pre-built Docker images instead of building them from scratch.

### 5. Start to develop with ROS

You've successfully completed all the instructions.  
Wishing you a productive and successful journey in your ROS development !

## ⚠️ Warning ⚠️

- Do not place your files in any folder named ```build```, ```install```, or ```log```. These folders will not be tracked by Git.
- If you encounter an error when opening Gazebo, consider closing the container and deleting the cache folder in the ```.devcontainer```.
