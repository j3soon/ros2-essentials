# Third Party Libraries

This package contains all common third-party software libraries and packages. These are managed through git submodules.

To update the submodules in *interbotix_ros_toolboxes* for the *first time*, run the following command:

```bash
git submodule update --init --recursive 
```

To update the submodules in *interbotix_ros_toolboxes* for any other time, run the following command:

```bash
git submodule update --recursive --remote
```
