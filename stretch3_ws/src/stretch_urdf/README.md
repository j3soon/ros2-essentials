# Overview

The Stretch URDF package provides URDF and Mesh files for use by Stretch Body. 
This URDF data is uncalibrated and is managed completely seperately from the Stretch ROS(2) URDF data. 

This package can be installed by:

```
python3 -m pip install  -U hello-robot-stretch-urdf
```

The URDF and mesh data is installed as a Python package. It's location can be found as:

    import importlib.resources as importlib_resources
    pkg = str(importlib_resources.files("stretch_urdf"))

The URDF naming convention is `stretch_description_<model_name>_<tool_name>.urdf`
For example:

    model_name = robot.params['model_name']
    tool_name = robot.params['tool']
    urdf_name = pkg + '/%s/stretch_description_%s_%s.urdf' % (model_name, model_name, tool_name)

For further example usesage, see the included `stretch_urdf_example.py`
