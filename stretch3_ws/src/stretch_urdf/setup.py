import setuptools
from os.path import isfile
import glob

with open("README.md", "r") as fh:
    long_description = fh.read()

script_path='./tools'
ex_scripts = glob.glob(script_path+'/*.py') + glob.glob(script_path+'/*.sh')
stretch_scripts=[f for f in ex_scripts if isfile(f)]

setuptools.setup(
    name="hello-robot-stretch-urdf",
    version="0.1.2",
    author="Hello Robot Inc.",
    author_email="support@hello-robot.com",
    description="Stretch URDF",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/hello-robot/stretch_urdf",
    package_data={"stretch_urdf": ["RE2V0/meshes/*.STL","RE2V0/meshes/*.stl","RE2V0/meshes/*.dae","RE2V0/xacro/*.xacro","RE2V0/xacro/d435i/*.xacro","RE2V0/xacro/d405/*.xacro","RE2V0/*.urdf",
                                   "RE1V0/meshes/*.STL","RE1V0/meshes/*.dae","RE1V0/xacro/*.xacro","RE1V0/xacro/d435i/*.xacro","RE1V0/*.urdf",
                                   "SE3/meshes/*.STL","SE3/meshes/*.stl","SE3/meshes/*.dae","SE3/xacro/*.xacro","SE3/xacro/d435i/*.xacro","SE3/xacro/d405/*.xacro","SE3/*.urdf"]},
    scripts=stretch_scripts,
    packages=['stretch_urdf'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent"
    ],

    install_requires=['urchin']
)
