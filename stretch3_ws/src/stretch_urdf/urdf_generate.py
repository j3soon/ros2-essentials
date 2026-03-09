#!/usr/bin/env python3
import subprocess
import shlex
import sys
import os
import glob

#This script generates
def get_ros_version():
    if 'ROS_DISTRO' in os.environ.keys():
        if os.environ['ROS_DISTRO'] == 'noetic':
            print(f"Found ROS_DISTRO = noetic")
            return 1
        if os.environ['ROS_DISTRO'] == 'humble':
            print(f"Found ROS_DISTRO = humble")
            return 2

ros_version = get_ros_version()

if ros_version is None:
    print("Unable to find a valid ROS Installation")
    sys.exit()

#Factory tool to generate URDFs from Xacros

def run_cmd(cmdstr):
    process = subprocess.run(shlex.split(cmdstr), capture_output=True, text=True)
    if(process.returncode != 0):
        print("update_uncalibrated_urdf.py ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process

def generate_from_descriptions(root_dir,xacro_dir,descriptions):
    urdfs = []
    for d in descriptions:
        if ros_version==2:
            bashCommand = "ros2 run xacro xacro {}".format(xacro_dir + d + '.xacro')
        if ros_version==1:
            bashCommand = f"rosrun xacro xacro {xacro_dir + d}.xacro"

        print(bashCommand)
        process = run_cmd(bashCommand)
        urdf = process.stdout
        urdf_filepath = root_dir + d + '.urdf'
        urdfs.append(urdf_filepath)
        urdf_lines = urdf.split('\n')
        with open(urdf_filepath, "w") as open_file:
            for u in urdf_lines:
                # if u.find('d435.dae') > -1:  # Replace path for realsense
                #     ss = u.find('file://')
                #     u = u[:ss] + './meshes/d435.dae"/>'
                print(u, file=open_file)

            open_file.close()

def get_descriptions(xacro_dir):
    descriptions=[]
    d=glob.glob(xacro_dir+'stretch_description*.xacro')
    for a in d:
        descriptions.append(a[a.rfind('/')+1:-6])
    return descriptions

def main():

    #Descriptions should include all configurations that we officially "support"
    for model_name in ['RE1V0', 'RE2V0', 'SE3']:
        root_dir = './stretch_urdf/'+model_name+'/'
        xacro_dir = root_dir + 'xacro/'
        generate_from_descriptions(root_dir, xacro_dir, get_descriptions(xacro_dir))


if __name__ == '__main__':
     main()
