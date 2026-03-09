#!/usr/bin/env python3

import os
import sys
import subprocess
import shlex
import re
import argparse
import click
import pathlib

print("For use with S T R E T C H (R) from Hello Robot Inc.")
print("---------------------------------------------------------------------\n")

############## Initialize Variables #####################

tool_name = None
model_name = None
root_dir = None
ros_repo_path = None
ros_version = None
data_dir = None


############### Helper Methods ########################

def get_robot_params():
    global tool_name, model_name
    try:
        import stretch_body.robot_params
        tool_name = stretch_body.robot_params.RobotParams().get_params()[1]['robot']['tool']
        model_name = stretch_body.robot_params.RobotParams().get_params()[1]['robot']['model_name']
    except ModuleNotFoundError:
        print("Unable to find stretch tool params. Use --model and --tool to manually configure.")

def run_cmd(cmdstr,verbose=False):
    if verbose:
        print(cmdstr)
    process = subprocess.run(shlex.split(cmdstr), capture_output=True, text=True)
    if(process.returncode != 0):
        print("ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process

def remove_lines_between_patterns(text, start_pattern, end_pattern):
    start_regex = re.compile(re.escape(start_pattern) + r'.*?' + re.escape(end_pattern), re.DOTALL)
    end_regex = re.compile(re.escape(end_pattern) + r'.*?' + re.escape(start_pattern), re.DOTALL)
    cleaned_text = start_regex.sub(start_pattern + end_pattern, text)
    cleaned_text = end_regex.sub(start_pattern + end_pattern, cleaned_text)
    cleaned_text = cleaned_text.replace(start_pattern + end_pattern, "")
    return cleaned_text

def verify_ros():
    global ros_version, ros_repo_path
    ros_version = get_ros_version()

    # This tool can only be used if there is a valid ros installation present
    if ros_version is None:
        print("Unable to find a valid ROS Installation")
        sys.exit()

    if ros_version==2:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
        try:
            p = get_package_prefix('stretch_description')
            ros_repo_path = str(pathlib.Path(p).parent.parent / 'src' / 'stretch_ros2').replace(str(pathlib.Path(p).home()), '~')
            ros_repo_path = os.path.expanduser(ros_repo_path)
        except PackageNotFoundError:
            print("Unable to find stretch_description package.")
            sys.exit(1)
        if not os.path.exists(ros_repo_path):
            print(f"Unable to find path {ros_repo_path}.")
            sys.exit(1)
    else:
        import rospkg
        rospack = rospkg.RosPack()
        p = rospack.get_path('stretch_description')
        ws_paths = [str(par.parent) for par in pathlib.Path(p).parents if str(par).endswith('src')]
        ros_repo_path = ''
        if len(ws_paths) > 0:
            ros_repo_path = ws_paths[0].replace(str(pathlib.Path(p).home()), '~')
            ros_repo_path = str(pathlib.Path(ros_repo_path) / 'src' / 'stretch_ros')
            ros_repo_path = os.path.expanduser(ros_repo_path)
        else:
            print("Unable to find stretch_description package.")
            sys.exit(1)
        if not os.path.exists(ros_repo_path):
            print(f"Unable to find path {ros_repo_path}.")
            sys.exit(1)

def get_ros_version():
    global root_dir, data_dir
    if 'ROS_DISTRO' in os.environ.keys():
        if os.environ['ROS_DISTRO'] == 'noetic':
            # works on ubuntu 20.04
            import importlib_resources
            root_dir = str(importlib_resources.files("stretch_urdf"))
            data_dir = f"{root_dir}/{model_name}"
            return 1
        if os.environ['ROS_DISTRO'] == 'humble':
            # works on ubuntu 22.04
            import importlib.resources as importlib_resources
            root_dir = importlib_resources.files("stretch_urdf")
            data_dir = f"{root_dir}/{model_name}"
            return 2
        if os.environ['ROS_DISTRO'] == 'jazzy':
            # works on ubuntu 24.04
            import importlib.resources as importlib_resources
            root_dir = importlib_resources.files("stretch_urdf")
            data_dir = f"{root_dir}/{model_name}"
            return 2


def print_info():
    global model_name, tool_name, data_dir, ros_repo_path
    print(f"Robot Model Name = {model_name}")
    print(f"Robot Tool Name = {tool_name}")
    print(f"Found Stretch URDF files at = {data_dir}")
    print(f"Found ROS_DISTRO = {os.environ['ROS_DISTRO']}")
    print(f"Stretch Description Package Path = {ros_repo_path}/stretch_description")
    print("\n")

def search_and_replace(file_path, search_word, replace_word):
   with open(file_path, 'r') as file:
      file_contents = file.read()

      updated_contents = file_contents.replace(search_word, replace_word)

   with open(file_path, 'w') as file:
      file.write(updated_contents)

def verify_robot_tool_name():
    global data_dir, ros_repo_path, model_name, tool_name
    data_dir = f"{root_dir}/{model_name}"
    if model_name in os.listdir(root_dir):
        files = os.listdir(data_dir)
        fn = f"stretch_description_{model_name}_{tool_name}.urdf"
        if fn in files:
            return True
        else:
            click.secho(f'Unable to find tool {tool_name} for model {model_name}')
            return False
    else:
        click.secho(f'Unable to find model {model_name}')
        return False


def copy_mesh_files(v=False):
    global data_dir, ros_repo_path, model_name, tool_name
    print("\nCopying Mesh Files.......\n")
    for f in os.listdir(f"{data_dir}/meshes/"):
        src = f"{data_dir}/meshes/{f}"
        dst = f"{ros_repo_path}/stretch_description/meshes/"
        cmd = f"cp -r {src} {dst}"
        run_cmd(cmd,v)
    
def copy_xacro_files(v=False):
    global data_dir, ros_repo_path, model_name, tool_name
    print("\nCopying Xacro Files.......\n")
    # Process and copy the xacro files to ros
    for f in os.listdir(f"{data_dir}/xacro/"):
        src = f"{data_dir}/xacro/{f}"
        dst = f"{ros_repo_path}/stretch_description/urdf/"
        cmd = f"cp -r {src} {dst}"
        run_cmd(cmd,v)

        # Replace meshses path to point to ros package source
        try:
            search_and_replace(f"{dst}/{f}",'./meshes','package://stretch_description/meshes')
        except IsADirectoryError:
            for f2 in os.listdir(f"{dst}/{f}"):
                search_and_replace(f"{dst}/{f}/{f2}",'./meshes','package://stretch_description/meshes')
        
        # Manually remove the link_ground and joint_ground
        if f == 'stretch_main.xacro':
            main = f"{dst}/{f}"
            file = open(main)
            r = remove_lines_between_patterns(file.read(),'<link\n    name="link_ground">','</link>')
            r = remove_lines_between_patterns(r,'<joint\n    name="joint_ground"','</joint>')
            file.close()
            run_cmd(f"rm {main}")
            f = open(main, "a")
            f.write(r)
            f.close()

    stretch_description_xacro = f"stretch_description_{model_name}_{tool_name}.xacro"
    print(f"\nUpdating stretch_description.xacro from {stretch_description_xacro}\n")
    if stretch_description_xacro in os.listdir(f"{ros_repo_path}/stretch_description/urdf/"):
        cmd = f"cp {ros_repo_path}/stretch_description/urdf/{stretch_description_xacro} {ros_repo_path}/stretch_description/urdf/stretch_description.xacro"
        run_cmd(cmd,v)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Tool to update the ROS Stretch Description package Xacros and Mesh files based on the configured End-of-Arm tool.')
    parser.add_argument("-v","--verbose", help="Prints more info", action="store_true")
    parser.add_argument('--model', type=str,choices=['RE1V0','RE2V0','SE3'], help='Choose a Robot model name.')
    parser.add_argument('--tool', type=str, help='Choose a supported Robot tool name.')
    parser.add_argument("--ros2_rebuild", help="Rebuild ROS2's Stretch Description package", action="store_true")
    parser.add_argument('-y','--yes',help="Override Confirmation prompt.", action="store_true")
    args = parser.parse_args()

    get_robot_params()
    verify_ros()
    
    if len([x for x in (args.model,args.tool) if x is not None]) == 1:
        parser.error('--model and --tool must be given together')

    if args.ros2_rebuild:
        if ros_version==2:
            print("Updating Uncalibrated URDF...\n")
            os.system('ros2 run stretch_calibration update_uncalibrated_urdf')
            print("\nUpdating URDF after xacro change...\n")
            os.system('ros2 run stretch_calibration update_urdf_after_xacro_change')
            print("\nRebuild stretch_description package...\n")
            os.system('cd ~/ament_ws;colcon build --packages-select stretch_core stretch_description')
        else:
            print("Unable to find a ROS2 install.")
    elif args.model and args.tool:
        model_name = args.model
        tool_name = args.tool
        if verify_robot_tool_name():
            print_info()
            if args.yes:
                x = 'y'
            else:
                x = input("Proceed with URDF Update (y/n)?")
            if x=='y' or x=='Y':
                copy_mesh_files(args.verbose)
                copy_xacro_files(args.verbose)
                if ros_version==2:
                    print("Note: Rebuild Stretch Description package to apply the changes with command `stretch_urdf_ros_update.py --ros2_rebuild`.")
    else:
        if verify_robot_tool_name():
            print_info()
            if args.yes:
                x = 'y'
            else:
                x = input("Proceed with URDF Update (y/n)?")
            if x=='y' or x=='Y':
                copy_mesh_files(args.verbose)
                copy_xacro_files(args.verbose)
                if ros_version==2:
                    print("Note: Rebuild Stretch Description package to apply the changes with command `stretch_urdf_ros_update.py --ros2_rebuild`.")

        
