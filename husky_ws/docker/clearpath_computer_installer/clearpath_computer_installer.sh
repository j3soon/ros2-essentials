#!/bin/bash -e
# Software License Agreement (BSD)
#
# Author    Tony Baltovski <tbaltovski@clearpathrobotics.com>
# Copyright (c) 2022, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


prompt_option() {
  # ask the user to select from a numbered list of options & return their selection
  # $1 is the variable into which the result is returned
  # $2 should be the question to ask the user as a prompt
  # $3+ should be the available options

  local __resultvar=$1
  shift
  local __prompt=$1
  shift
  local __n_options=$#

  echo -e "\e[39m$__prompt\e[0m"
  for (( i=1; $i<=$__n_options; i++ ));
  do
    opt=${!i}
    echo -e "[$i] \e[32m$opt\e[0m"
  done

  read answer
  eval $__resultvar="'$answer'"
}

prompt_YESno() {
  # as the user a Y/n question
  # $1 is the variable into which the answer is saved as either "n" or "y"
  # $2 is the question to ask

  local __resultvar=$1
  local __prompt=$2

  echo -e "\e[39m$__prompt\e[0m"
  echo "Y/n: "

  if [[ $AUTO_YES == 1 ]];
  then
    echo "Automatically answering Yes"
    eval $__resultvar="y"
  else
    read answer
    if [[ $answer =~ ^[n,N].* ]];
    then
      eval $__resultvar="n"
    else
      eval $__resultvar="y"
    fi
  fi
}

prompt_yesNO() {
  # as the user a y/N question
  # $1 is the variable into which the answer is saved as either "n" or "y"
  # $2 is the question to ask

  local __resultvar=$1
  local __prompt=$2

  echo -e "\e[39m$__prompt\e[0m"
  echo "y/N: "

  if [[ $AUTO_YES == 1 ]];
  then
    echo "Automatically answering No"
    eval $__resultvar="n"
  else
    read answer
    if [[ $answer =~ ^[y,Y].* ]];
    then
      eval $__resultvar="y"
    else
      eval $__resultvar="n"
    fi
  fi
}

# Enable the feature: Automatically answering Yes or No to 'prompt_YESno' and 'prompt_yesNO' questions
AUTO_YES=1

# available robots; pre-load the user-choice with -1 to indicate undefined
ROBOT_HUSKY_A200=1
ROBOT_JACKAL_J100=2
ROBOT_WARTHOG_W200=3
ROBOT_CHOICE=1

# Set front end to non-interactive to avoid prompts while installing packages
export DEBIAN_FRONTEND=noninteractive

echo ""
echo -e "\e[32mStarting Clearpath Computer Installer\e[0m"
echo ""

# Check if the script is run as root
if [ "$EUID" -eq 0 ]; then
    echo "You are the root user, this needs to be ran as a user to be completed."
fi

# Temporarily disable the blocking messages about restarting services in systems with needrestart installed
if [ -d /etc/needrestart/conf.d ]; then
  sudo bash -c "echo '\$nrconf{restart} = '\''a'\'';' > /etc/needrestart/conf.d/10-auto-cp.conf"
fi

echo -e "\e[94mSetup Open Robotics package server to install ROS 2 Humble\e[0m"

# Check if ROS 2 sources are already installed
if [ -e /etc/apt/sources.list.d/ros2.list ]; then
  echo -e "\e[33mWarn: ROS 2 sources exist, skipping\e[0m"
else
  sudo apt -y -qq install software-properties-common
  sudo add-apt-repository universe -y
  sudo apt -y -qq update && sudo apt -y -qq upgrade && sudo apt -y -qq install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  # Check if sources were added
  if [ ! -e /etc/apt/sources.list.d/ros2.list ]; then
    echo -e "\e[31mError: Unable to add ROS 2 package server, exiting\e[0m"
    exit 0
  fi
fi

echo -e "\e[32mDone: Setup ROS 2 package server\e[0m"
echo ""

echo -e "\e[94mSetup Clearpath Robotics package server\e[0m"

# Check if Clearpath sources are already installed
if [ -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
  echo -e "\e[33mWarn: Clearpath Robotics sources exist, skipping\e[0m"
else
  wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
  sudo bash -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
  # Check if sources were added
  if [ ! -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
    echo -e "\e[31mError: Unable to add Clearpath Robotics package server, exiting\e[0m"
    exit 0
  fi
fi

echo -e "\e[32mDone: Setup Clearpath Robotics package server\e[0m"
echo ""

echo -e "\e[94mUpdating packages and installing ROS 2\e[0m"
sudo apt -y -qq update
sudo apt install ros-humble-ros-base python3-argcomplete ros-dev-tools python3-vcstool ros-humble-clearpath-robot python3-clearpath-computer-setup -y
echo -e "\e[32mDone: Updating packages and installing ROS 2\e[0m"
echo ""

echo -e "\e[94mConfiguring rosdep\e[0m"

# Check if rosdep sources are already installed
if [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo -e "\e[33mWarn: rosdep was initalized, skipping\e[0m"
else
  sudo rosdep -q init
  # Check if sources were added
  if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo -e "\e[31mError: rosdep failed to initalize, exiting\e[0m"
    exit 0
  fi
fi

# Check if Clearpath rosdep sources are already installed
if [ -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
  echo -e "\e[33mWarn: CPR rosdeps exist, skipping\e[0m"
else
  sudo wget -q https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O \
    /etc/ros/rosdep/sources.list.d/50-clearpath.list
  # Check if sources were added
  if [ ! -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
    echo -e "\e[31mError: CPR rosdeps, exiting\e[0m"
    exit 0
  fi
fi

echo -e "\e[32mDone: Configuring rosdep\e[0m"
echo ""

echo -e "\e[94mConfiguring network service, if needed\e[0m"
# Check if the service file exists
if [ -e "/etc/systemd/system/network-online.target.wants/systemd-networkd-wait-online.service" ]; then
    # Check if TimeoutStartSec is present in the service file
    if grep -q "TimeoutStartSec=2sec" "/etc/systemd/system/network-online.target.wants/systemd-networkd-wait-online.service"; then
        echo "TimeoutStartSec is already present in /etc/systemd/system/network-online.target.wants/systemd-networkd-wait-online.service"
    else
        # Add TimeoutStartSec after RemainAfterExit=yes
        sudo sed -i '/RemainAfterExit=yes/a \'"TimeoutStartSec=2sec"'' "/etc/systemd/system/network-online.target.wants/systemd-networkd-wait-online.service"
        echo "TimeoutStartSec added to /etc/systemd/system/network-online.target.wants/systemd-networkd-wait-online.service"
    fi
else
    echo "Service file /etc/systemd/system/network-online.target.wants/systemd-networkd-wait-online.service not found."
fi
echo -e "\e[32mDone: Configuring network service, if needed\e[0m"
echo ""


### USER ONLY SECTION
if [ ! "$EUID" -eq 0 ]; then

  echo -e "\e[94mUpdating rosdep\e[0m"
  rosdep -q update
  echo -e "\e[32mDone: Updating rosdep\e[0m"
  echo ""

  if [[ $ROBOT_CHOICE -eq -1 ]];
  then
    echo ""
    prompt_option ROBOT_CHOICE "Which robot are you installing?" "Clearpath Husky A200" "Clearpath Jackal J100" "Clearpath Warthog W200"
  fi
  case "$ROBOT_CHOICE" in
    1)
      platform="a200"
      ;;
    2)
      platform="j100"
      ;;
    3)
      platform="w200"
      ;;
    * )
      echo -e "\e[31mERROR: Invalid selection"
      exit 1
      ;;
  esac
  echo "Selected ${platform}."
  echo ""

  # Check if Clearpath folder exists
  if [ -d /etc/clearpath/ ]; then
    echo -e "\e[33mWarn: Clearpath folder exist, skipping\e[0m"
  else
    echo -e "\e[94mCreating setup folder\e[0m"
    sudo mkdir -p -m 777 /etc/clearpath/
    # Check if directory was created
    if [ !  -d /etc/clearpath/ ]; then
      echo -e "\e[31mError: Clearpath folder setup, exiting\e[0m"
      exit 0
    fi
  fi

  # Check if Clearpath Config YAML exists
  if [ -e /etc/clearpath/robot.yaml ]; then
    echo -e "\e[33mWarn: Cleaprath Robot YAML exists\e[0m"
    prompt_YESno update_config "\eWould you like to change Cleaprath Robot YAML?\e[0m"
    if [[ $update_config == "y" ]]; then
      sudo mv /etc/clearpath/robot.yaml /etc/clearpath/robot.yaml.bkup.$(date +"%Y%m%d%H%M%S")
      echo -e "\e[94mCreating default robot YAML for ${platform}\e[0m"
      sudo cp /opt/ros/humble/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
      # Check if sources were added
      if [ ! -e /etc/clearpath/robot.yaml ]; then
        echo -e "\e[31mError: Clearpath robot YAML, exiting\e[0m"
        exit 0
      fi
    else
      echo "No change to Cleaprath Robot YAML"
    fi
  else
    echo -e "\e[94mCreating default robot YAML for ${platform}\e[0m"
    sudo cp /opt/ros/humble/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
    sudo chown "$(id -u -n):$(id -g -n)" /etc/clearpath/robot.yaml
    # Check if sources were added
    if [ ! -e /etc/clearpath/robot.yaml ]; then
      echo -e "\e[31mError: Clearpath robot YAML, exiting\e[0m"
      exit 0
    fi
  fi

  echo -e "\e[32mDone: Configuring Clearpath Setup\e[0m"
  echo ""

  source /opt/ros/humble/setup.bash

  prompt_YESno install_service "\e Would you like to install Cleaprath services?\e[0m"
  if [[ $install_service == "y" ]]; then
    echo -e "\e[94mInstalling clearpath robot service\e[0m"
    ros2 run clearpath_robot install

    if [ $? -eq 0 ]; then
      echo -e "\e[32mDone: Installing clearpath robot service\e[0m"
      echo ""
    else
      echo -e "\e[31mError: Failed to install clearpath robot service\e[0m"
      exit 0
    fi
  else
    echo "Skipping installing Clearpath services"
  fi

  sudo systemctl enable clearpath-robot

  echo -e "\e[94mSetting up clearpath enviroment\e[0m"
  grep -qxF "source /etc/clearpath/setup.bash" ~/.bashrc || echo "source /etc/clearpath/setup.bash" >> ~/.bashrc
  echo -e "\e[32mDone: Setting up clearpath enviroment\e[0m"
  echo ""

  echo -e "\e[94mSetting up groups\e[0m"

  if [ $(getent group flirimaging) ];
  then
    echo "flirimaging group already exists";
  else
    echo "Adding flirimaging group";
    sudo addgroup flirimaging;
  fi
  if id -nGz "$(whoami)" | grep -qzxF "flirimaging";
  then
    echo "User:$(whoami) is already in flirimaging group";
  else
    echo "Adding user:$(whoami) to flirimaging group";
    sudo usermod -a -G flirimaging $(whoami);
  fi

  val=$(< /sys/module/usbcore/parameters/usbfs_memory_mb)
  if [ "$val" -lt "1000" ]; then
    if [ -e /etc/default/grub ]; then
      if [ $(grep -c "usbcore.usbfs_memory_mb=" /etc/default/grub) -eq 0 ]; then # Memory Limit has not already been set
        sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="[^"]*/& usbcore.usbfs_memory_mb=1000/' /etc/default/grub
        echo "Increased the usbfs memory limits in the default grub configuration. Updating grub"
        sudo update-grub
      else
        echo -e "\e[33mWarn: usbfs memory limit is already set in /etc/default/grub in the following line:\e[0m"
        echo "$(grep "usbcore.usbfs_memory_mb" /etc/default/grub)"
        echo -e "\e[33mNo changes made, verify that usbfs_memory_mb is set to a minimum of 1000 and then try rebooting the computer\e[0m"
      fi

    else
      echo -e "\e[33mWarn: /etc/default/grub configuration file not found, no changes made. usbfs_memory_mb must be set manually.\e[0m"
      echo -e "\e[33mSee https://github.com/ros-drivers/flir_camera_driver/blob/humble-devel/spinnaker_camera_driver/docs/linux_setup_flir.md for instructions\e[0m"
      exit 0
    fi
  else
    echo "usbfs_memory_mb is already set to $val, no changes necessary."
  fi

  echo -e "\e[32mDone: Setting up groups\e[0m"
  echo ""
  echo -e "\e[32mClearpath Computer Installer Complete\e[0m"
  echo -e "\e[94mTo continue installation visit: https://docs.clearpathrobotics.com/docs/ros/networking/computer_setup \e[0m"
  echo ""
else
  echo -e "\e[32mClearpath Computer Installer needs to be ran as a user, please re-run.\e[0m"
  echo ""
fi

# Check if the hostname is cpr-unassigned
echo -e "\e[94mChecking hostname\e[0m"
if [ "$(hostname)" = "clearpath-unassigned" ]; then
  echo "Hostname is currently set to 'clearpath-unassigned'."
  prompt_YESno change_hostname "\eWould you like to change hostname?\e[0m"
  if [[ $change_hostname == "y" ]]; then
    # Prompt the user for a new hostname
    read -p "Enter a new hostname (Format is cpr-ROBOT_MODEL-SERIAL_NO, ie cpr-a200-1234): " new_hostname
    # Change the hostname
    sudo hostnamectl set-hostname "$new_hostname"
    # Display the new hostname
    echo "Hostname changed to '$new_hostname'."
    # Notify the user to restart for changes to take effect
    echo "Please restart your system for the changes to take effect."
  else
    echo "No change to hostname"
  fi
else
    echo "Hostname is already set to '$(hostname)'. No changes needed."
fi
echo -e "\e[32mDone: Checking hostname\e[0m"
echo ""

# Reenable messages about restarting services in systems with needrestart installed
if [ -e /etc/needrestart/conf.d/10-auto-cp.conf ]; then
  sudo rm /etc/needrestart/conf.d/10-auto-cp.conf
fi
