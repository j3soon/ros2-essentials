#!/bin/bash -e

# Get the download target from the command line.
# If no target is provided, download all models.
# Available targets: 
# - all
# - aws-robomaker-hospital-world
# - aws-robomaker-small-house-world
# - aws-robomaker-small-warehouse-world
# - citysim
# - turtlebot3_gazebo
if [ "$#" -ne 1 ]; then
    TARGET="all"
else
    TARGET=$1
fi
echo "Downloading models for target: $TARGET"

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

# Check if the gdown command is available. If not, install it.
# Reference: https://github.com/wkentaro/gdown
export PATH=$PATH:~/.local/bin
if ! [ -x "$(command -v gdown)" ]; then
    echo "gdown is not installed. Installing it..."
    pip install gdown
fi

# Download aws-robomaker-hospital-world models.
if [ "$TARGET" == "all" ] || [ "$TARGET" == "aws-robomaker-hospital-world" ]; then
    AWS_HOSPITAL_DIR="$SCRIPT_DIR/../../aws-robomaker-hospital-world"

    # Check whether the folder exists.
    if ! [ -d "$AWS_HOSPITAL_DIR" ]; then
        echo "The aws-robomaker-hospital-world folder is not found. Please clone the repository first."
        exit 1
    fi

    echo "Downloading aws-robomaker-hospital-world models..."

    MODEL_URL="https://drive.google.com/drive/folders/1YTKmuYMSIlo_IsWhbl0sFGc_5kkklUMg?usp=drive_link"
    FUEL_MODEL_URL="https://drive.google.com/drive/folders/1TzOUUlmMAFhWJgh-sLOM2yHZp4BnvzUy?usp=drive_link"
    PHOTO_URL="https://drive.google.com/drive/folders/1rKy4kJYt8Xepxmb714AefHWglaCrNVKp?usp=drive_link"

    gdown --folder --output "$AWS_HOSPITAL_DIR/models" "$MODEL_URL"
    gdown --folder --output "$AWS_HOSPITAL_DIR/fuel_models" "$FUEL_MODEL_URL"
    gdown --folder --output "$AWS_HOSPITAL_DIR/photos" "$PHOTO_URL"
fi

# Download aws-robomaker-small-house-world models.
if [ "$TARGET" == "all" ] || [ "$TARGET" == "aws-robomaker-small-house-world" ]; then
    AWS_SMALL_HOUSE_DIR="$SCRIPT_DIR/../../aws-robomaker-small-house-world"

    # Check whether the folder exists.
    if ! [ -d "$AWS_SMALL_HOUSE_DIR" ]; then
        echo "The aws-robomaker-small-house-world folder is not found. Please clone the repository first."
        exit 1
    fi

    echo "Downloading aws-robomaker-small-house-world models..."

    # Since the gdown can't download folder that contain more than 50 files or folders,
    # we need to download the models separately.
    MODEL_1_URL="https://drive.google.com/drive/folders/1ygsbLpdznAb7YtTuRz_705vFy2BBw29l?usp=drive_link"
    MODEL_2_URL="https://drive.google.com/drive/folders/1mUNeJKv60sjBgGHJ5khnb_fsFOLHpHzx?usp=drive_link"
    PHOTO_URL="https://drive.google.com/drive/folders/1o3m7AAgWXe3oBNHkYBZv7d6cdcTX3iwi?usp=drive_link"

    gdown --folder --output "$AWS_SMALL_HOUSE_DIR/models" "$MODEL_1_URL"
    gdown --folder --output "$AWS_SMALL_HOUSE_DIR/models" "$MODEL_2_URL"
    gdown --folder --output "$AWS_SMALL_HOUSE_DIR/photos" "$PHOTO_URL"
fi

# Download aws-robomaker-small-warehouse-world models.
if [ "$TARGET" == "all" ] || [ "$TARGET" == "aws-robomaker-small-warehouse-world" ]; then
    AWS_WAREHOUSE_DIR="$SCRIPT_DIR/../../aws-robomaker-small-warehouse-world"

    # Check whether the folder exists.
    if ! [ -d "$AWS_WAREHOUSE_DIR" ]; then
        echo "The aws-robomaker-small-warehouse-world folder is not found. Please clone the repository first."
        exit 1
    fi

    echo "Downloading aws-robomaker-small-warehouse-world models..."

    MODEL_URL="https://drive.google.com/drive/folders/1iVa5twAVRW_-mL4Ak8An6lMh1PXJvSu4?usp=drive_link"

    gdown --folder --output "$AWS_WAREHOUSE_DIR/models" "$MODEL_URL"
fi

# Download citysim models.
if [ "$TARGET" == "all" ] || [ "$TARGET" == "citysim" ]; then
    CITYSIM_DIR="$SCRIPT_DIR/../../citysim"

    # Check whether the folder exists.
    if ! [ -d "$CITYSIM_DIR" ]; then
        echo "The citysim folder is not found. Please clone the repository first."
        exit 1
    fi

    echo "Downloading citysim models..."

    # Since the gdown can't download folder that contain more than 50 files or folders,
    # we need to download the models separately.
    MODEL_1_URL="https://drive.google.com/drive/folders/1dHCd7v9jwl4K8s9coqIWGVHZY3VdKDNs?usp=drive_link"
    MODEL_2_URL="https://drive.google.com/drive/folders/1rrQhc0FbPTDWKeNu5YlMg1bSDlQaNFH7?usp=drive_link"
    MEDIA_URL="https://drive.google.com/drive/folders/1sJGrmBk4MXTjjsfeCfgB3uN2B3jonljw?usp=drive_link"

    gdown --folder --output "$CITYSIM_DIR/models" "$MODEL_1_URL"
    gdown --folder --output "$CITYSIM_DIR/models" "$MODEL_2_URL"
    gdown --folder --output "$CITYSIM_DIR/media" "$MEDIA_URL"
fi

# Download turtlebot3_gazebo models.
if [ "$TARGET" == "all" ] || [ "$TARGET" == "turtlebot3_gazebo" ]; then
    TURTLEBOT3_GAZEBO_DIR="$SCRIPT_DIR/../../turtlebot3_gazebo"

    # Check whether the folder exists.
    if ! [ -d "$TURTLEBOT3_GAZEBO_DIR" ]; then
        echo "The turtlebot3_gazebo folder is not found. Please clone the repository first."
        exit 1
    fi

    echo "Downloading turtlebot3_gazebo models..."

    MODEL_URL="https://drive.google.com/drive/folders/1_YZlsKYe_ZORMR-GCDn5xY8cGI8OSCC9?usp=drive_link"

    gdown --folder --output "$TURTLEBOT3_GAZEBO_DIR/models" "$MODEL_URL"
fi

echo "Done."