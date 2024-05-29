#!/bin/bash

# Colors
Black='\033[0;30m'        # Black
Red='\033[0;31m'          # Red
Green='\033[0;32m'        # Green
Yellow='\033[0;33m'       # Yellow
Blue='\033[0;34m'         # Blue
Purple='\033[0;35m'       # Purple
Cyan='\033[0;36m'         # Cyan
White='\033[0;37m'        # White
Color_Off='\033[0m'       # Text Reset

# Check if the directory 'bag' exists, if not, create it
echo -e "${Purple} Checking if the 'bag' directory exists... ${Color_Off}"
if [ ! -d "ros_tutorial/bag" ]; then
  mkdir ros_tutorial/bag
    echo "'bag' directory created!" 
fi

# Check if gdown is installed, if not, install it
if ! command -v gdown &> /dev/null; then
  pip install gdown
fi

# rosbag file name
bag_file=aerotrain_perception.bag

# download the bag file and save it in the 'bag' directory
echo -e "${Purple}Downloading the bag file... ${Color_Off}"
gdown  1SJzwnQf8-Vaf5-trvTghDQIJYRnBvb5T
echo -e "${Purple} Download complete! ${Color_Off}"
echo -e "${Purple} Moving the bag file to the 'bag' directory... ${Color_Off}"
mv "$bag_file" ros_tutorial/bag
echo -e "${Green} Move complete! ${Color_Off}"


# Pull an image from Docker Hub
echo "Pulling the Docker image..."
docker pull ricfr/aero_train_summer_school_day_1
echo "Pull complete!"