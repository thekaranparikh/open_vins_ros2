# open_vins_ros2
Trying out open vins code with realtime camera data on an OBC. Refference https://github.com/rpng/open_vins

## System Specs
- OBC : Raspberry pi 5
- Camera : Waveshare Stereo IMX 219-83
- OS : Ubuntu24.04
- ROS2 Version : Jazzy Jalisco

## Dependencies
OpenVINS relies on three major libraries: OpenCV, Eigen3, and Ceres Solver.
```
sudo apt-get install -y \
    git \
    cmake \
    build-essential \
    libeigen3-dev \
    libboost-all-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    libceres-dev \
    libopencv-dev
```

## Install ROS 2 Jazzy
First lets set up the ROS 2 repositories and install the Desktop version.
```
# Set locale
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 GPG key
sudo apt update && sudo apt install curl gnupg -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools -y
```

## Create ROS2 Workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone OpenVINS
git clone https://github.com/rpng/open_vins.git
cd ~/ros2_ws
# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
## Build OpenVINS with ROS 2
We use colcon,
```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DENABLE_ROS=ON
source install/setup.bash
```
Install camera node
```
sudo apt install ros-jazzy-camera-ros
```

## Enable Cameras
The Raspberry Pi 5 has two camera ports (CAM0 and CAM1). For a stereo setup, we must tell the kernel to look for an IMX219 sensor on both.
```
#Open the configuration file
sudo nano /boot/firmware/config.txt

#Set camera_auto_detect=0

#Add these lines at the last to enable both IMX219 camera sensors
dtoverlay=imx219,cam0
dtoverlay=imx219,cam1

#Save (Ctrl+O, Enter) and Exit (Ctrl+X).

#Reboot
sudo reboot
```

## Setting Up the OpenVINS Workspace
Now that ROS 2 is installed, we need to build the OpenVINS package specifically for ROS 2. <br>
(See all the Issues first and do accordingly!)
```
sudo apt update
sudo apt install ros-jazzy-image-transport ros-jazzy-cv-bridge ros-jazzy-sensor-msgs -y

# Create the workspace directory
mkdir -p ~/ov_ws/src
cd ~/ov_ws/src

# Clone the repository
git clone https://github.com/rpng/open_vins.git

# Go back to workspace root
cd ~/ov_ws

# Install dependencies using rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the project (This may take 5-10 minutes on a Pi 5)
colcon build --symlink-install --cmake-args -DENABLE_ROS=ON
```
if there are any issues like fatal error: Killed signal terminated program cc1plus, it generally mean Raspberry Pi ran out of RAM <br>
we will increase swap space to 4GB
```
# 1. Turn off current swap
sudo swapoff -a

# 2. Resize the swap file to 4GB (this may take a minute)
sudo dd if=/dev/zero of=/swapfile bs=1M count=4096

# 3. Set correct permissions
sudo chmod 600 /swapfile

# 4. Setup the swap area
sudo mkswap /swapfile

# 5. Turn swap back on
sudo swapon /swapfile

# 6. Verify it worked (you should see ~4.0G under "Swap")
free -h
```
The "Jazzy" Header Issue
In ROS 2 Jazzy (Ubuntu 24.04), many packages have completed the transition from .h to .hpp file extensions for C++ headers. OpenVINS was likely written for older versions (like Humble) that still looked for .h <br>
Fix:
```
# 1. Navigate to your source directory
cd ~/ov_ws/src/open_vins

# 2. Update image_transport, cv_bridge, and tf2_geometry_msgs headers recursively
find . -type f \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -exec sed -i 's/image_transport\/image_transport.h/image_transport\/image_transport.hpp/g' {} +
find . -type f \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -exec sed -i 's/cv_bridge\/cv_bridge.h/cv_bridge\/cv_bridge.hpp/g' {} +
find . -type f \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -exec sed -i 's/tf2_geometry_msgs\/tf2_geometry_msgs.h/tf2_geometry_msgs\/tf2_geometry_msgs.hpp/g' {} +
```
Now rebuild 
```
cd ~/ov_ws

# Remove the failed build/install folders
rm -rf build/ install/ log/

# Run the build with strict memory limits
MAKEFLAGS="-j1" colcon build --executor sequential --cmake-args -DENABLE_ROS=ON
```

## Verify the Installation
Before doing anything else, let’s make sure the ROS 2 environment can see the new OpenVINS nodes. Open a terminal and run:
```
source ~/ov_ws/install/setup.bash
ros2 pkg executables ov_msckf
```
We should see run_subscribe_msckf in the output. This is the main "live" node we will use.

## Custom libcamera setup
Ubuntu 24.04 apt hasnt been updated with libcamera, so we will have to set it up from scratch. <br>
Reference https://github.com/raspberrypi/libcamera <br>
Dependencies:
```
sudo apt update
sudo apt install python3-pip cmake python3-yaml python3-ply python3-jinja2 \
libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev \
meson ninja-build -y
```
verify installation
```
meson --version
ninja --version
```
build and install libcamera
```
cd ~
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera

# Configure the build using Meson
# We enable the raspberrypi pipeline specifically
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dcam=enabled \
  -Dtest=false \
  -Ddocumentation=disabled

# Compile (This will take a few minutes)
ninja -C build

# Install the library to your system
sudo ninja -C build install
sudo ldconfig
```
verify if it recognizes both the cameras
```
./build/src/apps/cam/cam --list
```

## Build the ROS 2 Driver
Now that libcamera is officially installed our your system, lets go back to our workspace and build the camera node that will feed OpenVINS.
```
#Run this command to install the manager and the parser it usually needs:
sudo apt install ros-jazzy-camera-info-manager ros-jazzy-camera-calibration-parsers -y

cd ~/ov_ws/src
# Clone the high-performance camera driver if you haven't
git clone https://github.com/christianrauch/camera_ros.git
cd ~/ov_ws
colcon build --packages-select camera_ros --symlink-install
source install/setup.bash
```

## Launching both Cameras
Once it builds successfully, you can launch your two cameras. Since we built libcamera from source earlier, this node will now use that high-performance pipeline.
```
ros2 run camera_ros camera_node --ros-args -p "cameras:=[0, 1]" -p "width:=640" -p "height:=480"
```
verify
```
ros2 topic list
```
we should be able to should see:
- /camera0/image_raw
- /camera0/camera_info
- /camera1/image_raw
- /camera1/camera_info

## Automate ROS 2 Environment
To avoid typing source /opt/ros/jazzy/setup.bash every time you open a terminal, run these commands:
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Now, our terminal will always recognize ros2 commands immediately.

## Fix the Calibration Files (default for now)

For the Left Camera: nano ~/.ros/camera_info/imx219__base_axi_pcie_120000_rp1_i2c_88000_imx219_10_640x480.yaml
```
image_width: 640
image_height: 480
camera_name: left_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```
For the Right Camera: Perform the same steps for the other file: nano ~/.ros/camera_info/imx219__base_axi_pcie_120000_rp1_i2c_80000_imx219_10_640x480.yaml (Paste the same content, but change camera_name to right_camera).

## Create a "Stereo Launch" Script
Instead of opening two terminals, create a Python launch file in our workspace. This ensures both cameras start with synchronized parameters. <br>
Create the file: nano ~/ov_ws/stereo_launch.py <br>
You can find it in the github repo <br>
runt it with:
```
ros2 launch ~/ov_ws/stereo_launch.py
```

## The Multi-Terminal Workflow
Terminal 1 (The Driver)
```
ros2 launch ~/ov_ws/stereo_launch.py
```
Terminal 2 (The Visualizer)
```
ros2 run rqt_image_view rqt_image_view
```
Once the window opens, use the dropdown menu at the top left to switch between /left/image_raw and /right/image_raw.


## Create a "imu_publisher" Script
Create the file: nano ~/ov_ws/imu_publisher.py <br>
You can find it in the github repo
run it with:
```
python3 imu_publisher.py
```
check if its publishing:
```
ros2 topic list
ros2 topic echo /imu
```

## Calibration
official documentation - https://docs.openvins.com/gs-calibration.html <br><br>
There are three Calibrations:
- Camera Intrinsic Calibration 
- IMU Noise Calibration
- Dynamic IMU-Camera Calibration

Out of these three, we wont be doing IMU Noise calibration as it demands to take static videos of 20hour long using 'allan variance ros'. We will use the default values for our Waveshare Stereo IMX 219-83 Camera. <br> <br>
You can find it as imu.yaml in the repo. <br><br>
Add the aprilgrid.yaml file from this repo and change the values as per you april grid dimensions.
1) How to record a rosbag?

Terminal 1:
```
ros2 launch ~/ov_ws/stereo_launch.py
```
Terminal 2: <br>
To see the left camera
```
ros2 run rqt_image_view rqt_image_view
```
Terminal 3: <br>
To see the right camera
```
ros2 run rqt_image_view rqt_image_view
```
Terminal 4: 
```
ros2 bag record /left/image_raw /right/image_raw -o static
```
After the ros2 bag is recorded click ctrl+c to end it. <br>
Now to view the info:
```
ros2 bag info static
```
and to play the bag:
```
ros2 bag play static
ros2 run rqt_image_view rqt_image_view #in another terminal to view the video
```
2) Camera Intrinsic Calibration (Offline)
- Here either we can keep the April grid stationary or the camera stationary <br>
- Record a ros2 bag with /left/image_raw and /right/image_ros
- Convert the mcap into .bag format
  - To do this, we will have to make a virtual environement
    ```
    python3 -m venv ~/ros_tools_env
    ```
    activate it:
    ```
    source ~/ros_tools_env/bin/activate
    ```
    install the conversion tool:
    ```
    pip install --upgrade pip
    pip install rosbags
    ```
    convert your .map to .bag:
    ```
    rosbags-convert <ROS2_FOLDER_NAME> --dst <NEW_FILE.bag>
    ```
    its done! deactivate the environment to come out:
    ```
    deactivate
    ```
Why did we need a virtual environment? <br>
sometimes, we need different/older versions of the same software. to avaoid the dependencies problem and to keep the system clean, we use a virtual environemtn which acts as an isolation. we can at any point delete the environment as well.

- Once you have the bag file ready, now you are ready for the next step.

We will transfer the static.bag and aprilgrid.yaml files to our laptop or desktop for the calibration. <br>
We have to do this because our laptop uses x86_64 architecture where as the raspberry pi uses ARM64 architecture. <br>
Many of the pre-built Docker images for Kalibr (like the stereolabs/kalibr one we used) are only built for x86. Trying to run them on a Pi would result in an "Exec format error." <br>
while we can compile it from source, but it is very difficult and hours-long process.

- Once all the required files are on the laptop, we will start with building kalibr

reference: https://github.com/ethz-asl/kalibr <br>
i used ubuntu 20.04 from wsl for this <br>

- install docker desktop on your system <br>
  clone the repo and build it
  ```
  git clone https://github.com/ethz-asl/kalibr.git
  cd kalibr
  docker build -t kalibr -f Dockerfile_ros1_20_04 . #use sudo if it gives error
  ```
  create a dir named calibration_data where the .bag and .yaml files are kept <br>
  launch the docker container:
  ```
  sudo docker run -it -v ~/swarm/calibration_data:/data kalibr
  ```
  source and run the calibration command:
  ```
  source /catkin_ws/devel/setup.bash
  
  rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/static.bag --target /data/aprilgrid.yaml \
    --models pinhole-equi pinhole-equi \
    --topics /left/image_raw /right/image_raw \
    --bag-freq 10.0
  ```
if it says "Initialization of focal length failed. Provide manual initialization:" <br>
Then enter 450. <br><br>
You wont be able to see the live gui of the calibration if you are using wsl and docker. <br><br>
after the calibration is complete, you will have 3 files in your directory:<br><br>
static-results-cam.txt <br>
static-report-cam.pdf <br>
static-camchain.yaml <br><br>

If you open the pdf and see the reprojection errors, it should be a gaussian destribution having less than < 0.2-0.5 pixel reprojection errors. <br>
If you are planning on performing online calibration of the camera, then larger values might be acceptable (e.g. 1 pixel), but a more accurate offline calibration is always preferred.

                                                                                        

    
2) Dynamic IMU-Camera Calibration
- Here we have to keep the april grid stationary and move the camera such that all the axis of the imu are excited.
- As before, launch the stereo_launch.py and also imu__publisher.py in different terminals. 
- Open rqt_image_view to see the live camera feed. 
- Record a ros2 bag with /left/image_raw /right/image_raw and /imu

## 
