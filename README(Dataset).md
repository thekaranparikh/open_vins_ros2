## OPEN-VINS Setup on a Laptop
reference: https://docs.openvins.com/getting-started.html
## System Specs
(It took approx 20-25GB Storage end to end) <br>
WSL 2 <br>
Ubuntu 20.04 LTS <br>
ROS 2 Galactic <br>

## Setup WSL2
List all the wsl distros available:
```
wsl --list --online
```
install the required distribution:
```
wsl --install -d Ubuntu-20.04 #We are using ubuntu20.04 for this open vins setup
```
Restart your computer. <br>
Then, search your distros and open it from windows search. <br>
create a Unix username and password.

<br>Some necessary wsl commands:
```
wsl --version
wsl --update
wsl --shutdown
```
## ROS2 Installation
```
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
export ROS2_DISTRO=galactic
sudo apt install ros-$ROS2_DISTRO-desktop
```
installation of etential tools:
```
sudo apt-get install ros-galactic-ros2bag \
                     ros-galactic-rosbag2-storage-mcap \
                     ros-galactic-rosbag2-transport \
                     ros-galactic-rosbag2-cpp
```
verify the installation
```
source /opt/ros/galactic/setup.bash
ros2 bag list storage
```
make the source command permanenet(so that you dont have to type it again everytime in a new terminal):
```
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
install the openvins dependencies:
```
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
```
## Cloning the OpenVINS Project
```
mkdir -p ~/ov_ws/src
cd ~/ov_ws/src
git clone https://github.com/rpng/open_vins.git
cd ..
colcon build --event-handlers console_cohesion+ --packages-select ov_core ov_init ov_msckf ov_eval 
```
if you dont have colcon installed, then install it with this
```
sudo apt-get update
sudo apt-get install python3-colcon-common-extensions
```
verify the installation:
```
ros2 pkg prefix ov_msckf
```

## Troubleshooting
1) c++: fatal error: Killed signal terminated program cc1plus <br>
   This error occurs if you are running out of RAM during the compilation process. <br>
   FIX:
   - Limit Parallel Jobs
   - Adding Swap
     
We will create a swap file ( you can try 2GB, 4GB or 8GB ):
```
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```
Verify the new swap file:
```
swapon --show
```
Now we will Force colcon to compile fewer files at the same time. This will take longer but will prevent the "Killed" error.
```
colcon build --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+ --packages-select ov_msckf ov_eval
```
verify the installation:
```
ros2 pkg prefix ov_msckf
```
PS: It took me 50mins to install! <br>
After it is successfully built, you can reclaim the swap files:
```
sudo swapoff /swapfile
sudo rm /swapfile
```

## Simulation
Before running any nodes, you must tell your current terminal where the new packages are:
```
source ~/workspace/catkin_ws_ov/install/setup.bash
```
or set it permanent:
```
echo "source ~/workspace/catkin_ws_ov/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Dataset link: https://drive.google.com/drive/folders/1XcX9nC7HBMlpsYWIGpnYxgTe0FEVwv0p?usp=sharing <br>
Now Downlaod the EuRoC MAV dataset and keep it in ~/workspace/ov_data. <br>
Unzip it:
```
sudo apt update && sudo apt install unzip -y
unzip V1_01_easy.zip
```
Now you are ready for the simulation! <br>
Terminal 1:
```
rviz2
```
Rviz settings:
- In Rviz, Click on Add <br>
- Select path and click ok. <br>
- Now in the left panel, you can see path. Click on the dropdown beside it. <br>
- Now you can see Topic whihc is currently blank. Click on the dropdown and select /ov_msckf/pathimu or manually enter this. <br>
- Similarly, click on add again and add Image and set its topic to /ov_msckf/trackhist
- Now in the left panel, Displays, Global options, set Fixed Frame to 'global' or simply select it from the dropdown
  
Terminal 2:
```
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```
Terminal 3:
```
ros2 bag play V1_01_easy
```
As soon as you enter in terminal 3, go to Rviz and click reset and you will be able to see the image and path on your screen. <br>
Thats it!! <br>
On Rviz you can add multiple items and subscribe to that topic to see in more detail. <br>
To see all the Topic which you can subscribe to, run:
```
 ros2 topic list
```
## Troubleshooting
1) No cursor in Rviz
   
   FIX: Open PowerShell on Windows as Administrator and run:
   ```
   wsl --update
   wsl --shutdown
   ```
   Restart your Ubuntu terminal and try the rviz2 command again. <br>
   If that does not work, try running this:
   ```
   export LIBGL_ALWAYS_SOFTWARE=1
   rviz2
   ```
   This Disables GPU Acceleration for RViz and you will be able to see the cursor.
2) No Image or path in rviz2

   FIX:
   - Check if Fixed Frame is set to 'global' from the drop down menu or manually type it.
   - Check if the topic you have entered in path and image are valid topics from its dropdown menu or by running 'ros topic list' in a terminal.

3) The path draws in rviz2 is wrong or it crashes
To get a repeatable, accurate path on our hardware, we need to make the data "slower" so our CPU can process every single packet without dropping them.
   FIX:
   - Run the bag at 5% or 10% speed
     ```
     ros2 bag play V1_01_easy --rate 0.05
     ```
