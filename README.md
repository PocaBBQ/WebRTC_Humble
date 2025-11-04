# WebRTC_Humble
Learning WebRTC with ROS2 Humble

<img width="345" height="405" alt="image" src="https://github.com/user-attachments/assets/68209ea4-fe94-4bda-a019-c342340b00f7" />


# 0) One-time: ffmpeg & v4l tools
sudo apt-get install -y ffmpeg v4l-utils

# 1) Clone this repo
Open Terminal
git clone https://github.com/PocaBBQ/WebRTC_Humble.git

# 2) Install Python deps into your ROS env (recommended in an isolated venv)
cd ~/WebRTC_Humble/src
python3 -m pip install -r requirements.txt

# 3) Build
cd ~/WebRTC_Humble
colcon build --symlink-install
source install/setup.bash

# 4) Launch (default /dev/video0, 1280x720@30 on port 8000)
ros2 launch ros_streamer ros_streamer.launch.py

# 5) View from a browser on the same LAN:
# http://<ubuntu-host>:8000/
