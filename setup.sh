#!/bin/bash

echo "###########################"
echo "Getting prerequisites in..."
echo "###########################"

for i in {10..1}; do
  echo "$i"
  sleep 1
done

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep -y

sudo rosdep init
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install ros-humble-turtlebot3* -y
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-pip -y
pip3 install pandas scikit-learn joblib
pip3 install "numpy<1.25" --force-reinstall
pip3 install --force-reinstall scipy scikit-learn

echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

echo ""
echo "############################"
echo "Manual setup beginning in..."
echo "############################"

for i in {10..1}; do
  echo "$i"
  sleep 1
done

echo ""
echo "Please open a new terminal window and run the following command:"
echo "ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py"
echo "THIS MAY TAKE A WHILE TO OPEN"
read -p "Type 'yes' once you see the gazebo app opened: " ans; [ "$ans" = "yes" ] || exit 1

echo ""
echo "Please open a new terminal window and run the following command:"
echo "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"
read -p "Type 'yes' once it's running: " ans; [ "$ans" = "yes" ] || exit 1

echo ""
echo "Please open a new terminal window and run the following command:"
echo "ros2 run turtlebot3_teleop teleop_keyboard"
read -p "Type 'yes' once it's running: " ans; [ "$ans" = "yes" ] || exit 1
echo "Move the robot quite some distance for about a min with teleop in this new window (w, a, s, d, s. x keys)"
read -p "Type 'yes' once you are done: " ans; [ "$ans" = "yes" ] || exit 1

ros2 run nav2_map_server map_saver_cli -f ~/map

echo ""
echo "Please open a new terminal window and run the following command:"
echo "ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/map.yaml use_sim_time:=True"
read -p "Type 'yes' once it's done: " ans; [ "$ans" = "yes" ] || exit 1

cd ~/ros2_ws/src
ros2 pkg create smart_nav --build-type ament_python --dependencies rclpy std_msgs geometry_msgs nav2_simple_commander

cat > ~/ros2_ws/src/smart_nav/setup.py <<EOF
from setuptools import find_packages, setup

package_name = 'smart_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aakash Dwivedi',
    maintainer_email='dwivediaakash37@gmail.com',
    description='Smart robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_node = smart_nav.input_node:main',
            'decision_node = smart_nav.decision_node:main',
            'navigator_node = smart_nav.navigator_node:main',
        ],
    },
)
EOF

cd ~/ros2_ws/src
cd smart_nav/smart_nav
cp ~/python/__init__.py ~/ros2_ws/src/smart_nav/smart_nav/__init__.py
cp ~/python/input_node.py ~/ros2_ws/src/smart_nav/smart_nav/input_node.py
cp ~/python/decision_node.py ~/ros2_ws/src/smart_nav/smart_nav/decision_node.py
cp ~/python/navigator_node.py ~/ros2_ws/src/smart_nav/smart_nav/navigator_node.py

cd ~/ros2_ws
colcon build --packages-select smart_nav
source install/setup.bash

mkdir -p ~/turtlebot3_smart_nav/ml_model

cp ~/dataset/project_dataset.csv ~/turtlebot3_smart_nav/ml_model/project_dataset.csv

cd ~/turtlebot3_smart_nav/ml_model
cp ~/python/train_model.py ~/turtlebot3_smart_nav/ml_model/train_model.py
python3 train_model.py

cp ~/turtlebot3_smart_nav/ml_model/room_predictor.joblib ~/ros2_ws/src/smart_nav/smart_nav/

cd ~/ros2_ws
colcon build
source install/setup.bash

echo ""
echo "#############################"
echo "Approaching final steps in..."
echo "#############################"

for i in {5..1}; do
  echo "$i"
  sleep 1
done

echo ""
echo "Please open a new terminal window and run the following command:"
echo "ros2 run smart_nav input_node"
read -p "Type 'yes' once it's done: " ans; [ "$ans" = "yes" ] || exit 1

echo ""
echo "Please open a new terminal window and run the following command:"
echo "ros2 run smart_nav decision_node"
read -p "Type 'yes' once it's done: " ans; [ "$ans" = "yes" ] || exit 1

echo ""
echo "Please open a new terminal window and run the following command:"
echo "ros2 run smart_nav navigator_node"
read -p "Type 'yes' once it's done: " ans; [ "$ans" = "yes" ] || exit 1

echo ""
echo "##################################"
echo "Smart Nav initialized successfully"
echo "##################################"
