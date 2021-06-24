#!/bin/sh
export EAGER_HOME=$PWD
export PYTHONPATH=$EAGER_HOME/src:$PYTHONPATH
export PYTHON=python3

if [[ ! -d "eager_venv" ]]; then
	pip3 install virtualenv
	virtualenv -p python3 eager_venv
fi

source eager_venv/bin/activate

if [[ -z "$ROS_DISTRO" ]]; then
       echo "ROS is not installed"
       exit 1
fi

export EAGER_WS=${HOME}/eager_ws

if [ ! -d ${HOME}/eager_ws ]; then
		echo "Creating eager_ws"
		cd ${HOME}
		mkdir -p eager_ws/src
fi

cd ${EAGER_HOME}
pip install -r requirements.txt

source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update --rosdistro $ROS_DISTRO

# install universal robot packages UR5 robot package
if [ ! -d ${EAGER_WS}/src/universal_robot ]; then
    cd ${EAGER_WS}/src
    git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
fi

# install universal robot packages UR5 robot package
if [ ! -d ${EAGER_WS}/src/ur_modern_driver ]; then
    cd ${EAGER_WS}/src
    git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
fi

# install eager
ln -s ${EAGER_HOME} ${EAGER_WS}/src

cd ${EAGER_WS}
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
