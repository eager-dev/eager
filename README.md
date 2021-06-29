# EAGER
Engine Agnostic Gym Environment for Robotics.

## Installation
There are two ways to install EAGER:

1. By cloning the repository into a catkin workspace and building from source:
```
mkdir -p ~/eager_ws/src
cd ~/eager_ws/src
git clone https://github.com/eager-dev/eager.git
```
Be aware that if you want to use the `eager_bridge_gazebo` in combination with the `eager_robot_ur5e`, the following repositories should be cloned into `~/eager_ws/src`:
```
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
```
Now you can install the dependencies and build the workspace:
```
cd ~/eager_ws
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
2. Via PIP installation, which also provides the possibility to perform a custom installation rather than full installation with all EAGER packages:
```
pip install git+https://github.com/eager-dev/eager
install_eager
```
The bash script ```install_eager``` will clone the repository and create a catkin
workspace. It also asks for input in order to create links to the desired packages in this workspace. Afterwards, it will build the workspace. In order to use EAGER, the only thing that is required
is sourcing:
```
source ~/eager_ws/devel/setup.bash
```
It is possible to run ```install_eager``` multiple times in order to install
additional packages.

## Toolkit's advantages (current implementation)
- Assurances on action execution by using services (if simulator has same assurance, i.e. does not use topics for communication)
- Ability to use a python debugger (because we use services)
- Ability to train faster than real time in simulation environments.
- Ability to step environments in simulation. In this way the simulation time between actions is constant and performance is not affected by calculation time.
- Ability to use different physics engines: WeBots, PyBullet, Gazebo (which supports Open Dynamics Engine (ODE), Bullet, Dynamic Animation and Robotics Toolkit (DART), Simbody)

## Package building
- Create a symbolic link in your `catkin_ws/src` to the `ros` directory.

## Launch python training script with a different python interpreter (e.g. with an Anaconda virtual environment)
- Point the shebang `#!` line in the beginning of a python script to the correct python interpreter (e.g. `#!/home/bas/anaconda3/envs/py37tf23/bin/python3`).

## Run python training script dynamically (e.g. via PyCharm debugger)
- If you would like to start-up your python script without a launch file, make sure to source the `devel/setup.bash` (e.g. inside your `~/.bashrc`) right before you run your python script inside the IDE.
- For PyCharm, add `source ~/eager_ws/devel/setup.bash` to `~/.bashrc` and follow [this](http://wiki.ros.org/IDEs#PyCharm_.28community_edition.29).
- Make sure to have the python package `defusedxml` installed. `pip install defusedxml`.
- Before running the python script in your IDE, start up a roscore in a separate terminal.

## WeBots
Troubleshooting:
- Make sure WEBOTS_HOME is defined
- Make sure every robot to be controlled has 'ros' as its controller with flags '--synchronize' and 'name=NAME' where NAME is the name of the robot in you ros_env
- Make sure at least one robot has the supervisor flag set to true

## TEMPORARY HACKS TO MAKE CODE WORK :(
- In `physics_bridge/src/engines/gazebo/gazebo.py` in `_register_object(...)` function, change hardcoded object type.


## Python dependencies (listing them here, don't how to list in CMakeList if ros-<module> does not exist...)
- defusedxml
- stable-baselines3
- six
- gazebo (this probably does exist, check!)
- opencv==4.3.0.36
- pybullet==3.1.7
