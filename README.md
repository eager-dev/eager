# EAGER
Engine Agnostic Gym Environment for Robotics.

## Installation
The installation procedure for EAGER is simple. First clone this repository to a desired location. Then run:
```
source bin/setup.bash
```
This will create a virtual environment in which Python dependencies will be installed.
Also, a catkin workspace named *eager_ws* will be created in the home directory of the user.
Symbolic links will be created that point to the EAGER ROS packages and these packages will be build.
After running this command, the user is ready to go. The installation can be checked by running one of the examples:
```
roslaunch ur5e_example ur5e_example.launch
```

## Toolkit's advantages (current implementation)
- Assurances on action execution by using services (if simulator has same assurance, i.e. does not use topics for communication)
- Ability to use a python debugger (because we use services)
- Ability to train faster than real time in simulation environments.
- Ability to step environments in simulation. In this way the simulation time between actions is constant and performance is not affected by calculation time.
- Ability to use different physics engines: WeBots, PyBullet, Gazebo (which supports Open Dynamics Engine (ODE), Bullet, Dynamic Animation and Robotics Toolkit (DART), Simbody)

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
