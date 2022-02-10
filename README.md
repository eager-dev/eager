[![License](https://img.shields.io/github/license/eager-dev/eager?label=license)](https://github.com/eager-dev/eager/blob/master/LICENSE)
[![Documentation Status](https://readthedocs.org/projects/eager-control/badge/?version=latest)](https://eager-control.readthedocs.io/en/latest/?badge=latest)
[![Build Status](https://img.shields.io/github/workflow/status/eager-dev/eager/CI)](https://github.com/eager-dev/eager/actions/workflows/ros_tests.yaml)
[![PEP8 Style Test](https://img.shields.io/github/workflow/status/eager-dev/eager/Lint?label=PEP8)](https://github.com/eager-dev/eager/actions/workflows/lint.yaml)

# EAGER
**Discontinued. Please check-out [EAGERx](https://github.com/eager-dev/eagerx).**

Engine Agnostic Gym Environment for Robotics (EAGER) is a toolkit that
will allow users to apply (deep) reinforcement learning for both simulated
and real robots as well as combinations thereof. The toolkit serves as
bridge between the popular reinforcement learning toolkit [OpenAI
Gym](https://gym.openai.com/) and robots that can either be real or
simulated. The EAGER toolkit makes use of the widely used ROS
framework for communication. Nonetheless, thanks to
the flexible design of the toolkit, it is possible for users to create a
customized bridge for robots without ROS support.

## Key Functionalities and Features


| **Functionality/Feature**                                           | **EAGER**          |
| ------------------------------------------------------------------- | -------------------|
| User-friendly creation of Gym environments for robot control tasks. | :heavy_check_mark: |
| PyBullet integration                                                | :heavy_check_mark: |
| Webots integration                                                  | :heavy_check_mark: |
| Gazebo integration                                                  | :heavy_check_mark: |
| Preprocessing of actions                                            | :heavy_check_mark: |
| Switching between and/or combining physics engines                  | :heavy_check_mark: |
| Documentation                                                       | :heavy_check_mark: |

#### Planned Functionalities and Features
We are currently working on the following features and functionalities:
- Guaranteed synchronization of actions and observations in simulators
- Demos
- Increasing the number of supported robots and sensors
- Preprocessing of observations
- Adding reset procedures
- More efficient communcation protocol

## Documentation

Documentation is available online: [https://eager-control.readthedocs.io](https://eager-control.readthedocs.io)


## Installation

At this time ***no stable version has been released yet***, only a bleeding-edge version is available.

#### Bleeding-edge version

**Prerequisites**: EAGER requires ROS Melodic or Noetic and Python 3.6+ to be installed.

---
**NOTE**
Running in ROS Melodic is supported but may cause issues in some cases.
Make sure you have installed all modules needed to run ROS on Python 3.

In Melodic, the gym environment side of EAGER will need to be run in
Python 3 but the physics bridge side can be run in both Python 2 or
Python 3. This should allow users to run robotics hardware that does not
support Noetic and Python 3.

---

We provide two ways to install the bleeding-edge version, i.e. (1) cloning the repository into a catkin workspace and (2) installation with pip.


**(1) Cloning the repository into a catkin workspace**

If you do not have a ROS workspace yet create one:
```
  mkdir -p catkin_ws/src
  cd catkin_ws/src
```
Clone the eager repository into the ``src`` folder:
```
  git clone https://github.com/eager-dev/eager.git
```

---
**NOTE**
At this time two requirements to run the UR5e cannot be retreived using rosdep.
Please clone these into the ``src`` folder using the following commands:
```
  git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
  git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
```

---
Move to your workspace folder and run rosdep to install any
requirements:
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
```
Then build and source the packages:
```
catkin_make
source devel/setup.bash
```

**(2) Install using pip**

This also provides the possibility to perform a custom installation rather than full installation with all EAGER packages. This can done via HTTPS or SSH.
- Using HTTPS, run:
```
pip install git+https://github.com/eager-dev/eager
```
- Using SSH, run:
```
pip install git+ssh://git@github.com/eager-dev/eager.git
```

Now install EAGER by running:
```
install_eager
```
The bash script ```install_eager``` will clone the repository and create a catkin
workspace at desired locations. It also asks for input in order to create links to the desired packages in this workspace. Afterwards, it will build the workspace. In order to use EAGER, the only thing that is required is sourcing:
```
source [EAGER_WORKSPACE_LOCATION]/devel/setup.bash
```
It is possible to run ```install_eager``` multiple times in order to install
additional packages later.

## Example
In this example we create a Webots environment with a single UR5e robot. Once started we step the environment with random actions from the action space and then close the environment.

```python
#!/usr/bin/env python3

import rospy
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_bridge_webots.webots_engine import WebotsEngine


if __name__ == '__main__':

    rospy.init_node('ur5e_example')

    engine = WebotsEngine()

    ur5e_robot = Object.create('robot1', 'eager_robot_ur5e', 'ur5e')
    env = EagerEnv(engine, objects=[ur5e_robot])

    for _ in range(1000):
        env.step(env.action_space.sample())

    env.close()
```

## Implemented Objects

The following objects are currently implemented in EAGER. We are currently working on extending the number of supported objects.

| **Object**                                                              | **Type** | **PyBullet**       | **Webots**         | **Gazebo**         | **Real-World**     |
| ----------------------------------------------------------------------- | -------- | ------------------ | ------------------ | ------------------ | ------------------ |
| [UR5e](https://www.universal-robots.com/nl/producten/ur5-robot/)        | Robot    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| [Panda](https://www.franka.de/robot-system/)                            | Robot    | :heavy_check_mark: | :x:                | :x:                | :x:                |
| [MultiSense S21](https://carnegierobotics.com/products/multisense-s21/) | Sensor   | :heavy_check_mark: | :heavy_check_mark: | :x:                | :x:                |
| Can                                                                     | Solid    | :x:                | :heavy_check_mark: | :x:                | :x:                |

## Citing the Project

To cite this repository in publications:
```bibtex
@misc{eager-control,
  author = {Van der Heijden, Bas and Keijzer, Alexander and Luijkx, Jelle},
  title = {EAGER: Engine Agnostic Gym Environment for Robotics},
  year = {2021},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/eager-dev/eager}},
}
```

## Maintainers

EAGER is currently maintained by [Bas van der Heijden](https://github.com/bheijden) (@bheijden), [Alexander Keijzer](https://github.com/AlexanderKeijzer) (@AlexanderKeijzer) and [Jelle Luijkx](https://github.com/jelledouwe) (@jelledouwe).

---


## Acknowledgments


EAGER is funded by the [OpenDR](https://opendr.eu/) Horizon 2020 project.

## Troubleshooting

#### Webots
- Make sure WEBOTS_HOME is defined
- Make sure every robot to be controlled has 'ros' as its controller with flags '--synchronize' and 'name=NAME' where NAME is the name of the robot in you EAGER environment
- Make sure at least one robot has the supervisor flag set to true
