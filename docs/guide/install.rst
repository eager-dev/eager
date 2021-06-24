.. _install:

Installation
============

Prerequisites
-------------

EAGER requires ROS Melodic or Noetic and python 3.6+ to be installed.

.. note::

    Running in ROS Melodic is supported but may cause issues in some cases.
    Make sure you have installed all modules needed to run ROS on python 3.

    In Melodic, the gym environment side of EAGER will need to be run in python 3
    but the physics bridge side can be run in both python 2 or python 3.
    This should allow users to run robotics hardware that does not support Noetic and python 3.

Stable Release
--------------
At this time no stable version has been released yet.

Bleeding-edge version
---------------------
To install EAGER directly from the repository follow these steps:

If you do not have a ROS workspace yet create one:

.. code-block:: bash

    mkdir -p catkin_ws/src
    cd catkin_ws/src

Clone the eager repository into the ``src`` folder:

.. code-block:: bash

    git clone https://github.com/eager-dev/eager.git

.. note::
    At this time two requirements to run the UR5e cannot be retreived using rosdep.
    Please clone these into the ``src`` folder using the following commands:

    .. code-block:: bash

        git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
        git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git

Move to your workspace folder and run rosdep to install any requirements:

.. code-block:: bash

    cd ..
    rosdep install --from-paths src --ignore-src -r -y

Then build and source the packages:

.. code-block:: bash

    catkin_make
    source devel/setup.bash
