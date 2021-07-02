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
Currently, only a bleeding-edge version is available. We provide two ways to
install the bleeding-edge version, i.e. cloning the repository into a catkin
workspace and installation with pip.

Direct installation from the repository
---------------------------------------
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

Installation with pip
---------------------------------------

Installation with pip also provides the possibility to perform a custom
installation rather than full installation with all EAGER packages. This can
done via HTTPS or SSH.

- Using HTTPS, run:

.. code-block:: bash

    pip install git+https://github.com/eager-dev/eager

- Using SSH, run:

.. code-block:: bash

    pip install git+ssh://git@github.com/eager-dev/eager.git

Now install EAGER by running:

.. code-block:: bash

    install_eager

The bash script ``install_eager`` will clone the repository and create a catkin
workspace at desired locations. It also asks for input in order to create links to the desired packages in this workspace. Afterwards, it will build the workspace. In order to use EAGER, the only thing that is required is sourcing:

.. code-block:: bash

    source [EAGER_WORKSPACE_LOCATION]/devel/setup.bash

It is possible to run ``install_eager`` multiple times in order to install
additional packages later.
