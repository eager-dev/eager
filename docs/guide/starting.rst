.. _starting:

Getting Started
===============

Initializing an Engine
----------------------

Switching and initializing physics engines is very simple in EAGER.
Currently the following physics engines are supported:

WeBots
~~~~~~

.. autoclass:: eager_bridge_webots.webots_engine.WebotsEngine
    :noindex:

PyBullet
~~~~~~~~

.. autoclass:: eager_bridge_pybullet.pybullet_engine.PyBulletEngine
    :noindex:

Gazebo
~~~~~~

.. autoclass:: eager_bridge_gazebo.gazebo_engine.GazeboEngine
    :noindex:

Reality
~~~~~~~

.. autoclass:: eager_bridge_real.real_engine.RealEngine
    :noindex:

In the codeblock below we import the engine class and create a default world configuration.

.. code-block:: python

    from eager_bridge_webots.webots_engine import WebotsEngine
    engine = WebotsEngine()

The classes here do not actually do anything when initialized but only store parameters to be used when starting the engine.

Initializing Objects
--------------------

To add objects to the world function create them using the ``create`` funcion in the ``Object`` class.

.. autofunction:: eager_core.objects.Object.create
    :noindex:

In the codeblock below we create a single robot of type ``ur5e`` from the ``eager_robot_ur5e`` package with the name ``robot1``.

.. code-block:: python

    from eager_core.objects import Object
    ur5e_robot = Object.create('robot1', 'eager_robot_ur5e', 'ur5e')

As long as objects do not have the same name as many as needed can be added to an environment.
These objects do not do anything yet when initialized and will only start when passed on to an ``EagerEnv``.

Creating an EagerEnv
--------------------

Now an envirnment can be made with the created objects and the simulator.
As soon as the ``EagerEnv`` is initialized the chosen physics engine will start and the objects will be added to the world.

.. autoclass:: eager_core.eager_env.EagerEnv
    :noindex:

In the codeblock below we create a default ``EagerEnv`` with the single robot we created.

.. code-block:: python

    from eager_core.eager_env import EagerEnv
    env = EagerEnv(engine, objects=[ur5e_robot])

From this point on the ``EagerEnv`` works exactly as any other gym Env. The full observation and action space of all the objects of the world will be combined.
To alter the behaviour of the environment and its observations and actions to be exposed to the reinforcement learning algoritms the ``BaseEagerEnv`` can be subclassed.

Full Example
------------

In this example we create a WeBots environment with a single UR5e robot.
Once started we step the environment with random actions from the action space and then close the environment.

.. code-block:: python

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