from eager_core.engine_params import EngineParams


class GazeboEngine(EngineParams):
    """
    Gazebo engine parameters for EAGER environments.

    This class includes all settings available for the Gazebo physics engine.

    :param world: A path to a Gazebo world (.world) file.
    :param dt: The time step when :func:`eager_core.eager_env.EagerEnv.step` is called
    :param max_update_rate: The maximum amount of steps within a second
    :param gui: Will run Pybullet without gui if set to False
    :param seed: The seed for the physics simulation
    """

    def __init__(self,
                 world: str = '$(find eager_bridge_gazebo)/worlds/eager_empty.world',
                 dt: float = 0.08,
                 max_update_rate: float = 0.0,  # 0.0 --> simulate as fast as possible
                 gui: bool = True,
                 seed = None):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'gazebo'
        launch_file = '$(find eager_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        if seed is None:
            kwargs.pop('seed')
        super(GazeboEngine, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.
        self.time_step = self.dt

        # Error check the parameters here.
