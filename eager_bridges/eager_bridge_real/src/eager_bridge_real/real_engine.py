from eager_core.engine_params import EngineParams


class RealEngine(EngineParams):
    """
    Real engine parameters for EAGER environments.

    This class includes all settings available for reality.

    :param dt: The time step when :func:`eager_core.eager_env.EagerEnv.step` is called
    """

    def __init__(self,
                 dt: float = 0.4):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'real'
        launch_file = '$(find eager_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)
        action_rate = int(1 / dt)

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RealEngine, self).__init__(**kwargs)

        # Error check the parameters here.
