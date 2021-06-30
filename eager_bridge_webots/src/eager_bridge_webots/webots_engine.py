from eager_core.engine_params import EngineParams

class WebotsEngine(EngineParams):
    """
    Webots engine parameters for EAGER environments.

    This class includes all settings available for the Webots physics engine.

    :param world: A path to a Webots world (.wbt) file. This file will be copied to a temporary file and edited to include the chosen settings
    :param dt: The time step when :func:`eager_core.eager_env.EagerEnv.step` is called, must be a multiple of ``physics_step``
    :param no_gui: For Webots this will launch minimized and without rendering
    :param mode: The running mode of Webots ('pauze', 'realtime' or 'fast')
    :param physics_step: The time step used in the physics calculations, ``dt`` must be a multiple of this
    :param seed: The seed for the physics simulation
    :param virtual_display: Run webots in a virtual display (:99)
    :param continuous_integration: Run in CI mode
    """
    def __init__(self,
                 world: str = '$(find eager_bridge_webots)/worlds/default.wbt',
                 dt: float = 0.08,
                 no_gui: bool = False,
                 mode: str = 'realtime',
                 physics_step = 0.02,
                 seed: int = None,
                 virtual_display: bool = False,
                 continuous_integration: bool = False):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'webots'
        launch_file = '$(find eager_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)
        step_time = int(dt * 1000)
        basicTimeStep = int(physics_step * 1000)

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        if seed is None:
            kwargs.pop('seed')
        super(WebotsEngine, self).__init__(**kwargs)

        if self.step_time % self.basicTimeStep != 0:
            raise RuntimeError('The steptime (%d ms) is not a multiple of the basicTimeStep (%d ms).' % (self.step_time, self.basicTimeStep))
