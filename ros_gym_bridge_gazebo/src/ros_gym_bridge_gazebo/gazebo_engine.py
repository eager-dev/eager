from ros_gym_core.engine_params import EngineParams

class GazeboEngine(EngineParams):
    def __init__(self,
                 world: str = '$(find ros_gym_bridge_gazebo)/worlds/ros_gym_empty.world',
                 dt: float = 0.08,
                 max_update_rate: float = 0.0,  # 0.0 --> simulate as fast as possible
                 no_gui: str = 'false'):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'gazebo'
        launch_file = '$(find ros_gym_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(GazeboEngine, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.
        self.time_step = self.dt

        # Error check the parameters here.
