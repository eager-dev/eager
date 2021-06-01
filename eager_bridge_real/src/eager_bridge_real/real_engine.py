from eager_core.engine_params import EngineParams

class RealEngine(EngineParams):
    def __init__(self,
                 action_rate: int = 30):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'real'
        launch_file = '$(find eager_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RealEngine, self).__init__(**kwargs)

        # Error check the parameters here.
