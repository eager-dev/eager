from eager_bridge_webots.webots_parser import WebotsParser
from eager_core.engine_params import EngineParams
from eager_core.utils.file_utils import substitute_xml_args


class WebotsEngine(EngineParams):
    def __init__(self,
                 world: str = '$(find eager_bridge_webots)/worlds/default.wbt',
                 dt: float = 0.08,
                 no_gui: bool = False,
                 mode: str = 'realtime',
                 physics_step = 0.02,
                 seed = None,
                 virtual_display=False,
                 ci=False):
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
