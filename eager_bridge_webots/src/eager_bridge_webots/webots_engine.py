from eager_bridge_webots.webots_parser import WebotsParser
from eager_core.engine_params import EngineParams
from eager_core.utils.file_utils import substitute_xml_args


class WebotsEngine(EngineParams):
    def __init__(self,
                 world: str,
                 dt: float = 0.08,
                 no_gui: str = 'false',
                 mode: str = 'fast'):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'webots'
        launch_file = '$(find eager_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(WebotsEngine, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.
        self.step_time = int(self.dt * 1000)

        wp = WebotsParser()
        wp.load(substitute_xml_args('%s' % world))
        for f in wp.content['root']:
            for field in f['fields']:
                if field['name'] == 'basicTimeStep':
                    self.basicTimeStep = int(field['value'])
                    break
            else:
                continue
            break

        if self.step_time % self.basicTimeStep != 0:
            raise RuntimeError('The steptime (%d ms) is not a multiple of the basicTimeStep (%d ms).' % (self.step_time, self.basicTimeStep))
