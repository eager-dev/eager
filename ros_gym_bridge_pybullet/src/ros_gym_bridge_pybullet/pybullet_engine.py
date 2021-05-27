from ros_gym_core.engine_params import EngineParams
from ros_gym_core.utils.file_utils import substitute_xml_args


class PyBulletEngine(EngineParams):
    def __init__(self,
                 world: str,
                 dt: float = 0.08,
                 no_gui: str = 'false',
                 num_substeps: int = 1,
                 num_solver_iterations: int = 5,
                 contact_erp: float = 0.9):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'pybullet'
        launch_file = '$(find ros_gym_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(PyBulletEngine, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.
        self.dt_sim = self.dt / self.num_substeps

        # Error check the parameters here.
        if self.num_substeps < 1:
            raise RuntimeError('"num_substeps" must be an integer value larger than 1 (%d !> 1).' % self.num_substeps)
