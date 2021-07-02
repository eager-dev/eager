from eager_core.engine_params import EngineParams
import pybullet_data


class PyBulletEngine(EngineParams):
    """
    Pybullet engine parameters for EAGER environments.

    This class includes all settings available for the Pybullet physics engine.

    :param world: A path to a Pybullet world urdf
    :param dt: The time step when :func:`eager_core.eager_env.EagerEnv.step` is called
    :param no_gui: For Webots this will launch minimized and without rendering
    :param num_substeps: Subdivide the physics simulation step further within a single step of ``dt``
    :param num_solver_iterations: Choose the maximum number of constraint solver iterations
    :param contact_erp: Contact error reduction parameter
    """

    def __init__(self,
                 world: str = '%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'),
                 dt: float = 0.08,
                 no_gui: bool = False,
                 num_substeps: int = 1,
                 num_solver_iterations: int = 5,
                 contact_erp: float = 0.9):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        bridge_type = 'pybullet'
        launch_file = '$(find eager_bridge_%s)/launch/%s.launch' % (bridge_type, bridge_type)

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
