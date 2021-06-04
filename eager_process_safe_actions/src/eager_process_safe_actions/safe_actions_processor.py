from eager_core.engine_params import EngineParams

class SafeActionsProcessor(EngineParams):
    def __init__(self,
                 moveit_package: str,
                 joint_names: list,
                 robot_type: str,
                 group_name: str,
                 object_frame: str,
                 checks_per_rad: int,
                 duration: float,
                 vel_limit: float):
        kwargs = locals().copy()
        kwargs.pop('self')
        self.launch_args = kwargs
        super(SafeActionsProcessor, self).__init__(**kwargs)