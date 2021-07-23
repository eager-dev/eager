from eager_core.engine_params import EngineParams


class SafeActionsProcessor(EngineParams):
    def __init__(self,
                 moveit_package: str,
                 urdf_path: str,
                 joint_names: list,
                 robot_type: str,
                 group_name: str,
                 object_frame: str,
                 checks_per_rad: int,
                 duration: float,
                 vel_limit: float,
                 collision_height: float,
                 base_length: float,
                 workspace_length: float):
        kwargs = locals().copy()
        kwargs.pop('self')
        self.launch_args = kwargs
        super(SafeActionsProcessor, self).__init__(**kwargs)
