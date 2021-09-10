from eager_core.engine_params import EngineParams


class SafeActionsProcessor(EngineParams):
    def __init__(self,
                 robot_type: str,
                 vel_limit: float,
                 collision_height: float,
                 checks_per_rad: int=15,
                 duration: float=0.4,
                 ):
        launch_file = '$(find eager_process_safe_actions)/launch/safe_actions.launch'

        kwargs = locals().copy()
        kwargs.pop('self')
        self.launch_args = kwargs
        super(SafeActionsProcessor, self).__init__(**kwargs)
