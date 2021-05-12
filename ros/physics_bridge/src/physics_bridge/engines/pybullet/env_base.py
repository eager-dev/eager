import gym, gym.spaces, gym.utils, gym.utils.seeding, gym.error
import numpy as np
import os
from pybullet_gym.camera import Camera
from pybullet_utils import bullet_client

from pkg_resources import parse_version

try:
    import pybullet
except ImportError as e:
    raise gym.error.DependencyNotInstalled("{}. (HINT: you need to install pybullet)".format(e))

try:
    if os.environ["PYBULLET_EGL"]:
        import pkgutil
except:
    pass


class URDFBaseBulletEnv(gym.Env):
    """
      Base class for Bullet physics simulation loading URDF pybullet_gym in a Scene.
      These pybullet_gym create single-player scenes and behave like normal Gym pybullet_gym, if
      you don't use multiplayer.
      """

    metadata = {'render.modes': ['human', 'rgb_array', 'rgb_segmap_depth'], 'video.frames_per_second': 60}

    def __init__(self, robot, render=False, target=None, world=None, intrinsic_yaml=None, extrinsic_yaml=None, camera_id=-1, timestep=0.0165, frame_skip=1):
        self.scene = None
        self.physicsClientId = -1
        self.ownsPhysicsClient = 0
        self.timestep = timestep
        self.frame_skip = frame_skip
        self.camera = Camera(intrinsic_yaml=intrinsic_yaml, extrinsic_yaml=extrinsic_yaml, camera_id=camera_id)
        self.isRender = render
        self.robot = robot
        self.target = target
        self.world = world
        self.seed()
        self.update_spaces()
        self.reset()

    @property
    def dt(self):
        return self.timestep * self.frame_skip

    def update_spaces(self):
        self.action_space = self.robot.action_space
        self.observation_space = self.robot.observation_space
        self.state_space = self.robot.state_space
        if self.target:
            self.target_space = self.target.observation_space

    def configure(self, args):
        self.robot.args = args
        if self.target:
            self.target.args = args
        if self.world:
            self.world.args = args

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        self.robot.np_random = self.np_random  # use the same np_randomizer for robot as for env
        if self.target:
            self.target.np_random = self.np_random
        if self.world:
            self.world.np_random = self.np_random
        return [seed]

    def reset(self):
        if (self.physicsClientId < 0):
            self.ownsPhysicsClient = True

            if self.isRender:
                self._p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
            else:
                self._p = bullet_client.BulletClient()
            self._p.resetSimulation()
            self._p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
            # optionally enable EGL for faster headless rendering
            try:
                if os.environ["PYBULLET_EGL"]:
                    con_mode = self._p.getConnectionInfo()['connectionMethod']
                    if con_mode == self._p.DIRECT:
                        egl = pkgutil.get_loader('eglRenderer')
                        if (egl):
                            self._p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
                        else:
                            self._p.loadPlugin("eglRendererPlugin")
            except:
                pass
            self.physicsClientId = self._p._client
            self._p.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
            # todo: BEGIN DEBUG (setting camera visualizer to proper position)
            camInfo = self._p.getDebugVisualizerCamera()
            lookat = camInfo[11]
            distance = camInfo[10] * 0.2
            pitch = camInfo[9]
            yaw = camInfo[8]
            self._p.resetDebugVisualizerCamera(distance, yaw, pitch, lookat)
            # todo: END DEBUG

        if self.scene is None:
            self.scene = self.create_single_player_scene(self._p)
        if not self.scene.multiplayer and self.ownsPhysicsClient:
            self.scene.episode_restart(self._p)

        self.robot.scene = self.scene
        if self.target:
            self.target.scene = self.scene
        if self.world:
            self.world.scene = self.scene

        self.frame = 0
        self.done = 0
        self.reward = 0
        dump = 0

        # RESET ALL OBJECTS (robot, target, world)
        s, obs = self.robot.reset(self._p)
        info = {'state': s}
        self.object_ids = {'robot': self.robot.robot_objectid}
        if self.target:
            s_target, _ = self.target.reset(self._p)
            info['target'] = s_target
            self.object_ids['target'] = self.target.robot_objectid
        else:
            s_target = None
        if self.world:
            s_world = self.world.reset(self._p)
            self.object_ids['world'] = self.world.robot_objectid
        else:
            s_world = None
        if self.robot.action_init is not None:
            info['action_init'] = self.robot.action_init
        self.update_spaces()
        self.camera_adjust(s, s_target=s_target, s_world=s_world)
        self.potential = self.robot.calc_potential()
        return obs, info

    def camera_adjust(self, s, s_target=None, s_world=None):
        pass

    def render(self, mode='human', close=False):
        if mode == "human":
            self.isRender = True

        if mode not in ["rgb_array", "rgb_segmap_depth"]:
            return np.array([])

        if (self.physicsClientId >= 0):
            (_, _, px, depth, segmap) = self.camera.get_camera_image(self._p)
        else:
            px = np.full((self.camera.in_render_height, self.camera.in_render_width, 4), 255, dtype=np.uint8)
            depth = np.zeros((self.camera.in_render_height, self.camera.in_render_width), dtype=np.float32)
            segmap = np.full((self.camera.in_render_height, self.camera.in_render_width), -1, dtype=np.uint8)
        if mode == "rgb_array":
            return px[:, :, :3]
        if mode == "rgb_segmap_depth":
            return px[:, :, :3], depth, segmap

    def close(self):
        if (self.ownsPhysicsClient):
            if (self.physicsClientId >= 0):
                self._p.disconnect()
        self.physicsClientId = -1

    def HUD(self, state, a, done):
        pass

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed


# class Camera:
#
#     def __init__(self, env):
#         self.env = env
#         pass
#
#     def move_and_look_at(self, i, j, k, x, y, z):
#         lookat = [x, y, z]
#         camInfo = self.env._p.getDebugVisualizerCamera()
#
#         distance = camInfo[10]
#         pitch = camInfo[9]
#         yaw = camInfo[8]
#         self.env._p.resetDebugVisualizerCamera(distance, yaw, pitch, lookat)


# class MJCFBaseBulletEnv(gym.Env):
#     """
#       Base class for Bullet physics simulation loading MJCF (MuJoCo .xml) pybullet_gym in a Scene.
#       These pybullet_gym create single-player scenes and behave like normal Gym pybullet_gym, if
#       you don't use multiplayer.
#       """
#
#     metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 60}
#
#     def __init__(self, robot, render=False):
#         self.scene = None
#         self.physicsClientId = -1
#         self.ownsPhysicsClient = 0
#         self.camera = Camera(self)
#         self.isRender = render
#         self.robot = robot
#         self.seed()
#         self._cam_dist = 3
#         self._cam_yaw = 0
#         self._cam_pitch = -30
#         self._render_width = 320
#         self._render_height = 240
#
#         self.action_space = robot.action_space
#         self.observation_space = robot.observation_space
#         # self.reset()
#
#     def configure(self, args):
#         self.robot.args = args
#
#     def seed(self, seed=None):
#         self.np_random, seed = gym.utils.seeding.np_random(seed)
#         self.robot.np_random = self.np_random  # use the same np_randomizer for robot as for env
#         return [seed]
#
#     def reset(self):
#         if (self.physicsClientId < 0):
#             self.ownsPhysicsClient = True
#
#             if self.isRender:
#                 self._p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
#             else:
#                 self._p = bullet_client.BulletClient()
#             self._p.resetSimulation()
#             self._p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
#             # optionally enable EGL for faster headless rendering
#             try:
#                 if os.environ["PYBULLET_EGL"]:
#                     con_mode = self._p.getConnectionInfo()['connectionMethod']
#                     if con_mode == self._p.DIRECT:
#                         egl = pkgutil.get_loader('eglRenderer')
#                         if (egl):
#                             self._p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
#                         else:
#                             self._p.loadPlugin("eglRendererPlugin")
#             except:
#                 pass
#             self.physicsClientId = self._p._client
#             self._p.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
#
#         if self.scene is None:
#             self.scene = self.create_single_player_scene(self._p)
#         if not self.scene.multiplayer and self.ownsPhysicsClient:
#             self.scene.episode_restart(self._p)
#
#         self.robot.scene = self.scene
#
#         self.frame = 0
#         self.done = 0
#         self.reward = 0
#         dump = 0
#         s = self.robot.reset(self._p)
#         self.potential = self.robot.calc_potential()
#         return s
#
#     def camera_adjust(self):
#         pass
#
#     def render(self, mode='human', close=False):
#
#         if mode == "human":
#             self.isRender = True
#         if self.physicsClientId >= 0:
#             self.camera_adjust()
#
#         if mode != "rgb_array":
#             return np.array([])
#
#         base_pos = [0, 0, 0]
#         if (hasattr(self, 'robot')):
#             if (hasattr(self.robot, 'body_real_xyz')):
#                 base_pos = self.robot.body_real_xyz
#         if (self.physicsClientId >= 0):
#             view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
#                                                                     distance=self._cam_dist,
#                                                                     yaw=self._cam_yaw,
#                                                                     pitch=self._cam_pitch,
#                                                                     roll=0,
#                                                                     upAxisIndex=2)
#             proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
#                                                              aspect=float(self._render_width) /
#                                                                     self._render_height,
#                                                              nearVal=0.1,
#                                                              farVal=100.0)
#             (_, _, px, _, _) = self._p.getCameraImage(width=self._render_width,
#                                                       height=self._render_height,
#                                                       viewMatrix=view_matrix,
#                                                       projectionMatrix=proj_matrix,
#                                                       renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
#
#             self._p.configureDebugVisualizer(self._p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)
#         else:
#             px = np.array([[[255, 255, 255, 255]] * self._render_width] * self._render_height, dtype=np.uint8)
#         rgb_array = np.array(px, dtype=np.uint8)
#         rgb_array = np.reshape(np.array(px), (self._render_height, self._render_width, -1))
#         rgb_array = rgb_array[:, :, :3]
#         return rgb_array
#
#     def close(self):
#         if (self.ownsPhysicsClient):
#             if (self.physicsClientId >= 0):
#                 self._p.disconnect()
#         self.physicsClientId = -1
#
#     def HUD(self, state, a, done):
#         pass
#
#     # def step(self, *args, **kwargs):
#     # 	if self.isRender:
#     # 		base_pos=[0,0,0]
#     # 		if (hasattr(self,'robot')):
#     # 			if (hasattr(self.robot,'body_xyz')):
#     # 				base_pos = self.robot.body_xyz
#     # 				# Keep the previous orientation of the camera set by the user.
#     # 				#[yaw, pitch, dist] = self._p.getDebugVisualizerCamera()[8:11]
#     # 				self._p.resetDebugVisualizerCamera(3,0,0, base_pos)
#     #
#     #
#     # 	return self.step(*args, **kwargs)
#     if parse_version(gym.__version__) < parse_version('0.9.6'):
#         _render = render
#         _reset = reset
#         _seed = seed



