# ROS packages required
# from genpy import message
import rospy, xacro
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
from eager_bridge_pybullet.pybullet_world import World
from eager_bridge_pybullet.pybullet_robot import URDFBasedRobot
from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def, get_length_from_def, get_dtype_from_def

import tempfile
import re
import numpy as np
import functools
import os
import sys
os.environ['PYBULLET_EGL'] = "1"
# ^^^^ before importing pybullet_gym
try:
    if os.environ["PYBULLET_EGL"]:
        import pkgutil
except:
    pass

try:
    import pybullet
    import pybullet_data
    from pybullet_utils import bullet_client
except ImportError as e:
    from gym import error
    raise error.DependencyNotInstalled("{}. (HINT: you need to install PyBullet)".format(e))

# todo: change string checks with dict checks similar to control_modes
control_modes = {'position_control': pybullet.POSITION_CONTROL,
                 'velocity_control': pybullet.VELOCITY_CONTROL,
                 'torque_control': pybullet.TORQUE_CONTROL,
                 'pd_control': pybullet.PD_CONTROL}


class PyBulletBridge(PhysicsBridge):

    def __init__(self):
        # Initialize simulator
        self._p, self.physics_client_id = self._start_simulator()

        # Initialize world
        world_file = '%s' % substitute_xml_args(rospy.get_param('physics_bridge/world'))
        self._world = World(self._p, gravity=9.81,
                            world_file=world_file,
                            timestep=rospy.get_param('physics_bridge/dt_sim'),
                            frame_skip=rospy.get_param('physics_bridge/num_substeps'),
                            num_solver_iterations=rospy.get_param('physics_bridge/num_solver_iterations'),
                            contact_erp=rospy.get_param('physics_bridge/contact_erp'))

        self._robots = dict()

        self._sensor_cbs = dict()
        self._sensor_buffer = dict()
        self._set_sensor_services = []

        self._get_actuator_services = dict()

        self._state_cbs = dict()
        self._state_buffer = dict()
        self._set_state_services = []

        self._reset_services = dict()

        super(PyBulletBridge, self).__init__("pybullet")

    def _start_simulator(self):
        if rospy.get_param('physics_bridge/no_gui') == 'false':
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            p = bullet_client.BulletClient()
        physics_client_id = p._client
        p.resetSimulation()
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        # optionally enable EGL for faster headless rendering
        try:
            if os.environ["PYBULLET_EGL"]:
                con_mode = p.getConnectionInfo()['connectionMethod']
                if con_mode == p.DIRECT:
                    egl = pkgutil.get_loader('eglRenderer')
                    if (egl):
                        p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
                    else:
                        p.loadPlugin("eglRendererPlugin")
                    # STRANGE! Must pre-render atleast nun_runs=2 images, else crash in _camera_callback(...)
                    self._test_fps_rendering(p, physics_client_id, num_runs=100)
        except:
            pass

        return p, physics_client_id

    def _test_fps_rendering(self, p, physics_client_id, num_runs=100):
        import time
        times = np.zeros(num_runs)
        for i in range(num_runs):
            start = time.time()
            (_, _, rgb, depth, seg) = p.getCameraImage(width=640,
                                                       height=480,
                                                       viewMatrix=(0.0, -0.7071068286895752, 0.7071067690849304, 0.0, 1.0, 0.0, -0.0, 0.0, 0.0, 0.7071067690849304, 0.7071068286895752, 0.0, -0.0, 0.141421377658844, -0.841421365737915, 1.0),
                                                       projectionMatrix=(1.299038052558899, 0.0, 0.0, 0.0, 0.0, 1.7320507764816284, 0.0, 0.0, 0.0, 0.0, -1.0020020008087158, -1.0, 0.0, 0.0, -0.20020020008087158, 0.0),
                                                       flags=pybullet.ER_NO_SEGMENTATION_MASK,
                                                       renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                                                       physicsClientId=physics_client_id)
            # renderer=pybullet.ER_TINY_RENDERER)
            stop = time.time()
            duration = (stop - start)
            if (duration):
                fps = 1. / duration
            else:
                fps = 0
            times[i] = fps
        print("mean: {0} for {1} runs".format(np.mean(times), num_runs))

    def _register_object(self, topic, name, package, object_type, args, config):
        if 'xacro' in config and config['xacro']:
            # Check what kind of path was given and set urdf_filename accordingly
            if bool(re.match(r"\/", config['urdf_name'])):
                urdf_filename = config['urdf_name'] + ".urdf"
            elif bool(re.match(r"\$", config['urdf_name'])):
                urdf_filename = substitute_xml_args(config['urdf_name']) + ".urdf"
            else:
                eager_robot_pkg_path = substitute_xml_args('$(find eager_robot_' + object_type + ')')
                urdf_filename = eager_robot_pkg_path + '/' + config['urdf_name'] + '.urdf'
            generate_urdf = not('generate_urdf' in args and not(args['generate_urdf']))
            if generate_urdf:
                # Runs xacro to generate robot urdf
                rospy.loginfo("Running xacro to create urdf and storing it at %s", urdf_filename)
                try:
                    xacro_file = substitute_xml_args(config['xacro'])
                    robot_urdf = xacro.process_file(xacro_file).toprettyxml()
                except Exception as e:
                    rospy.logfatal('Failed to run xacro with error: \n%s', str(e))
                    sys.exit(1)
                re_expr = re.compile(r"package\:\/\/[a-z\_]*")
                m = re.findall(re_expr, robot_urdf)
                description_pkg_path = substitute_xml_args(re.search('\$\((.*)\)', config['xacro'])[0])
                str_urdf_full = re.sub(re_expr, description_pkg_path, robot_urdf)
                with open(urdf_filename, 'w') as file:
                    file.write(str_urdf_full)
            try:
                self._robots[name] = URDFBasedRobot(self._p,
                                                    model_urdf=urdf_filename,
                                                    robot_name=name,
                                                    basePosition=args['position'],
                                                    baseOrientation=args['orientation'],
                                                    fixed_base=args['fixed_base'],
                                                    self_collision=args['self_collision'])
            except Exception as e:
                rospy.logfatal('Failed to load urdf in pybullet: \n%s', str(e))
        elif 'urdf' in config and config['urdf']:  # check if urdf is available
            urdf_path = substitute_xml_args(config['urdf'])
            self._robots[name] = URDFBasedRobot(self._p,
                                                model_urdf=urdf_path,
                                                robot_name=name,
                                                basePosition=args['position'],
                                                baseOrientation=args['orientation'],
                                                fixed_base=args['fixed_base'],
                                                self_collision=args['self_collision'])
        # if 'type' is 'camera':
            # todo: create camera class, initialize with .yaml sensor description.
        else:  # if no urdf, also no pybullet robot. Create dummy robot.
            self._robots[name] = None
        if 'sensors' in config:
            self._init_sensors(topic, name, config['sensors'], self._robots[name])
        if 'actuators' in config:
            self._init_actuators(topic, name, config['actuators'], self._robots[name])
        if 'states' in config:
            self._init_states(topic, name, config['states'], self._robots[name])
            self._init_resets(topic, name, config['states'], self._robots[name])
        return True

    def _init_sensors(self, topic, name, sensors, robot):
        robot_sensors = dict()
        sensor_cb = dict()
        for sensor in sensors:
            topic_list = sensors[sensor]['names']
            space = sensors[sensor]['space']
            robot_sensors[sensor] = [get_value_from_def(space)] * get_length_from_def(space)
            if 'joint' in sensors[sensor]['type']:
                bodyUniqueId = []
                jointIndices = []
                for idx, pybullet_name in enumerate(topic_list):
                    bodyid, jointindex = robot.jdict[pybullet_name].get_bodyid_jointindex()
                    bodyUniqueId.append(bodyid), jointIndices.append(jointindex)
                    if 'force_torque' in sensors[sensor]['type']:
                        self._p.enableJointForceTorqueSensor(bodyUniqueId=bodyid, jointIndex=jointindex,
                                                             enableSensor=True, physicsClientId=self.physics_client_id)
                callback = functools.partial(self._joint_callback,
                                             buffer=self._sensor_buffer,
                                             name=name,
                                             obs_name=sensor,
                                             obs_type=sensors[sensor]['type'],
                                             bodyUniqueId=bodyUniqueId[0],
                                             jointIndices=jointIndices,
                                             physicsClientId=self.physics_client_id)
            elif 'link' in sensors[sensor]['type']:
                bodyUniqueId = []
                linkIndices = []
                for idx, pybullet_name in enumerate(topic_list):
                    bodyid, linkindex = robot.parts[pybullet_name].get_bodyid_linkindex()
                    bodyUniqueId.append(bodyid), linkIndices.append(linkindex)
                callback = functools.partial(self._link_callback,
                                             buffer=self._sensor_buffer,
                                             name=name,
                                             obs_name=sensor,
                                             obs_type=sensors[sensor]['type'],
                                             bodyUniqueId=bodyUniqueId[0],
                                             linkIndices=linkIndices,
                                             physicsClientId=self.physics_client_id)
            elif 'camera' in sensors[sensor]['type']:
                # todo: make position adjustable
                # todo: breaks if observation space is not defined as uint8 with 'shape' keyword...
                intrinsic = sensors[sensor]['intrinsic']
                extrinsic = sensors[sensor]['extrinsic']
                aspect = space['shape'][0] / space['shape'][1]  # height / width
                proj_matrix = pybullet.computeProjectionMatrixFOV(fov=intrinsic['fov'],
                                                                  aspect=aspect,
                                                                  nearVal=intrinsic['near_val'],
                                                                  farVal=intrinsic['far_val'])
                view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=extrinsic['pos'],
                                                                         distance=extrinsic['dist'],
                                                                         roll=extrinsic['euler'][0],
                                                                         pitch=extrinsic['euler'][1],
                                                                         yaw=extrinsic['euler'][2],
                                                                         upAxisIndex=extrinsic['up_axis'])

                callback = functools.partial(self._camera_callback,
                                             buffer=self._sensor_buffer,
                                             name=name,
                                             obs_name=sensor,
                                             obs_type=sensors[sensor]['type'],
                                             dtype=get_dtype_from_def(space),
                                             height=space['shape'][0],
                                             width=space['shape'][1],
                                             viewMatrix=view_matrix,
                                             projectionMatrix=proj_matrix,
                                             renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                                             physicsClientId=self.physics_client_id)
            else:
                raise ValueError('Sensor_type ("%s") must contain either {joint, link, camera}.' % sensors[sensor]['type'])

            sensor_cb[sensor] = callback
            self._set_sensor_services.append(rospy.Service(topic + "/sensors/" + sensor, get_message_from_def(space),
                                                           functools.partial(self._service,
                                                                             buffer=self._sensor_buffer,
                                                                             name=name,
                                                                             obs_name=sensor,
                                                                             message_type=get_response_from_def(space))))
        self._sensor_cbs[name] = sensor_cb
        self._sensor_buffer[name] = robot_sensors

    def _init_actuators(self, topic, name, actuators, robot):
        robot_actuators = dict()
        for actuator in actuators:
            topic_list = actuators[actuator]['names']
            space = actuators[actuator]['space']
            bodyUniqueId = []
            if 'joint' in actuators[actuator]['type']:
                jointIndices = []
                for idx, pybullet_name in enumerate(topic_list):
                    bodyid, jointindex = robot.jdict[pybullet_name].get_bodyid_jointindex()
                    bodyUniqueId.append(bodyid), jointIndices.append(jointindex)
                if actuators[actuator]['control_mode'] == 'position_control':
                    cb = functools.partial(self._p.setJointMotorControlArray,
                                           bodyUniqueId=bodyUniqueId[0],
                                           jointIndices=jointIndices,
                                           controlMode=control_modes[actuators[actuator]['control_mode']],
                                           # targetPositions=[],
                                           targetVelocities=actuators[actuator]['vel_target'],
                                           positionGains=actuators[actuator]['pos_gain'],
                                           velocityGains=actuators[actuator]['vel_gain'],
                                           physicsClientId=self.physics_client_id)
                elif actuators[actuator]['control_mode'] == 'velocity_control':
                    cb = functools.partial(self._p.setJointMotorControlArray,
                                           bodyUniqueId=bodyUniqueId[0],
                                           jointIndices=jointIndices,
                                           controlMode=control_modes[actuators[actuator]['control_mode']],
                                           # targetVelocities=actuators[actuator]['vel_target'],
                                           positionGains=actuators[actuator]['pos_gain'],
                                           velocityGains=actuators[actuator]['vel_gain'],
                                           physicsClientId=self.physics_client_id)
                elif actuators[actuator]['control_mode'] == 'pd_control':
                    cb = functools.partial(self._p.setJointMotorControlArray,
                                           bodyUniqueId=bodyUniqueId[0],
                                           jointIndices=jointIndices,
                                           controlMode=control_modes[actuators[actuator]['control_mode']],
                                           # targetVelocities=actuators[actuator]['vel_target'],
                                           positionGains=actuators[actuator]['pos_gain'],
                                           velocityGains=actuators[actuator]['vel_gain'],
                                           physicsClientId=self.physics_client_id)
                elif actuators[actuator]['control_mode'] == 'torque_control':
                    cb = functools.partial(self._p.setJointMotorControlArray,
                                           bodyUniqueId=bodyUniqueId[0],
                                           jointIndices=jointIndices,
                                           controlMode=control_modes[actuators[actuator]['control_mode']],
                                           # forces=[],
                                           positionGains=actuators[actuator]['pos_gain'],
                                           velocityGains=actuators[actuator]['vel_gain'],
                                           physicsClientId=self.physics_client_id)
                else:
                    raise ValueError('Control_mode ("%s") not recognized.' % actuators[actuator]['control_mode'])
                set_action_srvs = functools.partial(self._joint_actuator_srvs,
                                                    control_mode=actuators[actuator]['control_mode'],
                                                    callback=cb)
            else:
                raise ValueError('Actuator_type ("%s") must contain "joint".' % actuators[actuator]['type'])
            get_action_srv = rospy.ServiceProxy(topic + "/actuators/" + actuator, get_message_from_def(space))
            robot_actuators[actuator] = (get_action_srv, set_action_srvs)
        self._get_actuator_services[name] = robot_actuators

    def _init_states(self, topic, name, states, robot):
        robot_states = dict()
        state_cb = dict()
        for state in states:
            topic_list = states[state]['names']
            space = states[state]['space']
            robot_states[state] = [get_value_from_def(space)] * get_length_from_def(space)
            bodyUniqueId = []
            if 'joint' in states[state]['type']:
                jointIndices = []
                for idx, pybullet_name in enumerate(topic_list):
                    bodyid, jointindex = robot.jdict[pybullet_name].get_bodyid_jointindex()
                    bodyUniqueId.append(bodyid), jointIndices.append(jointindex)
                    if 'force_torque' in states[state]['type']:
                        self._p.enableJointForceTorqueSensor(bodyUniqueId=bodyid, jointIndex=jointindex,
                                                             enableSensor=True, physicsClientId=self.physics_client_id)
                callback = functools.partial(self._joint_callback,
                                             buffer=self._state_buffer,
                                             name=name,
                                             obs_name=state,
                                             obs_type=states[state]['type'],
                                             bodyUniqueId=bodyUniqueId[0],
                                             jointIndices=jointIndices,
                                             physicsClientId=self.physics_client_id)
            elif 'link' in states[state]['type']:
                linkIndices = []
                for idx, pybullet_name in enumerate(topic_list):
                    bodyid, linkindex = robot.parts[pybullet_name].get_bodyid_linkindex()
                    bodyUniqueId.append(bodyid), linkIndices.append(linkindex)
                callback = functools.partial(self._link_callback,
                                             buffer=self._state_buffer,
                                             name=name,
                                             obs_name=state,
                                             obs_type=states[state]['type'],
                                             bodyUniqueId=bodyUniqueId[0],
                                             linkIndices=linkIndices,
                                             physicsClientId=self.physics_client_id)
            elif 'base' in states[state]['type']:
                # todo: implement.
                pass
            else:
                raise ValueError('State type ("%s") must contain either {joint, link, base}.' % states[state]['type'])
            state_cb[state] = callback
            # self._set_state_services.append(rospy.Service(topic + "/states/" + state, BoxSpace, functools.partial(self._service,
            self._set_state_services.append(rospy.Service(topic + "/states/" + state, get_message_from_def(space), functools.partial(self._service,
                                                                                                         buffer=self._state_buffer,
                                                                                                         name=name,
                                                                                                         obs_name=state,
                                                                                                         message_type=get_response_from_def(
                                                                                                             space))))
        self._state_cbs[name] = state_cb
        self._state_buffer[name] = robot_states

    def _init_resets(self, topic, name, states, robot):
        robot_resets = dict()
        for state in states:
            topic_list = states[state]['names']
            space = states[state]['space']
            bodyUniqueId = []
            if 'joint' in states[state]['type']:
                jointIndices = []
                for idx, pybullet_name in enumerate(topic_list):
                    bodyid, jointindex = robot.jdict[pybullet_name].get_bodyid_jointindex()
                    bodyUniqueId.append(bodyid), jointIndices.append(jointindex)
                set_reset_srv = functools.partial(self._reset_srvs,
                                                  type=states[state]['type'],
                                                  bodyUniqueId=bodyUniqueId[0],
                                                  jointIndices=jointIndices,
                                                  physicsClientId=self.physics_client_id)
            elif 'link' in states[state]['type']:
                raise ValueError('State type ("%s") cannot be reset in pybullet.' % states[state]['type'])
            elif 'base' in states[state]['type']:
                bodyUniqueId = robot.robot_objectid[0]
                set_reset_srv = functools.partial(self._reset_srvs,
                                                  type=states[state]['type'],
                                                  bodyUniqueId=bodyUniqueId,
                                                  jointIndices=None,
                                                  physicsClientId=self.physics_client_id)
            else:
                raise ValueError('State type ("%s") must contain "{joint, base}".' % states[state]['type'])
            get_state_srv = rospy.ServiceProxy(topic + "/resets/" + state, get_message_from_def(space))
            robot_resets[state] = (get_state_srv, set_reset_srv)
        self._reset_services[name] = robot_resets

    def _joint_callback(self, buffer, name, obs_name, obs_type, bodyUniqueId, jointIndices, physicsClientId):
        states = self._p.getJointStates(bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=physicsClientId)
        obs = []
        if 'pos' in obs_type:  # (x)
            for i, (pos, vel, force_torque, applied_torque) in enumerate(states):
                obs.append(pos)
        elif 'vel' in obs_type:  # (v)
            for i, (pos, vel, force_torque, applied_torque) in enumerate(states):
                obs.append(vel)
        elif 'force_torque' in obs_type:  # (Fx, Fy, Fz, Mx, My, Mz)
            for i, (pos, vel, force_torque, applied_torque) in enumerate(states):
                obs += list(force_torque)
        elif 'applied_torque' in obs_type:  # (T)
            for i, (pos, vel, force_torque, applied_torque) in enumerate(states):
                obs.append(applied_torque)
        else:
            raise ValueError('Type of [%s][%s] in .yaml robot description (%s) must contain either {"pos", "vel", "force_torque", "applied_torque"}.' % (name, obs_name, obs_type))
        buffer[name][obs_name] = obs

    def _link_callback(self, buffer, name, obs_name, obs_type, bodyUniqueId, linkIndices, physicsClientId):
        obs = []
        if 'pos' in obs_type:
            states = self._p.getLinkStates(bodyUniqueId=bodyUniqueId, linkIndices=linkIndices,
                                           physicsClientId=physicsClientId, computeLinkVelocity=0)
            for i, (pos, quat, _, _, _, _) in enumerate(states):
                obs += pos
        elif 'orientation' in obs_type:
            states = self._p.getLinkStates(bodyUniqueId=bodyUniqueId, linkIndices=linkIndices,
                                           physicsClientId=physicsClientId, computeLinkVelocity=0)
            for i, (pos, quat, _, _, _, _) in enumerate(states):
                obs += quat
        elif 'angular' in obs_type:
            # IMPORTANT! check 'angular' before 'vel', because 'angular_vel' contains 'vel' as well...
            states = self._p.getLinkStates(bodyUniqueId=bodyUniqueId, linkIndices=linkIndices,
                                           physicsClientId=physicsClientId, computeLinkVelocity=1)
            for i, (pos, quat, _, _, _, _, vel, rate) in enumerate(states):
                obs += rate
        elif 'vel' in obs_type:
            states = self._p.getLinkStates(bodyUniqueId=bodyUniqueId, linkIndices=linkIndices,
                                           physicsClientId=physicsClientId, computeLinkVelocity=1)
            for i, (pos, quat, _, _, _, _, vel, rate) in enumerate(states):
                obs += vel
        else:
            raise ValueError(
                'Type of [%s][%s] in .yaml robot description (%s) must contain either {"pos", "vel", "orientation", "angular_vel"}.' % (name, obs_name, obs_type))
        buffer[name][obs_name] = obs

    def _camera_callback(self, buffer, name, obs_name, obs_type, dtype, width, height, viewMatrix, projectionMatrix, renderer, physicsClientId):
        obs = []
        (_, _, rgba, depth, seg) = self._p.getCameraImage(width=width,
                                                         height=height,
                                                         viewMatrix=viewMatrix,
                                                         projectionMatrix=projectionMatrix,
                                                         flags=pybullet.ER_NO_SEGMENTATION_MASK,
                                                         renderer=renderer,
                                                         physicsClientId=physicsClientId,)
        # todo: infer order via 'type' description
        if 'rgb' in obs_type[7:]:
            obs.append(rgba[:, :, :3])
        if 'a' in obs_type[7:]:
            obs.append(rgba[:, :, [3]])
        if 'd' in obs_type[7:]:
            if dtype == 'uint8':
                depth *= 255
            obs.append(depth[:, :, np.newaxis].astype(dtype))
        # todo: if float, probably calculate as value between [0, 1]
        # todo: Reduce computational load
        obs = np.concatenate(obs, axis=2).astype(dtype)  # computational load:  250 fps --> 190 fps
        obs = obs.flatten()  # computational load: 190 fps --> 185 fps
        obs = obs.tolist()  # computational load:  185 fps --> 85 fps
        buffer[name][obs_name] = obs

    def _joint_actuator_srvs(self, action, control_mode, callback):
        if control_mode == 'position_control':
            callback(targetPositions=action)
        elif control_mode == 'velocity_control':
            callback(targetVelocities=action)
        elif control_mode == 'pd_control':
            callback(targetVelocities=action)
        elif control_mode == 'torque_control':
            callback(forces=action)
        else:
            raise ValueError('Control_mode ("%s") not recognized.' % control_mode)

    def _reset_srvs(self, reset_state, type, bodyUniqueId, jointIndices, physicsClientId):
        if type == 'joint_pos':  # Only 1-dof joints are supported here.
            # https://github.com/bulletphysics/bullet3/issues/2803
            velocities = []
            states = self._p.getJointStates(bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=physicsClientId)
            for i, (pos, vel, _, _) in enumerate(states):
                velocities.append([vel])
            self._p.resetJointStatesMultiDof(targetValues=[[s] for s in reset_state], targetVelocities=velocities, bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=physicsClientId)
        elif type == 'joint_vel':
            positions = []
            states = self._p.getJointStates(bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=physicsClientId)
            for i, (pos, vel, _, _) in enumerate(states):
                positions.append([pos])
            self._p.resetJointStatesMultiDof(targetValues=positions, targetVelocities=[[s] for s in reset_state], bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=physicsClientId)
        elif type == 'base_pos':
            _, base_orientation = self._p.getBasePositionAndOrientation(bodyUniqueId=bodyUniqueId, physicsClientId=physicsClientId)
            self._p.resetBasePositionAndOrientation(bodyUniqueId=bodyUniqueId, posObj=reset_state, ornObj=base_orientation, physicsClientId=physicsClientId)
        elif type == 'base_orientation':
            base_pos, _ = self._p.getBasePositionAndOrientation(bodyUniqueId=bodyUniqueId, physicsClientId=physicsClientId)
            self._p.resetBasePositionAndOrientation(bodyUniqueId=bodyUniqueId, posObj=base_pos, ornObj=reset_state, physicsClientId=physicsClientId)
        elif type == 'base_angular_vel':
            base_vel, _ = self._p.getBaseVelocity(bodyUniqueId=bodyUniqueId, physicsClientId=physicsClientId)
            self._p.resetBaseVelocity(objectUniqueId=bodyUniqueId, linearVelocity=base_vel, angularVelocity=reset_state, physicsClientId=physicsClientId)
        elif type == 'base_vel':
            _, base_rate = self._p.getBaseVelocity(bodyUniqueId=bodyUniqueId, physicsClientId=physicsClientId)
            self._p.resetBaseVelocity(objectUniqueId=bodyUniqueId, linearVelocity=reset_state, angularVelocity=base_rate, physicsClientId=physicsClientId)
        else:
            raise ValueError('Type ("%s") not recognized.' % type)

    def _service(self, req, buffer, name, obs_name, message_type):
        return message_type(buffer[name][obs_name])

    def _step(self):
        # Set all actions before stepping the world
        for robot in self._get_actuator_services:
            robot_actuators = self._get_actuator_services[robot]
            for actuator in robot_actuators:
                (get_action_srv, set_action_srvs) = robot_actuators[actuator]
                actions = get_action_srv()
                set_action_srvs(action=list(actions.value))

        # Step the world
        self._world.step()

        # Run sensor & state callbacks todo: parallelize?
        # todo: optimization: only perform callbacks if service is called.
        for name in self._sensor_cbs:  # todo: How to get sensor measurements at different rates?
            for sensor in self._sensor_cbs[name]:
                self._sensor_cbs[name][sensor]()
        for name in self._state_cbs:
            for state in self._state_cbs[name]:
                self._state_cbs[name][state]()
        return True

    def _reset(self):
        # Set all actions before stepping the world
        for robot in self._reset_services:
            robot_resets = self._reset_services[robot]
            for state in robot_resets:
                (get_state_srv, set_reset_srvs) = robot_resets[state]
                state = get_state_srv()
                set_reset_srvs(reset_state=list(state.value))

        # update all observation & state buffers
        for name in self._sensor_cbs:
            for sensor in self._sensor_cbs[name]:
                self._sensor_cbs[name][sensor]()
        for name in self._state_cbs:
            for state in self._state_cbs[name]:
                self._state_cbs[name][state]()
        return True

    def _close(self):
        return True
    
    def _seed(self, seed):
        pass
