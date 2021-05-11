from sensor_msgs.msg import JointState



def create_sensor_converter(sensor_type):
    def convert_joint_state_to_list(joint_state):
        data = joint_state.position
        return data
    if isinstance(sensor_type, JointState):
        return convert_joint_state_to_list
