from eager_core.utils.file_utils import substitute_xml_args
from eager_core.srv import BoxFloat32Data, BoxFloat32DataResponse
from eager_core.srv import BoxUInt8Data, BoxUInt8DataResponse
from eager_core.msg import Space

def get_message_from_def(object):
    if object['type'] == 'boxf32':
        return BoxFloat32Data
    elif object['type'] == 'boxu8':
        return BoxUInt8Data
    else:
        raise NotImplementedError('Unknown space type:', object['type'])

def get_response_from_def(object):
    if object['type'] == 'boxf32':
        return BoxFloat32DataResponse
    elif object['type'] == 'boxu8':
        return BoxUInt8DataResponse
    else:
        raise NotImplementedError('Unknown space type:', object['type'])

def get_value_from_def(object):
    if object['type'] == 'boxf32':
        return 0.0
    elif object['type'] == 'boxu8':
        return 0
    else:
        raise NotImplementedError('Unknown message type:', object['type'])

def get_length_from_def(object):
    if 'shape' in object and object['shape']:
        length = 1
        for element in object['shape']:
            length *= element
    else:
        length = len(object['high'])
    return length

def get_dtype_from_def(object):
    if 'f32' in object['type'] == 'boxf32':
        return 'float32'
    elif 'u8' in object['type']:
        return 'uint8'
    else:
        raise NotImplementedError('Unknown space type:', object['type'])

def get_def_from_space_msg(msg: Space) -> dict:
    object = dict()
    object['type'] = msg.type
    object['high'] = [] 
    for value in msg.high:
        object['high'].append(substitute_xml_args(value))
    object['low'] = []
    for value in msg.low:
        object['low'].append(substitute_xml_args(value))
    return object
    
def get_space_msg_from_def(object: dict) -> Space:
    msg = Space()
    msg.type = object['type']
    msg.high = []
    for value in object['high']:
    	msg.high.append(str(value))
    msg.low = []
    for value in object['low']:
    	msg.low.append(str(value))
    return msg