from eager_core.srv import BoxFloat32Data, BoxFloat32DataResponse
from eager_core.srv import BoxUInt8Data, BoxUInt8DataResponse

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

def get_value_from_message(message_type):
    if message_type is BoxFloat32Data:
        return 0.0
    elif message_type is BoxUInt8Data:
        return 0
    else:
        raise NotImplementedError('Unknown message type:', message_type)