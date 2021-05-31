import gym
import numpy as np
from gym.spaces.box import Box
from typing import Type
from eager_core.srv import BoxFloat32Data, BoxFloat32DataResponse
from eager_core.srv import BoxUInt8Data, BoxUInt8DataResponse

def get_space_from_def(object: dict) -> gym.Space:
    if object['type'] == 'boxf32':
        if 'shape' in object:
            high = object['high']
            low = object['low']
            shape = object['shape']
            return Box(low, high, shape, dtype=np.float32)
        else:
            high = np.array(object['high'])
            low = np.array(object['low'])
            return Box(low, high, dtype=np.float32)
    elif object['type'] == 'boxu8':
        if 'shape' in object:
            high = object['high']
            low = object['low']
            shape = object['shape']
            return Box(low, high, shape, dtype=np.uint8)
        else:
            high = np.array(object['high'])
            low = np.array(object['low'])
            return Box(low, high, dtype=np.uint8)
    else:
        raise NotImplementedError('Unknown space type:', object['type'])

def get_message_from_space(space: gym.Space) -> Type:
    if type(space) is Box:
        if space.dtype is np.dtype(np.float32):
            return BoxFloat32Data
        elif space.dtype is np.dtype(np.uint8):
            return BoxUInt8Data
        else:
            raise NotImplementedError('Unknown space type:', space.dtype)
    else:
        raise NotImplementedError('Unknown space type:', type(space))

def get_response_from_space(space: gym.Space) -> Type:
    if type(space) is Box:
        if space.dtype is np.dtype(np.float32):
            return BoxFloat32DataResponse
        elif space.dtype is np.dtype(np.uint8):
            return BoxUInt8DataResponse
        else:
            raise NotImplementedError('Unknown space type:', space.dtype)
    else:
        raise NotImplementedError('Unknown space type:', type(space))