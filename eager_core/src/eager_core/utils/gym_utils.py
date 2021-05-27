import gym
import numpy as np
from gym.spaces.box import Box
from typing import Type
from eager_core.srv import BoxSpace, BoxSpaceResponse

def get_space_from_def(object: dict) -> gym.Space:
    if object['type'] == 'box':
        high = np.array(object['high'])
        low = np.array(object['low'])
        return Box(low, high)
    else:
        raise NotImplementedError('Unknown space type.')

def get_message_from_space(space_class: Type[gym.Space]) -> Type:
    if space_class is Box:
        return BoxSpace
    else:
        raise NotImplementedError('Unknown space type.')

def get_response_from_space(space_class: Type[gym.Space]) -> Type:
    if space_class is Box:
        return BoxSpaceResponse
    else:
        raise NotImplementedError('Unknown space type.')