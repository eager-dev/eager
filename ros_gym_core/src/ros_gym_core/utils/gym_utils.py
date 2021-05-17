import gym
import numpy as np
from gym.spaces.box import Box

def get_space_from_def(object: dict) -> gym.Space:
    if object['type'] == 'box':
        high = np.array(object['high'])
        low = np.array(object['low'])
        return Box(low, high)
    else:
        raise Exception('Unknown space type.')