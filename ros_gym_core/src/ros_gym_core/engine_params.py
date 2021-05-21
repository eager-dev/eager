import abc


# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()})

class EngineParams(ABC):
    def __init__(self, **kwargs):
        # Iterates over provided arguments and sets the provided arguments as class properties
        for key, value in kwargs.items():
            setattr(self, key, value)
