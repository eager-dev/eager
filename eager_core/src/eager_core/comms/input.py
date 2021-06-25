import rospy
import threading
import sys
from collections import deque

TIMEOUT_FACTOR = 10

class Input(object):
    def __init__(self, topic, msg_class, input_rate):
        super(Input, self).__init__()

        self.input_dt = 1.0/input_rate

        self.expected_input = 0
        if sys.version_info >= (3, 0):
            self.condition = threading.Condition()
        else:
            self.condition = ConditionWithReturn()
        self.callback_queue = deque()
        self.current_data = None

        self.sub = rospy.Subscriber(topic, msg_class, callback=self._callback)

    def _callback(self, msg):
        with self.condition:
            self.callback_queue.append(msg.data)
            self.condition.notify_all()

    def get_inputs_in_step(self, time, dt):

        time_since_input = (time % self.input_dt) + dt
        return int(time_since_input / self.input_dt)

    def get(self, time, dt):
        self.expected_input = self.get_inputs_in_step(time, dt)

        if self.expected_input == 0:
            return [self.current_data]

        with self.condition:
            while len(self.callback_queue) < self.expected_input:
                if not self.condition.wait(self.input_dt*10):
                    raise Exception("Did not get data after {} seconds!".format(self.input_dt*TIMEOUT_FACTOR))
            output = [self.callback_queue.pop() for _ in range(self.expected_input)]
            self.current_data = output[-1]
            return output    

if sys.version_info < (3, 0):
    class ConditionWithReturn(threading._Condition):
        """
        Overrides python 2 Condition wait function to return wether the condition timed out.
        This makes the behaviour similar to python 3.
        """
        def wait(self, timeout=None):
            if not self._is_owned():
                raise RuntimeError("cannot wait on un-acquired lock")
            waiter = threading._allocate_lock()
            waiter.acquire()
            self._Condition__waiters.append(waiter)
            saved_state = self._release_save()
            try:
                if timeout is None:
                    waiter.acquire()
                else:
                    endtime = threading._time() + timeout
                    delay = 0.0005
                    while True:
                        gotit = waiter.acquire(0)
                        if gotit:
                            break
                        remaining = endtime - threading._time()
                        if remaining <= 0:
                            break
                        delay = min(delay * 2, remaining, .05)
                        threading._sleep(delay)
                    if not gotit:
                        try:
                            self._Condition__waiters.remove(waiter)
                        except ValueError:
                            pass
                    return gotit
            finally:
                self._acquire_restore(saved_state)