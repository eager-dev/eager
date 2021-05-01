from physics_bridge import PhysicsBridge

class WeBotsBridge(PhysicsBridge):

    def _register_object(self):
        return True

    def _step(self):
        return True

    def _reset(self):
        return True

    def _close(self):
        return True