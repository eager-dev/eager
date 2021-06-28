import os
import subprocess
import time
import rospy

class WebotsRunner(object):
    def __init__(self, world, mode, no_gui, virtual_display):
        if 'WEBOTS_HOME' not in os.environ:
            raise Exception('WEBOTS_HOME not defined')

        self.world = world
        command = [os.path.join(os.environ['WEBOTS_HOME'], 'webots'), '--mode=' + mode, world]

        command.append('--batch') # Always disable popups

        if no_gui or virtual_display:
            command.append('--stdout')
            command.append('--stderr')
            command.append('--no-rendering')
            command.append('--minimize')
        
        if virtual_display:
            self.process = subprocess.Popen('Xvfb :99 & ' + ' '.join(command), shell=True, env = dict(
                os.environ,
                WEBOTS_DISABLE_SAVE_SCREEN_PERSPECTIVE_ON_CLOSE='true',
                DEBIAN_FRONTEND='noninteractive',
                DISPLAY=':99',
                LIBGL_ALWAYS_SOFTWARE='true'))
        else:
            self.process = subprocess.Popen(command, env = dict(
                os.environ,
                WEBOTS_DISABLE_SAVE_SCREEN_PERSPECTIVE_ON_CLOSE='true'
            ))
    
    def quit(self):
        self.process.terminate()
        tries = 0
        while self.process.poll() is None:
            time.sleep(0.1)
            tries += 1
            if tries >= 5:
                self.process.kill()
                rospy.logwarn("Unable to terminate WeBots, sigkill")
        self.process = None
        time.sleep(5) # The process holding the world files takes some time to stop 
        os.remove(os.path.dirname(self.world) + '/.' + os.path.splitext(os.path.basename(self.world))[0] + '.wbproj')
    
    def __del__(self):
        if self.process is not None:
            self.quit()