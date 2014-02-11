#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import sys
import signal
import tempfile
import subprocess

import rospy
import rocon_python_comms
import rocon_utilities

import rocon_tutorial_msgs.msg as rocon_tutorial_msgs

##############################################################################
# Methods
##############################################################################

class ProcessInfo(object):
    def __init__(self, process, temp_file):
        self.process = process
        self.temp_file = temp_file

class TurtleHerder(object):
    __slots__ = [
        'turtles',
        '_process_info'
    ]
    _port_counter = 11

    def __init__(self, turtles=[]):
        self._process_info = []
        self.turtles = []
        if turtles:
            self.spawn_turtles(turtles)
        
    def _prepare_rocon_launch_text(self, turtles):
        port = 11
        launch_text = '<concert>\n'
        for name in turtles:
            launch_text += '  <launch title="%s:114%s" package="turtle_concert" name="turtle.launch" port="114%s">\n' % (name, str(port), str(port))
            launch_text += '    <arg name="turtle_name" value="%s"/>\n' % name
            launch_text += '  </launch>\n'
            port = port + 1
        launch_text += '</concert>\n'
        return launch_text

    def spawn_turtles(self, turtles):
        '''
          Whip up a rocon launcher for spawning turtles. This is fairly
          limited right now - you're only allowed one user
        '''
        if turtles:
            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            rocon_launch_text = self._prepare_rocon_launch_text(turtles)
            print("%s" % rocon_launch_text)
            temp.write(rocon_launch_text)
            temp.close()  # unlink it later
            #print("Starting process %s" % ['rocon_launch', temp.name, '--screen'])
            #process = subprocess.Popen(['rocon_launch', temp.name, '--screen'])
            #process = rocon_utilities.Popen(['konsole'])
            #process = subprocess.Popen('konsole -p tabtitle=Dude --nofork -e "/bin/bash" -c ls', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            #process = subprocess.Popen(['ls', '-l'])
            #process = subprocess.Popen('/usr/bin/konsole --nofork', shell=True)
            #process = subprocess.Popen(['konsole'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            #process = subprocess.Popen(['gnome-terminal']) # works
            #process = subprocess.Popen(['eog'])  # works
            #process = subprocess.Popen('eog', shell=True)  # works
            #process = subprocess.Popen('gnome-terminal', shell=True)  # works
            #process = subprocess.Popen('konsole --nofork', shell=True, stdout=sys.stdout, stderr=sys.stdout)
            process = subprocess.Popen(['konsole', '--nofork', '--hold',  '-e', '/bin/bash', '-c', 'ls'])
            print("Poll: %s" % process.poll())
            #process = subprocess.Popen(['konsole', '--nofork', '--hold',  '-e', '/bin/bash', '-c', 'ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            #print("Process communicate [0]: %s" % process.communicate()[0])
            #print("Process communicate [1]: %s" % process.communicate()[1])
            print("Return code: %s" % process.returncode)
            #process = subprocess.Popen(['konsole', '--nofork'])
            #process = subprocess.Popen(['konsole', '-p', 'tabtitle=Dude', '--hold', '-e', '/bin/bash', '-c', 'ls'])
            print("Pid: %s" % process.pid)
            #process = subprocess.Popen(['konsole', '-p', 'tabtitle=Dude', '--nofork', '--hold', '-e', "/bin/bash"])
            self.turtles.extend(turtles)
            self._process_info.append(ProcessInfo(process, temp))
                
    def shutdown(self):
        for process_info in self._process_info:
            print("Pid: %s" % process_info.process.pid)
            #process_info.process.terminate()
            process_info.process.send_signal(signal.SIGINT)
            #send_signal(signal.SIGINT)
            os.unlink(process_info.temp_file.name)

    def signal_handler(self, sig, frame):
        for process_info in self._process_info:
            print("Pid: %s" % process_info.process.pid)
            process_info.process.terminate()
            try:
                #process_info.process.terminate()
                process_info.process.send_signal(signal.SIGHUP)
            except OSError:
                print("OSERROR on SIGHUP")
            os.unlink(process_info.temp_file.name)
            #wait_pid(pid)

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    process = subprocess.Popen(['konsole', '--nofork', '--hold',  '-e', '/bin/bash', '-c', 'ls'])
    #rospy.init_node('spawn_kobuki')
    # This would ideally have ros service pairs for
    # spawning and killing turtles
    #turtle_herder = TurtleHerder()
    #signal.signal(signal.SIGINT, turtle_herder.signal_handler)
    #turtle_herder.spawn_turtles(rospy.get_param('~turtles', ['kobuki', 'guimul']))
    #rospy.spin()
    #turtle_herder.shutdown()
