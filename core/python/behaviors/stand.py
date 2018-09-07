"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, StateMachine

class Playing(StateMachine):
    class Stand(Node):
        def run(self):
         	memory.speech.say("demonstrate standing")
            commands.standStraight()
            if self.getTime() > 1.5:
                self.finish()

    class Off(Node):
        def run(self):
            memory.speech.say("turning off stiffness")
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                self.finish()

    def setup(self):
        stand = self.Stand()
        sit = pose.Sit()
        off = self.Off()
        self.trans(sit, C, stand, C, off)
