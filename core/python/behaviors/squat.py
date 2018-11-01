"""Simple behavior that stands, squats, and then sits down."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import commands
import cfgstiff
from state_machine import StateMachine, Node, C


class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 3.0:
                self.finish()

    class Squat(Node):
        def run(self):
            if self.getFrames() <= 3:
                return pose.squat2()
            if self.getFrames() > 10 and not memory.kick_request.kick_running_:
                self.finish()

    class Walk(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    def setup(self):
        self.trans(self.Stand(), C, self.Squat(), C, self.Stand(),
                   C, pose.Sit(), C, self.Off())
