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
  class Curve(Node):
    def run(self):
      commands.setWalkVelocity(0.3, 0, 0.3)

  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        memory.speech.say("playing stand complete")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    curve = self.Curve()
    sit = pose.Sit()
    stand = self.Stand()
    off = self.Off()
    self.trans(stand,C,curve,T(3.0),sit,C,off)
