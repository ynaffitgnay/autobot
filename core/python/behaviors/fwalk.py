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
  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.3, 0, 0)


  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    walk = self.Walk()
    sit = pose.Sit()
    stand = commands.stand()
    off = self.Off()
    self.trans(sit,C,stand,C,walk,T(3.0),sit,C,off)
