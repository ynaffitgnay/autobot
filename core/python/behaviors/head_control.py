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
  class MoveHead(Node):
    def run(self):
      commands.setHeadPan(-4,1)
      if self.getTime() > 1.5:
        memory.speech.say("moved my head")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      print('done turning off')
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    moveHead = self.MoveHead()
    sit = pose.Sit()
    off = self.Off()
    self.trans(sit,C,moveHead,C,off)

