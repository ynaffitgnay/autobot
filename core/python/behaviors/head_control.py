"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import head
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, StateMachine

class Playing(StateMachine):
  class MoveHead(Node):
    def run(self):
      head.moveHead(4.0,1.0)
      if self.getTime() > 1.0:
        memory.speech.say("moved my head")
        self.finish()

  class Shutdown(Node):
    def run(self):
      head.moveHead()
      if self.getTime() > 2.0:
        memory.speech.say("turning off")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    moveHead = self.MoveHead()
    shutdown = self.Shutdown()
    sit = pose.Sit()
    off = self.Off()
    self.trans(moveHead,C,shutdown,C,off)

