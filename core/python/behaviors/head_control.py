"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import pose
import head
import commands
import cfgstiff
from state_machine import Node, C, StateMachine

class Playing(StateMachine):
  class HeadMove(Node):
    def run(self):
      commands.setHeadTilt(20.0)
      commands.setHeadPan(-1.0,2.0)
      if self.getTime() > 6.0:
        memory.speech.say("moved my head")
        self.finish()

  class Stand(Node):
    def run(self):
      commands.standStraight()
      if self.getTime() > 2.5:
        memory.speech.say("demonstrate head turning")
        self.finish()

  class Shutdown(Node):
    def run(self):
      commands.setHeadPan(0.0,2.0)
      if self.getTime() > 6.0:
        memory.speech.say("shutdown")
        commands.setStiffness(cfgstiff.Zero)
        self.finish()

  def setup(self):
    moveHead = self.HeadMove()
    stand = self.Stand()
    shutdown = self.Shutdown()
    self.trans(stand,C,moveHead)
    self.trans(moveHead,C,shutdown)
