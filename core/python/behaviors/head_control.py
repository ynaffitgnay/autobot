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
  class HeadMove(Node):
    def run(self):
#      head.MoveHead(0.0,-4.0,1.0)
      commands.setHeadPan(-3.0,1.5)
      print("getTime: %f" % (self.getTime()))
      if self.getTime() > 3.0:
        memory.speech.say("moved my head")
        self.finish()

  class Shutdown(Node):
    def run(self):
      head.MoveHead(0.0,0.0,1.5)
      if self.getTime() > 2.0:
        memory.speech.say("turning off")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      print("getTime2: %f" % (self.getTime()))
      if self.getTime() > 5.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    
    moveHead = self.HeadMove()
    off = self.Off()
    sit = pose.Sit()
    shutdown = self.Shutdown()
    self.trans(sit,C,moveHead,C,sit,C,shutdown,C,sit,C,off)

