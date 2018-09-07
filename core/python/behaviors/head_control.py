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
      # commands.setHeadTilt(20.0)
      commands.setHeadPan(-3.0/2,2.0)
      # print("getTime: %f" % (self.getTime()))
      if self.getTime() > 4.0:
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
      # head.MoveHead(0.0,0.0,1.5)
      # print("getTime shutdown 1: %f" % (self.getTime()))
      # commands.standStraight()
      commands.setHeadPan(0.0,2.0)
      # print("getTime shutdown 2: %f" % (self.getTime()))
      if self.getTime() > 4.0:
        memory.speech.say("shutdown")
        commands.setStiffness(cfgstiff.Zero)
        self.finish()

  # class Off(Node):
  #   def run(self):
  #     # print("getTime2: %f" % (self.getTime()))
  #     memory.speech.say("turning off stiffness")
  #     commands.setStiffness(cfgstiff.Zero)
  #     if self.getTime() > 2.0:
  #       self.finish()

  def setup(self):
    moveHead = self.HeadMove()
    # off = self.Off()
    sit = pose.Sit()
    stand = self.Stand()
    shutdown = self.Shutdown()
    # self.trans(sit,C,moveHead)
    self.trans(stand,C,moveHead)
    self.trans(moveHead,C,shutdown)
    # self.trans(shutdown,C,sit)