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
      commands.setHeadPan(-4.0,2.0)
      if self.getTime() > 2.0:
        commands.setHeadPan(4.0,4.0)
      print('getTime produces %f' % self.getTime())
      if self.getTime() > 4.0:
        memory.speech.say("moved my head")
        self.finish()
      print('debug 4')

  class Shutdown(Node):
    def run(self):
      print('debug 1')
      memory.speech.say("turning off")
      commands.setHeadPan(0.0,2.0)
      commands.setStiffness(cfgstiff.Zero)
      print('done turning off')
      print('getTime produces %f' % self.getTime())
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        print('debug 2')
        self.finish()
      print('debug 3')

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      # if self.getTime() > 2.0:
      memory.speech.say("turned off stiffness")
      self.finish()

  def setup(self):
    moveHead = self.MoveHead()
    shutdown = self.Shutdown()
    sit = pose.Sit()
    off = self.Off()
    self.trans(sit,C,moveHead,C,shutdown,C,off)

