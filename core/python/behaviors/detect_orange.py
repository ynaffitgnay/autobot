"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import core
import pose
import head
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, LoopingStateMachine

ledsC = core.ledsC

# class LightEars(Node):
#   def run(self):
#     # print('Debug 1')
#     ledsC.frontLeftEar(1)
#     if self.getTime() > 4.0:
#       print('More than 1 sec')
#       self.finish()

class Stand(Node):
  def run(self):
    commands.standStraight()
    if self.getTime() > 1.5:
      self.finish()

class DetectBall(Node):
  def run(self):
    if memory.world_objects.getObjPtr(core.WO_BALL).seen:
      print('Orange!')
    else:
      print('No orange!')
    if self.getTime() > 10.0:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    stand = Stand()
    detect = DetectBall()
