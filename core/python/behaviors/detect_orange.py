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

class Stand(Node):
  def run(self):
    commands.standStraight()
    if self.getTime() > 1.5:
      self.finish()

class DetectBall(Node):
  def run(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      print('Orange!')
    else:
      print('No orange!')
    if self.getTime() > 10.0:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    stand = Stand()
    detect = DetectBall()
    self.trans(stand,C,detect,C,detect)
