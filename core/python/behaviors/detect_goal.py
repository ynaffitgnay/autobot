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
    commands.stand()
    if self.getTime() > 1.5:
      self.finish()

class DetectGoal(Node):
  def run(self):
    goal = memory.world_objects.getObjPtr(core.WO_OWN_GOAL)
    if goal.seen:
      print('Goal!')
    else:
      print('No goal!')
    if self.getTime() > 10.0:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    stand = Stand()
    detect = DetectGoal()
    self.trans(stand,C,detect,C,detect)
