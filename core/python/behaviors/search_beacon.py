"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import math
import core
import pose
import head
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, S, LoopingStateMachine, EventNode, Event, NegationEvent

class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadTilt(5.0)
    if self.getTime() > 1.5:
      self.finish()

class TurnInPlace(Node):
  def run(self,beacon_by=None, beacon_yb=None, beacon_bp=None, beacon_pb=None, beacon_py=None, beacon_yp=None):
    commands.setWalkVelocity(0.0,0.0,-0.1)

class Playing(LoopingStateMachine):
  def setup(self):
    rdy = GetReady()
    turnInPlace = TurnInPlace()
    self.add_transition(rdy,C,turnInPlace)
    self.add_transition(turnInPlace,C,turnInPlace)