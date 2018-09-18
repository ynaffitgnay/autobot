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

class BeaconSeen(Event):
  """Event that fires if beacon is seen"""
  def __init__(self, beacon_by, beacon_yb, beacon_bp, beacon_pb, beacon_py, beacon_yp):
    super(BeaconSeen, self).__init__()
    self.found = False
  def ready(self):
    return self.found

def BC(beacon_by=None, beacon_yb=None, beacon_bp=None, beacon_pb=None, beacon_py=None, beacon_yp=None):
  """beacon found"""
  return BeaconSeen(beacon_by, beacon_yb, beacon_bp, beacon_pb, beacon_py, beacon_yp)

class GetReady(Node):
  def run(self):
    commands.standStraight()
    commands.setHeadTilt(10.0)
    if self.getTime() > 1.5:
      self.finish()

class TurnInPlace(Node):
  """Controller node for tracking the beacon"""
  def __init__(self, beacon_by=None, beacon_yb=None, beacon_bp=None, beacon_pb=None, beacon_py=None, beacon_yp=None):
    super(GetReady, self)__init__()
    self.beacon_by = beacon_by
    self.beacon_yb = beacon_yb
    self.beacon_bp = beacon_bp
    self.beacon_pb = beacon_pb
    self.beacon_py = beacon_py
    self.beacon_yp = beacon_yp
  def run(self,beacon_by=None, beacon_yb=None, beacon_bp=None, beacon_pb=None, beacon_py=None, beacon_yp=None):
    if beacon_by.seen:
      print("Blue yellow at: %f\t",beacon_by.distance)
    if beacon_by.seen:
      print("Yellow blue at: %f\t",beacon_yb.distance)
    if beacon_by.seen:
      print("Blue pink at: %f\t",beacon_bp.distance)
    if beacon_by.seen:
      print("Pink yellow at: %f\t",beacon_py.distance)
    if beacon_by.seen:
      print("Yellow pink at: %f",beacon_yp.distance)
    if (not beacon_by.seen not beacon_by.seen not beacon_by.seen not beacon_by.seen not beacon_by.seen):
      print("Nothing to see here\n")
    print("\n")
    # commands.setWalkVelocity(0.0,0.0,-0.2)

class Playing(LoopingStateMachine):
  def setup(self):
    beacon_by = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW)
    beacon_yb = memory.world_objects.getObjP  tr(core.WO_BEACON_YELLOW_BLUE)
    beacon_bp = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
    beacon_pb = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE)
    beacon_py = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
    beacon_yp = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)
    rdy = GetReady()
    turnInPlace = TurnInPlace(beacon_by,beacon_yb,beacon_bp,beacon_pb,beacon_py,beacon_yp)
    self.add_transition(rdy,C,turnInPlace)
    self.add_transition(turnInPlace,C,turnInPlace)