"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import time
import numpy as np
import math
import core
import pose
import head
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, S, LoopingStateMachine, StateMachine, EventNode, Event, NegationEvent


class BeaconCount(Event):
  """Event that fires if Robot is seen"""
  def __init__(self, by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon):
    super(BeaconCount, self).__init__()
    self.by_beacon = by_beacon
    self.yb_beacon = yb_beacon
    self.yp_beacon = yp_beacon
    self.py_beacon = py_beacon
    self.bp_beacon = bp_beacon
    self.pb_beacon = pb_beacon
  def ready(self):
    count = int(self.by_beacon.seen)+int(self.yb_beacon.seen)+int(self.yp_beacon.seen)+int(self.py_beacon.seen)+int(self.bp_beacon.seen)+int(self.pb_beacon.seen)
    return (count >= 2)

def BN(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon):
  """Two beacons seen"""
  return BeaconCount(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon)

def NBN(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon):
  """No ball found"""
  return NegationEvent(BeaconCount(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon))

class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(-core.DEG_T_RAD*110.0,-10.0,1.5)
    if self.getTime() > 2.5:
      self.finish()

class MoveHeadLeft(Node):
  """Search for the ball to the left"""
  def __init__(self, tilt):
    super(MoveHeadLeft, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(core.DEG_T_RAD*85.0,self.tilt,1.0)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 1.2:
      self.finish()

class MoveHeadRight(Node):
  """Search for the ball to the right"""
  def __init__(self, tilt):
    super(MoveHeadRight, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(-core.DEG_T_RAD*85.0,self.tilt,1.0)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 1.2:
      self.finish()

class TurnInPlace(Node):
  """Turn in place if ball not found"""
  def run(self):
    commands.setWalkVelocity(0.0,0.0,-0.4)
    if self.getTime() > 2.5:
      self.finish()

class GoToCenter(Node):
  """Face the ball"""
  def __init__(self, robot, dist):
    super(GoToCenter, self).__init__()
    self.loc = robot.loc
    self.orientation = robot.orientation
    self.dist = dist
    self.k_t = (0.7, 0.01, 0.1)
    self.k_d = (0.001, 0.0001, 0.0001)
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()

  def calc_integral(self, dt):
    self.theta_integral = self.theta_integral + dt*(self.orientation)
    self.dist_integral = self.dist_integral + dt*(self.robotDist - self.dist)
    if abs(self.theta_integral) >= 15.0:
      self.theta_integral = 15.0*np.sign(self.theta_integral)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    self.robotDist = np.sqrt(np.power(self.loc.x,2)+np.power(self.loc.y,2))
    self.bearing = np.arctan2(0.0-self.loc.y,0.0-self.loc.x)-self.orientation
    if (self.bearing + 2 * np.pi < abs(self.bearing)):
      self.bearing = self.bearing + 2 * np.pi    
    self.time_current = time.clock()
    dt = self.time_current - self.time_last
    self.calc_integral(dt)
    
    bearing = self.bearing
    distance = self.robotDist
    elevation = 5.0
    commands.setHeadPanTilt(bearing, -elevation, 1.5)
    if dt == 0:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
    else:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral + self.k_t[2] *(bearing - self.theta_prev) / dt
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    
    if abs(bearing) >=0.3:
      # Control only the heading of the robot and not the velocity
      commands.setWalkVelocity(0.0, 0.0, 0.4*np.sign(bearing))
    else:
      # Control both heading and velocity
      if abs(distance) >= 600.0:
        commands.setWalkVelocity(1.0, 0.0, theta_cont)
      else:
        commands.setWalkVelocity(dist_cont, 0.0, theta_cont)
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current
    if abs(self.robotDist) < 50.0:
      self.finish()

class Stand(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)

class Playing(LoopingStateMachine):
  def setup(self):
    robot = memory.world_objects.getObjPtr(core.WO_TEAM5)
    by_beacon = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW);
    yb_beacon = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE)
    yp_beacon = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK);
    py_beacon = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW);
    bp_beacon = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK);
    pb_beacon = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE);
    goToCenter = GoToCenter(robot, 0.0)
    rdy = GetReady()
    moveHeadLeft = MoveHeadLeft(-5.0)
    moveHeadRight = MoveHeadRight(-5.0)
    turnInPlace = TurnInPlace()
    stand1 = Stand()
    stand2 = Stand()
    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight,C,turnInPlace,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    self.add_transition(moveHeadRight,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    self.add_transition(turnInPlace,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    self.add_transition(goToCenter,NBN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),moveHeadLeft)
    self.add_transition(goToCenter,C,stand1,T(10.0),stand2,NBN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),moveHeadLeft)