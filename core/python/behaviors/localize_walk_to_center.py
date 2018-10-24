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
  def __init__(self, by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon,count):
    super(BeaconCount, self).__init__()
    self.by_beacon = by_beacon
    self.yb_beacon = yb_beacon
    self.yp_beacon = yp_beacon
    self.py_beacon = py_beacon
    self.bp_beacon = bp_beacon
    self.pb_beacon = pb_beacon
    self.count = count
  def ready(self):
    beacon_count = int(self.by_beacon.seen)+int(self.yb_beacon.seen)+int(self.yp_beacon.seen)+int(self.py_beacon.seen)+int(self.bp_beacon.seen)+int(self.pb_beacon.seen)
    return (beacon_count >= self.count)

def BN(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon,count):
  """Two beacons seen"""
  return BeaconCount(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon,count)

def NBN(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon,count):
  """No ball found"""
  return NegationEvent(BeaconCount(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon,count))

class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(-core.DEG_T_RAD*110.0,-10.0,1.5)
    if self.getTime() > 2.5:
      self.finish()

class MoveHead(Node):
  """Search for the ball to the right"""
  def __init__(self, pan, tilt):
    super(MoveHead, self).__init__()
    self.pan = pan
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(-core.DEG_T_RAD*self.pan,self.tilt,3.0)
    if self.getTime() > 4.0:
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
    self.robot = robot
    self.dist = dist
    self.frames_off_center = 0
    self.frames_at_center = 0
    self.time_since_stop = 0
    self.k_t = (0.08, 0.001, 0.05)
    self.k_d = (0.0015, 0.0001, 0.001)
    self.center_integral = 0.0
    self.center_prev = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()
    self.dir = 1.0;
    self.sum_t = 0.0;

  def calc_integral(self, dt):
    distance = np.sqrt(np.power(self.robot.loc.x,2)+np.power(self.robot.loc.y,2))
    bearing = np.arctan2(self.robot.loc.y,self.robot.loc.x) - self.robot.orientation
    center = -np.sign(bearing)*(np.pi - np.abs(bearing))

    self.center_integral = self.center_integral + dt*center
    self.dist_integral = self.dist_integral + dt*(distance - self.dist)

    if abs(self.center_integral) >= 15.0:
      self.center_integral = 15.0*np.sign(self.center_integral)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    commands.setHeadPanTilt(0.0, -5.0, 2.0)
    self.time_current = time.clock()
    dt = self.time_current - self.time_last

    # if self.sum_t < 2.5:
    #   self.sum_t = self.sum_t + dt
    # else:
    #   self.sum_t = 0.0
    #   self.dir = -1.0 * self.dir

    #print("sum t: %f, dir: %f" %(self.sum_t, self.dir))

    distance = np.sqrt(np.power(self.robot.loc.x,2)+np.power(self.robot.loc.y,2))
    bearing = np.arctan2(self.robot.loc.y,self.robot.loc.x) - self.robot.orientation
    center = -np.sign(bearing)*(np.pi - np.abs(bearing))

    if (np.pi - abs(center)) < 0.1:
      center = np.sign(self.center_prev) * abs(center)

    self.calc_integral(dt)
    print("Loc x: %f, Loc y: %f"%(self.robot.loc.x, self.robot.loc.y))
    # print("atan: %f, orientation: %f"%(core.RAD_T_DEG * np.arctan2(self.robot.loc.y,self.robot.loc.x),core.RAD_T_DEG * self.robot.orientation))
    # if (self.bearing + 2 * np.pi < abs(self.bearing)):
    #   self.bearing = self.bearing + 2 * np.pi

    if dt == 0:
      theta_cont = self.k_t[0] * center + self.k_t[1] * self.center_integral
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
    else:
      theta_cont = self.k_t[0] * center + self.k_t[1] * self.center_integral + self.k_t[2] * (center - self.center_prev) / dt
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    # print("center: %f, center_integral: %f, center derivative: %f" %(center, self.k_t[1] * self.center_integral, self.k_t[2]*(center - self.center_prev) / dt))
    # print("center_deriv: %f, dsit deriv: %f" %(((center - self.center_prev) / dt), ((distance - self.dist_prev) / dt)))
    if ((center - self.center_prev) / dt) > 5.0:
        theta_cont = 0.0

    if (distance < 400.0 and abs(center) > 0.5) or abs(center) > 0.8:
      self.frames_off_center += 1
      # Control only the heading of the robot and not the velocity
      # print("center > 0.3: Theta cont: %f"%(0.1*np.sign(center)))
      if (self.frames_off_center > 5):
        print("Turn toward center")
        commands.setWalkVelocity(0.0, 0.0, 0.5*np.sign(center))
    else:
      self.frames_off_center = 0
    # Control both heading and velocity
      if abs(distance) >= 400.0:
        print("d > 600.0, Loc x,y: %f, %f, center: %f, Theta cont: %f"%(self.robot.loc.x, self.robot.loc.y,center,theta_cont))
        commands.setWalkVelocity(0.5, 0.0, 0.0)
      else:
        print("d < 600.0, %f, Loc x,y: %f, %f, center: %f, Theta cont: %f, dist cont: %f"%(distance,self.robot.loc.x, self.robot.loc.y, center,theta_cont, dist_cont))
        commands.setWalkVelocity(0.3, 0.0, 0.0)
    self.center_prev = center
    self.dist_prev = distance
    self.time_last = self.time_current

    if abs(distance - self.dist) < 50.0:
      self.frames_at_center += 1
      print("Thinks at center!")
      if (self.frames_at_center >= 5):
        self.finish()
    else:
      self.frames_at_center = 0



class Stand(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)

class Walk(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)

class LookAtBeacon(Node):
  """Look at beacon bearing avg if seen"""
  def __init__(self, by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon):
    super(BeaconCount, self).__init__()
    self.by_beacon = by_beacon
    self.yb_beacon = yb_beacon
    self.yp_beacon = yp_beacon
    self.py_beacon = py_beacon
    self.bp_beacon = bp_beacon
    self.pb_beacon = pb_beacon
  def run(self):
    commands.stand()
    if self.by_beacon.seen:
      bearing = self.by_beacon.visionBearing
    commands.setHeadPanTilt(bearing,0.0,1.5)

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
    startMoveHeadLeft = MoveHead(110.0,-5.0)
    startMoveHeadRight = MoveHead(-110.0,-5.0)
    moveHeadLeft = MoveHead(110.0,-5.0)
    moveHeadRight = MoveHead(-110.0,-5.0)
    turnInPlace = TurnInPlace()
    stand1 = Stand()
    stand2 = Stand()
    #self.add_transition(rdy,C,startMoveHeadLeft,C,startMoveHeadRight,C,turnInPlace,C,startMoveHeadLeft)
    #self.add_transition(startMoveHeadLeft, BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon,2),goToCenter)
    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight,C,goToCenter)

    self.add_transition(goToCenter,T(3.0),moveHeadLeft)
    #self.add_transition(moveHeadLeft,C,moveHeadRight,C,goToCenter)

    # self.add_transition(goToCenter,NBN(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon),turnInPlace,C,moveHeadLeft)
    # self.add_transition(goToCenter,S("move"),moveHeadLeft)
    # self.add_transition(moveHeadLeft,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    # self.add_transition(moveHeadRight,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    # self.add_transition(turnInPlace,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    # self.add_transition(goToCenter,NBN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),moveHeadLeft)
    self.add_transition(goToCenter,C,stand1,T(10.0),moveHeadLeft)#,T(10.0),stand2,NBN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),moveHeadLeft)