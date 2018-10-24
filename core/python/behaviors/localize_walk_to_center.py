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
    commands.setHeadPanTilt(-core.DEG_T_RAD*110.0,0.0,1.5)
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
  def __init__(self, robot, dist, by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon):
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
    #self.dir = 1.0;
    self.total_time = 0.0
    self.time_for_turn_head = 0.0
    self.pan = core.DEG_T_RAD * 110.0
    self.tilt = 0.0
    self.time_for_panning = 12.0
    self.time_for_turn_in_place = 0.0
    self.time_to_walk = 4.0
    self.frames_away_from_front = 0
    self.dont_turn_toward_center = False
    self.turned_toward_front = False
    self.turned_toward_center = False

    self.by_beacon = by_beacon
    self.yb_beacon = yb_beacon
    self.yp_beacon = yp_beacon
    self.py_beacon = py_beacon
    self.bp_beacon = bp_beacon
    self.pb_beacon = pb_beacon
    self.count = 2
    self.beacons_seen = set()

  def calc_integral(self, dt):
    distance = np.sqrt(np.power(self.robot.loc.x,2)+np.power(self.robot.loc.y,2))
    bearing = np.arctan2(self.robot.loc.y,self.robot.loc.x) - self.robot.orientation
    center = -np.sign(bearing)*(np.pi - np.abs(bearing))

    self.center_integral = self.center_integral + dt*center
    self.dist_integral = self.dist_integral + dt*(distance - self.dist)

    if abs(self.center_integral) >= 15.0:
      self.center_integral = 15.0*np.sign(self.center_integral)
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    #commands.setHeadPanTilt(0.0, -5.0, 2.0)
    self.time_current = time.clock()
    dt = self.time_current - self.time_last

    if (dt > 1.0):
      self.turned_toward_front = False
      self.turned_toward_center = False
      dt = 0.0

    if (self.total_time == 0.0):
      self.beacons_seen.clear()
    
    self.total_time += dt

    #print("dt: %f, total_time: %f"%(dt, self.total_time))

    if self.by_beacon.seen:
      self.beacons_seen.add("BY")
    if self.yb_beacon.seen:
      self.beacons_seen.add("YB")
    if self.yp_beacon.seen:
      self.beacons_seen.add("YP")
    if self.py_beacon.seen:
      self.beacons_seen.add("PY") 
    if self.bp_beacon.seen:
      self.beacons_seen.add("BP")
    if self.pb_beacon.seen:
      self.beacons_seen.add("PB")

    if self.total_time < self.time_for_turn_in_place:
      print("Turning in place")
      self.beacons_seen.clear()
      commands.setHeadPanTilt(0.0,self.tilt,self.time_for_turn_in_place)
      commands.setWalkVelocity(0.0,0.0,-0.4)
      
    elif self.total_time <= (3.0 + self.time_for_turn_in_place):
      #self.time_to_turn_head = self.time_to_turn_head + dt
      #print("self.pan: %f, total_time: %f"%(self.pan, self.total_time))
      commands.setHeadPanTilt(self.pan,self.tilt,3.0)
      commands.setWalkVelocity(0.0,0.0,0.0)
      
    elif self.total_time <= (9.0 + self.time_for_turn_in_place):
      #print("other self.pan: %f total_time: %f"%(self.pan, self.total_time))
      commands.setHeadPanTilt(-self.pan,self.tilt,6.0)
      commands.setWalkVelocity(0.0,0.0,0.0)
      
    elif self.total_time <= (self.time_for_panning + self.time_for_turn_in_place):
      #print("self.pan back to center")
      commands.setHeadPanTilt(0.0,self.tilt,3.0)
      commands.setWalkVelocity(0.0,0.0,0.0)
      
    elif (self.total_time < (self.time_to_walk + self.time_for_panning + self.time_for_turn_in_place)):
      print("Walking. Beacons seen in beginning stages: %d, total_time: %f"%(len(self.beacons_seen), self.total_time))
      commands.setHeadPanTilt(0.0, self.tilt, 1.0)
      
      if (len(self.beacons_seen) < self.count):
        self.time_for_turn_in_place = 1.5
        self.total_time = 0.0
      else:
        self.time_for_turn_in_place = 0.0
    
        
      #print("sum t: %f, dir: %f" %(self.time_to_turn_head, self.dir))
      
      distance = np.sqrt(np.power(self.robot.loc.x,2)+np.power(self.robot.loc.y,2))
      bearing = np.arctan2(self.robot.loc.y,self.robot.loc.x) - self.robot.orientation
      center = -np.sign(bearing)*(np.pi - np.abs(bearing))
      
      if (np.pi - abs(center)) < 0.1:
        center = np.sign(self.center_prev) * abs(center)
      
      self.calc_integral(dt)
      #print("Loc x: %f, Loc y: %f"%(self.robot.loc.x, self.robot.loc.y))
      
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
      
      if (distance > 660.0 and abs(center) > 0.5):
        self.frames_off_center += 1
        self.turned_toward_center = True
        self.turned_toward_front = False
        # Control only the heading of the robot and not the velocity
        # print("center > 0.3: Theta cont: %f"%(0.1*np.sign(center)))
        if (self.frames_off_center > 5):
          print("Turn toward center. d: %f, Loc x,y: %f, %f, center: %f"%(distance, self.robot.loc.x, self.robot.loc.y,center))
          commands.setWalkVelocity(0.0, 0.0, 0.5*np.sign(center))
      else:
        self.frames_off_center = 0
        
        # Control both heading and velocity
        if abs(distance) >= 500.0:# and self.turned_toward_center:
          print("d > 500.0, %f Loc x,y: %f, %f, center: %f, Theta cont: %f"%(distance, self.robot.loc.x, self.robot.loc.y,center,theta_cont))
          commands.setWalkVelocity(0.5, 0.0, 0.0)
        else:
          commands.setHeadPanTilt(0.0,0.0,1.5)
          #bearing_forward = np.arctan2(0.0-self.robot.loc.y,1500.0-self.robot.loc.x) - self.robot.orientation
          # distance = np.sqrt(np.power(self.robot.loc.x,2)+np.power(self.robot.loc.y,2))
          if (abs(self.robot.orientation) > 0.2):
            # don't pan while trying to go to center
            self.frames_away_from_front += 1
            self.turned_toward_front = True
            self.turned_toward_center = False

            if (self.frames_away_from_front > 5):
              print("Turn toward front. d: %f, Loc x,y: %f, %f, center: %f"%(distance, self.robot.loc.x, self.robot.loc.y,center))
              commands.setWalkVelocity(0.0,0.0, -0.5 * np.sign(self.robot.orientation))
              
          else:
            self.frames_away_from_front = 0
            commands.setWalkVelocity(-0.01 * self.robot.loc.x, -0.01 * self.robot.loc.y, 0.0)
            print("x cont: %f, y cont: %f"%(-0.01 * self.robot.loc.x, -0.007 * self.robot.loc.y))
              
          #if (not self.by_beacon.seen and not self.yb_beacon and abs(self.by_beacon.visionBearing) < 0.1):
          #  setWalkVelocity(0.0,0.0,np.sign(self.by_beacon.visionBearing))
          #if self.by_beacon.seen:
          #  
          #  commands.setWalkVelocity(x_cont)
          #print("d < 660.0, %f, Loc x,y: %f, %f, center: %f, Theta cont: %f, dist cont: %f"%(distance,self.robot.loc.x, self.robot.loc.y, center,theta_cont, dist_cont))
          #commands.setWalkVelocity(0.3, 0.0, 0.0)
          
      self.center_prev = center
      self.dist_prev = distance
      self.time_last = self.time_current
      
      if abs(distance - self.dist) < 25.0:
        self.frames_at_center += 1
        print("Thinks at center!!!!!")
        if (self.frames_at_center > 3):
          self.total_time = 0.0
          self.finish()
      else:
        self.frames_at_center = 0
      
    else:
      self.total_time = 0.0

    # Make sure to reset time
    self.time_last = self.time_current


class Stand(Node):
  def run(self):
    print("STAND????")
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
    goToCenter = GoToCenter(robot, 0.0, by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon)
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
    
    #self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight,C,goToCenter)
    #self.add_transition(goToCenter,T(3.0),moveHeadLeft)
    
    #self.add_transition(moveHeadLeft,C,moveHeadRight,C,goToCenter)


    self.add_transition(rdy,C,goToCenter)

    # self.add_transition(goToCenter,NBN(by_beacon,yb_beacon,yp_beacon,py_beacon,bp_beacon,pb_beacon),turnInPlace,C,moveHeadLeft)
    # self.add_transition(goToCenter,S("move"),moveHeadLeft)
    # self.add_transition(moveHeadLeft,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    # self.add_transition(moveHeadRight,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    # self.add_transition(turnInPlace,BN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),goToCenter)
    # self.add_transition(goToCenter,NBN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),moveHeadLeft)
    self.add_transition(goToCenter,C,stand1,T(10.0),goToCenter)#,T(10.0),stand2,NBN(by_beacon, yb_beacon, yp_beacon, py_beacon, bp_beacon, pb_beacon),moveHeadLeft)
