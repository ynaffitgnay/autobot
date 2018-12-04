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

class GetReady(Node):
  def __init__(self):
    super(GetReady, self).__init__()
    memory.planning.coverageStarted = False
    memory.planning.planningIdx = 0
    memory.planning.nodesLeft = memory.planning.nodesInPath

  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)
    if self.getTime() > 4.5:
      location = memory.planning.getDestPose()
      print("[%f,%f,%f]" % (location.translation.x, location.translation.y, location.rotation))
      self.finish()

class FollowPath(Node):
  """Face the ball"""
  def __init__(self, robot):
    super(FollowPath, self).__init__()
    memory.planning.coverageStarted = True
    self.destloc = memory.planning.getDestPose()
    self.pathidx = memory.planning.pathIdx
    self.robot = robot
    self.k_t = (0.07, 0.01, 0.1)
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()
    self.des_theta = np.pi
    self.prev_idx = self.pathidx

  def calc_integral(self, dt):
    center = np.sign(self.robot.orientation)*(np.pi - np.abs(self.robot.orientation))
    self.theta_integral = self.theta_integral + dt*center
    if abs(self.theta_integral) >= 15.0:
      self.theta_integral = 15.0*np.sign(self.theta_integral)

  def run(self):
    commands.setHeadPanTilt(0.0,0.0,1.5)
    self.destloc = memory.planning.getDestPose()
    self.pathidx = memory.planning.pathIdx
    if not (self.prev_idx == self.pathidx):
      self.postSignal("head")
    center = np.sign(self.robot.orientation)*(np.pi - np.abs(self.robot.orientation))
    if memory.planning.nodesLeft == 0:
      self.finish()
    self.time_current = time.clock()
    dt = self.time_current - self.time_last
    self.calc_integral(dt)

    if dt == 0 or (self.k_t[2] * (center - self.theta_prev) / dt) > 0.3:
      theta_cont = self.k_t[0] * center + self.k_t[1] * self.theta_integral
    else:
      theta_cont = self.k_t[0] * center + self.k_t[1] * self.theta_integral + self.k_t[2] *(center - self.theta_prev) / dt

    if abs(theta_cont) > 0.2:
      theta_cont = np.sign(theta_cont)*0.2
    print("[P input: %f, I input: %f, D input: %f]" % (self.k_t[0] * center, self.k_t[1] * self.theta_integral, self.k_t[2] *(center - self.theta_prev)/dt))
    print("[pathIdx: %f, robot x: %f, robot y: %f, robot orientation: %f]" % (self.pathidx, self.robot.loc.x, self.robot.loc.y, core.RAD_T_DEG * self.robot.orientation))
    print("destloc: [%f,%f,%f]" % (self.destloc.translation.x, self.destloc.translation.y, self.destloc.rotation))

    x_disp = self.robot.loc.x - self.destloc.translation.x
    y_disp = self.robot.loc.y - self.destloc.translation.y

    if abs(y_disp) > abs(x_disp):
      commands.setWalkVelocity(0.05,0.4*np.sign(y_disp),-0.05*np.sign(y_disp))
    else:
      commands.setWalkVelocity(0.4*np.sign(x_disp),-0.05*np.sign(x_disp),theta_cont)

    self.curloc = self.destloc
    self.destloc = memory.planning.getDestPose()
    self.theta_prev = center
    self.time_last = self.time_current
    self.prev_idx = self.pathidx


class Stand(Node):
  def run(self):
    # print("STAND????")
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)

class MoveHeadLeft(Node):
  """Search for the ball to the left"""
  def __init__(self, tilt):
    super(MoveHeadLeft, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(core.DEG_T_RAD*85.0,self.tilt,2.0)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 2.5:
      self.finish()

class MoveHeadRight(Node):
  """Search for the ball to the right"""
  def __init__(self, tilt):
    super(MoveHeadRight, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(-core.DEG_T_RAD*85.0,self.tilt,4.0)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 5.0:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    robot = memory.world_objects.getObjPtr(core.WO_TEAM5)
    rdy = GetReady()
    stand = Stand()
    moveHeadLeft = MoveHeadLeft(0.0)
    moveHeadRight = MoveHeadRight(0.0)

    follow = FollowPath(robot)

    self.add_transition(rdy,C,follow,S("head"),moveHeadLeft,C,moveHeadRight,C,follow)
