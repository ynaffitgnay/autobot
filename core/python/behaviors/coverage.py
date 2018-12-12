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
    memory.planning.RestartPath()

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
    self.k_x = (0.002, 0.005, 0.000)
    self.k_y = (0.002, 0.005, 0.000)
    self.k_t = (0.7, 0.01, 0.1)
    # self.k_t = (0.008, 0.01, 0.2)
    self.x_int = 0.0
    self.y_int = 0.0
    self.t_int = 0.0
    self.x_prev = 0.0
    self.y_prev = 0.0
    self.t_prev = 0.0
    self.id_prev = self.pathidx
    self.tkm1 = time.clock()
    self.tk = time.clock()

  def calc_int(self, e_x, e_y, e_t, dt):
    self.x_int = self.x_int + dt*e_x
    self.y_int = self.y_int + dt*e_y
    self.t_int = self.t_int + dt*e_t
    if abs(self.x_int) >= 35.0:
      self.x_int = 35.0*np.sign(self.x_int)
    if abs(self.y_int) >= 35.0:
      self.y_int = 35.0*np.sign(self.y_int)
    if abs(self.t_int) >= 15.0:
      self.t_int = 15.0*np.sign(self.t_int)

  def run(self):
    commands.setHeadPanTilt(0.0,0.0,1.5)

    # The desired location on the field
    self.destloc = memory.planning.getDestPose()
    self.pathidx = memory.planning.pathIdx
    th = self.robot.orientation

    # The desired x, y, t
    x_des = self.destloc.translation.x
    y_des = self.destloc.translation.y
    t_des = self.destloc.rotation
    print("[pathIdx: %f, robot x: %f, robot y: %f, robot orientation: %f]" % (self.pathidx, self.robot.loc.x, self.robot.loc.y, core.RAD_T_DEG * self.robot.orientation))
    print("destloc: [%f,%f,%f]" % (self.destloc.translation.x, self.destloc.translation.y, core.RAD_T_DEG * self.destloc.rotation))

    # The error in x, y, t
    e_x = -(self.robot.loc.x - x_des)
    e_y = -(self.robot.loc.y - y_des)
    e_t = t_des - self.robot.orientation
    if np.sign(t_des)*e_t - np.pi > 0:
      e_t = e_t - np.sign(t_des) * 2 * np.pi
    # if t_des > 0:
    #   if self.robot.orientation > t_des - np.sign(t_des)*np.pi:
    #     e_t = t_des - self.robot.orientation
    #   else:
    #     e_t = t_des - (2*np.pi + self.robot.orientation)
    # else:
    #   if self.robot.orientation > t_des - np.sign(t_des)*np.pi:
    #     e_t = t_des - (self.robot.orientation -2*np.pi)
    #   else:
    #     e_t = t_des - self.robot.orientation

    self.tk = time.clock()
    dt = self.tk - self.tkm1
    self.calc_int(x_des, y_des, t_des, dt)

    de_x = (e_x - self.x_prev)
    de_y = (e_y - self.y_prev)
    de_t = (e_t - self.t_prev)

    # Bound the derivative term of x
    if dt == 0 or (self.k_x[2] * de_x > 0.1 * dt):
      de_x = 0.0
    else:
      de_x = de_x / dt

    # Bound the derivative term of y
    if dt == 0 or (self.k_y[2] * de_y > 0.1 * dt):
      de_y = 0.0
    else:
      de_y = de_y / dt

    # Bound the derivative term of theta
    if dt == 0 or (self.k_t[2] * de_t > 0.1 * dt):
      de_t = 0.0
    else:
      de_t = de_t / dt

    x_cont = self.k_x[0] * e_x + self.k_x[1] * self.x_int + self.k_x[2] * de_x
    # print("[x_cont: %f, ex: %f, kpx: %f, kix: %f, kdx: %f]" % (x_cont, e_x, self.k_x[0] * e_x, self.k_x[1] * self.x_int, self.k_x[2] * de_x))
    y_cont = self.k_y[0] * e_y + self.k_y[1] * self.y_int + self.k_y[2] * de_y
    # print("[y_cont: %f, ey: %f, kpy: %f, kiy: %f, kdy: %f]" % (y_cont, e_y, self.k_y[0] * e_y, self.k_y[1] * self.y_int, self.k_y[2] * de_y))
    t_cont = self.k_t[0] * e_t + self.k_t[1] * self.t_int + self.k_t[2] * de_t
    # print("[t_cont: %f, et: %f, kpt: %f, kit: %f, kdt: %f]" % (t_cont, e_t, self.k_t[0] * e_t, self.k_t[1] * self.t_int, self.k_t[2] * de_t))

    K = np.array([[np.cos(th), np.sin(th), 0],[-np.sin(th), np.cos(th), 0],[0, 0, 1]])
    dstate = np.array([[x_cont], [y_cont], [t_cont]])

    u = np.dot(K,dstate)
    # print("[u_1: %f, u_2: %f, u_3: %f]" % (u[0], u[1], u[2]))
    commands.setWalkVelocity(u[0,0],u[1,0],u[2,0])

    self.x_prev = e_x
    self.y_prev = e_y
    self.t_prev = e_t
    self.tkm1 = self.tk
    if not (self.id_prev == self.pathidx):
      self.postSignal("head")

    self.id_prev = self.pathidx
    if memory.planning.nodesLeft == 0:
      self.finish()

class FaceNextCell(Node):
  """Face the next cell to check for obstacles"""
  def __init__(self, robot, faced):
    super(FaceNextCell, self).__init__()
    self.robot = robot
    # self.faced = memory.planning.observedNextGC
    self.destloc = memory.planning.getDestPose()
    self.pathidx = memory.planning.pathIdx
    self.k_t = (0.7, 0.01, 0.1)
    self.t_int = 0.0
    self.t_prev = 0.0
    self.id_prev = self.pathidx
    self.tkm1 = time.clock()
    self.tk = time.clock()

  def calc_int(self, e_t, dt):
    self.t_int = self.t_int + dt*e_t
    if abs(self.t_int) >= 15.0:
      self.t_int = 15.0*np.sign(self.t_int)

  def run(self):
    # The desired location on the field
    
    self.destloc = memory.planning.getDestPose()
    self.tk = time.clock()
    dt = self.tk - self.tkm1
    x_des = self.destloc.translation.x
    y_des = self.destloc.translation.y
    t_des = np.arctan2(y_des - self.robot.loc.y, x_des - self.robot.loc.x)
    e_t = t_des - self.robot.orientation
    if np.sign(t_des)*e_t - np.pi > 0:
      e_t = e_t - np.sign(t_des) * 2 * np.pi
    self.calc_int(e_t, dt)
    commands.setHeadPanTilt(e_t, 0.0, 1.5)

    de_t = (e_t - self.t_prev)
    print("[pathIdx: %f, robot x: %f, robot y: %f, robot orientation: %f]" % (memory.planning.pathIdx, self.robot.loc.x, self.robot.loc.y, core.RAD_T_DEG * self.robot.orientation))
    print("destloc: [%f,%f,%f], e_t: %f" % (self.destloc.translation.x, self.destloc.translation.y, core.RAD_T_DEG * self.destloc.rotation, e_t))
    
    # Bound the derivative term of theta
    if dt == 0:
      de_t = 0.0
    else:
      de_t = de_t / dt

    t_cont = self.k_t[0] * e_t + self.k_t[1] * self.t_int + self.k_t[2] *de_t
    # print("t_cont = %f" % (t_cont))

    if abs(e_t) >=0.3:
      commands.setWalkVelocity(0.0, 0.0, 0.4*np.sign(e_t))
    else:
      commands.setWalkVelocity(0.0, 0.0, t_cont)

    self.t_prev = e_t
    self.tkm1 = self.tk
    if (abs(e_t) < 0.1):
      memory.planning.observedNextGC = True
      self.finish()

class Stand(Node):
  def run(self):
    # print("STAND????")
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)

class MoveHead(Node):
  """Search for the ball to the left"""
  def __init__(self, pan, tilt, time):
    super(MoveHead, self).__init__()
    self.tilt = tilt
    self.pan = pan
    self.time = time

  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(core.DEG_T_RAD*self.pan,self.tilt, self.time)
    if self.getTime() > (self.time + 0.5):
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    robot = memory.world_objects.getObjPtr(core.WO_TEAM5)
    rdy = GetReady()
    stand = Stand()
    faced = False
    moveHeadLeft = MoveHead(110.0, 0.0, 3.0)
    moveHeadRight = MoveHead(-110.0, 0.0, 6.0)

    follow = FollowPath(robot)

    faceNextCell = FaceNextCell(robot, faced)

    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight,C,follow,S("head"),faceNextCell,C,moveHeadLeft)
