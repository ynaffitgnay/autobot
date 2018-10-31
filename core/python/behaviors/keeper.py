"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import geometry
import commands
import mem_objects
import task, util
import time
import head
import math
import random
import numpy as np
from task import Task, MultiTask
import cfgpose, cfgstiff
import pose
#from pose import ToPoseMoveHead
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles, world_objects
from state_machine import Node, S, C, T, LoopingStateMachine, Event
import UTdebug


class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")

    return #pose.BlockLeft()

class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
  
    return #pose.BlockRight()

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking center")
    
    return pose.Squat(time=3.0)

class Reset(Node):
  """Go back to normal"""
  def run(self):
    UTdebug.log(15,"Resetting keeper")

    return #pose.Sit()


class DontBlock(Node):
  """Ball missed dont block"""
  def run(self):
    UTdebug.log(15, "Not Blocking")

    ball = mem_objects.world_objects[core.WO_BALL]
    commands.setStiffness(cfgstiff.OneArmsOnly)
    commands.setHeadPan(ball.bearing, 0.2)
    newPose = dict()
    newPose[core.LShoulderRoll] = 15
    newPose[core.LShoulderPitch] = -100
    newPose[core.LElbowRoll] = 0
    newPose[core.RShoulderRoll] = 15
    newPose[core.RShoulderPitch] = -100
    newPose[core.RElbowRoll] = 0
    newPose[core.RHipPitch] = -46.5
    newPose[core.LHipPitch] = -46.5
    return #pose.ToPoseMoveHead(pose=newPose, tilt=-15, pan = ball.bearing, time=0.5)

class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)
    if self.getTime() > 2.0:
      commands.setWalkVelocity(0.0,0.0,0.0)
      self.finish()

class MoveHead(Node):
  """General Move Head node"""
  def __init__(self, pan, tilt, time):
    super(MoveHead, self).__init__()
    self.pan = pan
    self.tilt = tilt
    self.time = time
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(core.DEG_T_RAD*self.pan,self.tilt,self.time)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > (self.time + 0.3):
      self.finish()

class CheckIfLocalized(Node):
  def __init__(self, localized,poseListX,poseListY, poseListTh,pose_index):
    super(CheckIfLocalized, self).__init__()
    self.localized = localized
    self.poseListX = poseListX
    self.poseListY = poseListY
    self.poseListTh = poseListTh
    self.pose_index = pose_index
    self.initialized = False
  def run(self):

    commands.setHeadPanTilt(0.0,-10.0,1.0)
    robot = world_objects.getObjPtr(core.WO_TEAM5)
    self.checkLocalized(robot)
    if self.localized:
      print("Localized")
      self.postSignal("localized")
      return
    else:
      print("Lost")
      self.postSignal("lost")
      return

  def checkLocalized(self,robot):
    xGlobal = robot.loc.x
    yGlobal = robot.loc.y
    thGlobal = robot.orientation
    print("Robot checking localization. Current pose: [%f, %f] Theta: %f\n" % (xGlobal,yGlobal,thGlobal*core.RAD_T_DEG))
    if not self.initialized:
        self.startLocalize(robot.loc.x,robot.loc.y,robot.orientation)
        return
    else:
      if not thGlobal < -np.pi/2.0 and not thGlobal > np.pi/2.0:
        #facing toward the goal, not localized
        self.localized = False
        print("4")
        return
      else:
        # facing correct general direction
        if xGlobal < 1000.0:
          # outside the top of the goalbox, not localized
          self.localized = False
          print("5")
          return
        else:
          if yGlobal > 700.0 or yGlobal < -700.0:
            # outside the left of right of the goalbox, not localized
            self.localized = False
            print("6")
            return
      # within bounds, close enough to localized
      self.localized = True
    return

  def startLocalize(self, x, y, th):
    window_size = 10
    if len(self.poseListTh) <= window_size:
      self.localized = False
      return
    else:
      self.poseListTh[self.pose_index % window_size] = Th
      self.pose_index = self.pose_index + 1

    th_sum = 0.0
    for th in range(len(self.poseListTh)):
        th_sum = th_sum + th

    th_avg = th_sum/10.0
    if (np.pi - abs(th_avg)) <= 0.1:
        self.localized = True
        self.initialized = True

    # x_stdev_sum = 0.0
    # y_stdev_sum = 0.0
    # for x in self.poseListX:
    #     x_stdev_sum = x_stdev_sum + abs(x-x_avg)
    # for y in self.poseListY:
    #     y_stdev_sum = y_stdev_sum + abs(y-y_avg)
    # x_stdev_avg = x_stdev_sum/window_size
    # y_stdev_avg = y_stdev_sum/window_size

    # print("Avg x stdev: %f Avg y stdev: %f\n" % (x_stdev_avg,y_stdev_avg))
    
    return 

class Blocker(Node):

  def run(self):
    # if not ball.seen, move the head back to the center
    ball = mem_objects.world_objects[core.WO_BALL]
    if not ball.seen:
      commands.setHeadPan(0.0, 0.2)
    else:
      commands.setHeadPan(ball.bearing, 0.2)
    # print("Ball distance: %f Ball bearing: %f\n" % (ball.distance,ball.bearing))
    v_mag = math.sqrt(ball.relVel.x*ball.relVel.x + ball.relVel.y*ball.relVel.y)
    x_ball = ball.distance*math.cos(ball.bearing)/10.0
    y_ball = ball.distance*math.sin(ball.bearing)/10.0
    delta_t = 2.0
    v_thresh = -(x_ball / delta_t)
    # print("X: %f, Y: %f, VThresh: %f, VMag: %f, Vx: %f, Vy: %f" % (x_ball, y_ball, v_thresh, v_mag, ball.relVel.x, ball.relVel.y))
    if ball.relVel.x < v_thresh and ball.seen and ball.relVel.x < 0.0:
      # ball moving away from the robot, reset to regular position
      print("Current Vx: %f, Current Vy: %f, x ball: %f, y ball: %f" % (ball.relVel.x, ball.relVel.y, x_ball, y_ball))
      y_intercept = -ball.relVel.y * x_ball /ball.relVel.x + y_ball
      print("Y intercept: %f" % y_intercept)
      ## can do if xvel < yvel && xvel < 1, etc.
      ## maybe look at distance between robot and goal? 
      UTdebug.log(15, "Ball is close, blocking!")
      if y_intercept > 42.0 or y_intercept < -40.0 or not ball.seen:
        choice = "miss"
      elif y_intercept > 13.5:
        choice = "left"
      elif y_intercept < -13.5:
        choice = "right"
      elif y_intercept <= 13.5 and y_intercept >= -13.5:
        choice = "center"
      self.postSignal(choice)
      return
    elif ball.seen:
      choice = "moveBall"
      # print("Move ball signal")
      self.postSignal(choice)
      return
    else:  
      return
    

class MoveBtwBall(Node):
  def __init__(self,localized,poseListX,poseListY, poseListTh,pose_index):
    super(MoveBtwBall, self).__init__()
    self.goalieStartX = 0.0
    self.goalieStartY = 0.0
    self.localized = localized
    self.initialized = False
    self.radius = 300.0
    self.roboDesiredX = 0.0
    self.roboDesiredY = 0.0
    self.roboDesiredTh = 0.0
    self.localized = localized
    self.poseListX = poseListX
    self.poseListY = poseListY
    self.pose_index = pose_index
    self.k_x = (0.00005, 0.0001, 0.0001)
    self.k_y = (0.000005, 0.0001, 0.0001)
    self.k_t = (0.7, 0.01, 0.1)
    
    # integrals and previouse values for the PID controllers
    self.x_int = 0.0
    self.x_prev = 0.0
    self.y_int = 0.0
    self.y_prev = 0.0
    self.theta_int = 0.0
    self.theta_prev = 0.0
    
    # Time for PID control
    self.time_last = time.clock()
    self.time_current = time.clock()

    # Objects
    self.gb_line_obj = mem_objects.world_objects[core.WO_OWN_PENALTY]
    self.gb_line_seg = None
    self.robot = world_objects.getObjPtr(core.WO_TEAM5)
    self.ball = mem_objects.world_objects[core.WO_BALL]

  def run(self):
    self.checkLocalized()

    if self.localized:
      if not self.initialized:
        self.goalieStartX = self.robot.loc.x
        self.goalieStartY = self.robot.loc.y
        self.goalieStartTh = self.robot.orientation
        self.initialized = True

      self.getDesiredGoaliePos()

      self.goToDesiredPos()
      



      # Look at the ball
      if self.ball.seen:
        commands.setHeadPan(self.ball.bearing, 0.2)
      # print("Robot pose: [%f,%f,%f]\n" %(robot.loc.x,robot.loc.y,robot.orientation*core.RAD_T_DEG))
      
    if self.getTime() > 2.0:
      if self.localized:
        signal = "localized"
      else:
        signal = "lost"
      self.postSignal(signal)

  def checkLocalized(self):
    xGlobal = self.robot.loc.x
    yGlobal = self.robot.loc.y
    thGlobal = self.robot.orientation
    # print("Robot checking localization before movement. Current pose: [%f, %f] Theta: %f\n" % (xGlobal,yGlobal,thGlobal*core.RAD_T_DEG))
    self.getSlidingCov(self.robot.loc.x,self.robot.loc.y)
    if not thGlobal < -np.pi/2.0 and not thGlobal > np.pi/2.0:
      #facing toward the goal, not localized
      # print("1")
      self.localized = False
      return
    else:
      # facing correct general direction
      if xGlobal < 1000.0:
        # outside the top of the goalbox, not localized
        self.localized = False
        # print("2")
        return
      else:
        if yGlobal > 700.0 or yGlobal < -700.0:
          # outside the left of right of the goalbox, not localized
          self.localized = False
          # print("3")
          return
    # within bounds, close enough to localized
    self.localized = True
    return

  def getSlidingCov(self, x, y):
    window_size = 10
    if len(self.poseListX) <= window_size:
      self.poseListX.append(x)
      self.poseListY.append(y)
    else:
      self.poseListX[self.pose_index % window_size] = x
      self.poseListY[self.pose_index % window_size] = y
      self.pose_index = self.pose_index + 1

    x_sum = 0.0
    y_sum = 0.0
    for x in self.poseListX:
        x_sum = x_sum + x
    for y in self.poseListY:
        y_sum = y_sum + y

    x_avg = x_sum/10.0
    y_avg = y_sum/10.0

    x_stdev_sum = 0.0
    y_stdev_sum = 0.0
    for x in self.poseListX:
        x_stdev_sum = x_stdev_sum + abs(x-x_avg)
    for y in self.poseListY:
        y_stdev_sum = y_stdev_sum + abs(y-y_avg)
    x_stdev_avg = x_stdev_sum/window_size
    y_stdev_avg = y_stdev_sum/window_size

    # print("Avg x stdev: %f Avg y stdev: %f\n" % (x_stdev_avg,y_stdev_avg))
    
    return 

  def calc_integral(self, dt, x, y, theta):
    self.x_int = self.x_int + dt*(self.robot.loc.x - x)
    self.y_int = self.y_int + dt*(y - self.robot.loc.y)
    self.theta_int = self.theta_int + dt*(self.ball.bearing)
    if abs(self.x_int) >= 100.0:
      self.x_int = 100.0*np.sign(self.x_int)
    if abs(self.y_int) >= 100.0:
      self.y_int = 100.0*np.sign(self.y_int)
    if abs(self.theta_int) >= 15.0:
      self.theta_int = 15.0*np.sign(self.theta_int)

  def goToDesiredPos(self):
    self.time_current = time.clock()
    globX = self.robot.loc.x
    globY = self.robot.loc.y
    globTh = self.robot.orientation
    dt = self.time_current - self.time_last
    x = self.roboDesiredX
    y = self.roboDesiredY
    theta = self.roboDesiredTh
    bearing = self.ball.bearing
    self.calc_integral(dt, x, y, theta)
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    commands.setHeadPanTilt(self.ball.bearing, -elevation, 1.5)
    if dt == 0:
      x_cont = self.k_x[0] * (globX - x) + self.k_x[1] * self.x_int
      y_cont = self.k_y[0] * (y - globY) + self.k_y[1] * self.y_int
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_int
    else:
      x_cont = self.k_x[0] * (globX - x) + self.k_x[1] * self.x_int + self.k_x[2] *(globX - self.x_prev) / dt
      y_cont = self.k_y[0] * (y - globY) + self.k_y[1] * self.y_int + self.k_y[2] *(self.y_prev - globY) / dt
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_int + self.k_t[2] *(bearing - self.theta_prev) / dt
    
    if abs(bearing) >=0.3:
      # Control only the heading of the robot and not the velocity
      commands.setWalkVelocity(0.0, 0.0, 0.4*np.sign(bearing))
    else:
      # Control both heading and velocity
      if self.gb_line_obj.seen:
        self.gb_line_seg = self.gb_line_obj.lineLoc
        gb_line_cent = self.gb_line_seg.getCenter()
        rel_robot = core.Point2D(0.0,0.0)
        gb_dist_thresh = 200.0
        gb_robo_dist = self.gb_line_seg.getDistanceTo(rel_robot)
        # print("Line seen. Center at [%f, %f], robot at [%f, %f] dist to closest point is: %f" % (gb_line_cent.x,gb_line_cent.y,self.robot.loc.x,self.robot.loc.y,gb_robo_dist))
        if gb_robo_dist < gb_dist_thresh:
          # Too close in either x or y
          x_cont = 0.0
      # else:
        # print("Line not seen.\n")
      commands.setWalkVelocity(x_cont, y_cont, theta_cont)
      print("Controls: [%f, %f, %f]\n" %(x_cont, y_cont, theta_cont))
    self.x_prev = globX - self.x_prev
    self.y_prev = self.y_prev - globY
    self.theta_prev = bearing
    self.time_last = self.time_current

    return

  def getDesiredGoaliePos(self):

    goalBallXComp = self.ball.loc.x - self.goalieStartX 
    goalBallYComp = self.ball.loc.y - self.goalieStartY
    goalBallTh = np.arctan2(goalBallYComp,-goalBallXComp)
    print("Ball global pose: [%f, %f]" % (self.ball.loc.x,self.ball.loc.y))
    print("Ball to goal bearing: %f" % (goalBallTh*core.RAD_T_DEG))
    self.roboDesiredX = self.goalieStartX - self.radius*np.cos(goalBallTh)
    self.roboDesiredY = self.goalieStartY + self.radius*np.sin(goalBallTh)
    if goalBallTh > 0:
      self.roboDesiredTh = np.pi - goalBallTh
    else:
      self.roboDesiredTh = -np.pi - goalBallTh

    print("Robot should go to [%f, %f] with bearing: %f Robot at [%f,%f] with bearing: %f" %(self.roboDesiredX,self.roboDesiredY,self.roboDesiredTh*core.RAD_T_DEG,self.robot.loc.x,self.robot.loc.y,self.robot.orientation))
    return


class Playing(LoopingStateMachine):
  def setup(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    robot = world_objects.getObjPtr(core.WO_TEAM5)
    
    localized = False
    poseListX = []
    poseListY = []
    poseListTh = []
    pose_index = 0 
    blocker = Blocker()
    lookStraight = MoveHead(0.0,-10.0,1.5)
    moveHeadLeft = MoveHead(85.0,-10.0,1.5)
    moveHeadRight = MoveHead(-85.0,-10.0,3.0)
    reset = Reset()
    rdy = GetReady()
    moveBtwBall = MoveBtwBall(localized,poseListX,poseListY, poseListTh,pose_index)
    checkLoc = CheckIfLocalized(localized,poseListX,poseListY, poseListTh,pose_index)
    blocks = {"left": BlockLeft(),
              "right": BlockRight(),
              "center": BlockCenter(),
              "miss": DontBlock()
              }
    locState = {"lost": moveHeadLeft,"localized": blocker}
    self.add_transition(rdy,C,checkLoc)
    for state in locState:
      s = locState[state]
      self.add_transition(checkLoc, S(state), s)
      self.add_transition(moveBtwBall, S(state),s)
    for name in blocks:
      b = blocks[name]
      self.add_transition(blocker, S(name), b, T(4.0), reset, T(3.0), blocker)
    self.add_transition(blocker, S("moveBall"), moveBtwBall)
    self.add_transition(moveHeadLeft,C,moveHeadRight,C,checkLoc)
