"""God tier brazilian Ronaldo level goal scoring behavior"""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import mem_objects
import time
import numpy as np
import math
import core
import pose
import head
import commands
import geometry
import cfgpose, cfgstiff
import random
import task, util
from task import Task, MultiTask
from state_machine import Node, C, T, S, LoopingStateMachine, StateMachine, EventNode, Event, NegationEvent
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles, world_objects, odometry
import UTdebug

class ObjectSeen(Event):
  """Event that fires when all the objects passed to this event are seen in the current frame"""
  def __init__(self,*objects):
    super(ObjectSeen, self).__init__()
    self.objects = objects
  def ready(self):
    seen = True
    for obj in self.objects:
      seen = obj.seen and seen
    return seen

def O(*objects):
  """Objects found"""
  return ObjectSeen(*objects)

def NO(*objects):
  """Not true that all objects found"""
  return NegationEvent(ObjectSeen(*objects))

class ObjectOrSeen(Event):
  """Event that fires when all the objects passed to this event are seen in the current frame"""
  def __init__(self,*objects):
    super(ObjectOrSeen, self).__init__()
    self.objects = objects
  def ready(self):
    seen = False
    for obj in self.objects:
      seen = obj.seen or seen
    return seen

def OR(*objects):
  """Objects found"""
  return ObjectOrSeen(*objects)

def NOR(*objects):
  """Not true that all objects found"""
  return NegationEvent(ObjectOrSeen(*objects))

class LineDistance(Event):
  """Event that fires if Ball is close enough"""
  def __init__(self, obj, dist):
    super(LineDistance, self).__init__()
    self.obj = obj
    self.dist = dist
  def ready(self):
    rel_robot = core.Point2D(0.0,0.0)
    distance = self.obj.lineLoc.getDistanceTo(rel_robot)
    print("LineDistance: %f" %(distance))
    return (self.obj.seen and (distance < self.dist))

def LD(obj, dist = 100.0):
  """Ball close enough"""
  return LineDistance(obj, dist)


class ObjectDistance(Event):
  """Event that fires if Ball is close enough"""
  def __init__(self, obj, dist):
    super(ObjectDistance, self).__init__()
    self.obj = obj
    self.dist = dist
  def ready(self):
    return (self.obj.seen and (self.obj.distance < self.dist))

def OD(obj, dist = 50.0):
  """Ball close enough"""
  return ObjectDistance(obj, dist)

class BallBearing(Event):
  """Event that fires if Ball bearing is close enough"""
  def __init__(self, ball, bearing):
    super(BallBearing, self).__init__()
    self.ball = ball
    self.bearing = bearing
  def ready(self):
    return abs(self.ball.bearing - self.bearing) < 0.1

def BB(ball, bearing = 0.28):
  """Ball close enough"""
  return BallBearing(ball, bearing)

class Aligned(Event):
  """Aligning ball, goal and robot"""
  def __init__(self, ball, goal, tolerance_ball, tolerance_goal):
    super(Aligned, self).__init__()
    self.ball = ball
    self.goal = goal
    self.tolerance_ball = tolerance_ball
    self.tolerance_goal = tolerance_goal

  def ready(self):
    return (abs(self.ball.bearing) < self.tolerance_ball and abs(self.goal.visionBearing) < self.tolerance_goal)

def A(ball, goal, tolerance_ball = 0.2, tolerance_goal = 0.3):
  """Ball, Goal, Robot aligned"""
  return Aligned(ball, goal, tolerance_ball, tolerance_goal)

class AlignedWithBearing(Event):
  """Aligning ball, goal and robot"""
  def __init__(self, ball, goal, tolerance, des_bearing):
    super(AlignedWithBearing, self).__init__()
    self.ball = ball
    self.goal = goal
    self.tolerance = tolerance
    self.des_bearing = des_bearing

  def ready(self):
    return (abs(self.goal.visionBearing - self.ball.bearing) < self.tolerance)

def AB(ball, goal, tolerance = 0.3, des_bearing = 0.5):
  """Ball, Goal, Robot aligned"""
  return AlignedWithBearing(ball, goal, tolerance, des_bearing)

class CheckDribble(Event):
  """Check whether or not robot should Dribble"""
  def __init__(self, ball, goal, dist):
    super(CheckDribble, self).__init__()
    self.ball = ball
    self.goal = goal
    self.dist = dist

  def ready(self):
    # print("goal dist: %f, ball dist: %f, goal dist - ball dist: %f" % (self.ball.distance,self.goal.visionDistance,abs(self.ball.distance - self.goal.visionDistance)))
    # print("ball vision dist: %f, goal dist - ball vision dist: %f" % (self.ball.visionDistance,abs(self.ball.visionDistance - self.goal.visionDistance)))
    # return np.sqrt(np.power(self.ball.loc.x-self.goal.loc.x,2) + np.power(self.ball.loc.y-self.goal.loc.y,2)) < self.dist
    return abs(self.ball.distance - self.goal.visionDistance) < self.dist
    #   self.frames_at_dist += 1
    # else:
    #   self.frames_at_dist = 0
    # return self.frames_at_dist >= 5

def D(ball, goal, dist = 1200.0):
  """Ball, Goal, Robot aligned"""
  return CheckDribble(ball, goal, dist)

class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")

    return pose.BlockLeft()

class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
 
    return pose.BlockRight()

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking center")
   
    return pose.Squat2()

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
      #if not thGlobal < -np.pi/2.0 and not thGlobal > np.pi/2.0:
      #  #facing toward the goal, not localized
      #  self.localized = False
      #  print("4")
      #  return
      #else:
      # facing correct general direction
      if xGlobal < 1000.0:
        # outside the top of the goalbox, not localized
        self.localized = False
        #print("5")
        return
      else:
        if yGlobal > 700.0 or yGlobal < -700.0:
          # outside the left of right of the goalbox, not localized
          self.localized = False
          #print("6")
          return
      # within bounds, close enough to localized
      self.localized = True
    return

  def startLocalize(self, x, y, theta):
    window_size = 3
    print("Theta: %f" % (theta*core.RAD_T_DEG))
    if len(self.poseListTh) < window_size:
      self.localized = False
      self.poseListTh.append(theta)
      print("Pose list Theta size: %d" % len(self.poseListTh))
      return
    else:
      self.poseListTh[self.pose_index % window_size] = theta
      self.pose_index = self.pose_index + 1

    th_sum = 0.0
    for th in self.poseListTh:
      if th < 0:
        th = th + 2*np.pi
      th_sum = th_sum + th
    print("Theta sum: %f" % (th_sum))

    th_avg = th_sum/window_size
    print("Theta avg before coordinate shift: %f" % (th_avg * core.RAD_T_DEG))
    if th_avg > np.pi:
      th_avg = th_avg - 2*np.pi
    if (np.pi - abs(th_avg)) <= 0.5:
        self.localized = True
        self.initialized = True
    print("Theta avg: %f" % (th_avg*core.RAD_T_DEG))

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
  def __init__(self):
    super(Blocker, self).__init__()
    self.prevTime = self.getTime()

  def run(self):
    dt = self.getTime() - self.prevTime
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
    print("V Thresh: %f" % (v_thresh))
    disp = odometry.displacement
    if not dt == 0:
      print("Disp: [%f, %f, %f]" % (0.1*disp.translation.x/dt, 0.1*disp.translation.y/dt, 0.1*disp.rotation/dt))
    # print("X: %f, Y: %f, VThresh: %f, VMag: %f, Vx: %f, Vy: %f" % (x_ball, y_ball, v_thresh, v_mag, ball.relVel.x, ball.relVel.y))
    self.prevTime = self.getTime()
    if ball.relVel.x < v_thresh and ball.seen and ball.relVel.x < 0.0 :
      # ball moving away from the robot, reset to regular position
      print("Current Vx: %f, Current Vy: %f, x ball: %f, y ball: %f" % (ball.relVel.x, ball.relVel.y, x_ball, y_ball))
      y_intercept = -ball.relVel.y * x_ball /ball.relVel.x + y_ball
      print("Y intercept: %f" % y_intercept)
      ## can do if xvel < yvel && xvel < 1, etc.
      ## maybe look at distance between robot and goal?
      UTdebug.log(15, "Ball is close, blocking!")
      if y_intercept > 45.0 or y_intercept < -45.0 or not ball.seen:
        choice = "miss"
      elif y_intercept > 18.5:
        choice = "left"
      elif y_intercept < -18.5:
        choice = "right"
      elif y_intercept <= 18.5 and y_intercept >= -18.5:
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
    self.radius = 500.0
    self.roboDesiredX = 0.0
    self.roboDesiredY = 0.0
    self.roboDesiredTh = 0.0
    self.localized = localized
    self.poseListX = poseListX
    self.poseListY = poseListY
    self.pose_index = pose_index
    self.k_x = (0.004, 0.0, 0.0)
    self.k_y = (0.003, 0.0, 0.0)
    self.k_t = (0.7, 0.01, 0.1)
    self.dir = 1.0
   
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
    self.time_current = time.clock()
    dt = self.time_current - self.time_last

    if (dt > 1.0):
      self.dir = self.dir * (-1.0)
      dt = 0.0

    self.checkLocalized()

    if self.localized:
      if not self.initialized:
        self.goalieStartX = 1800.0 #self.robot.loc.x
        self.goalieStartY = 0.0 #self.robot.loc.y
        self.goalieStartTh = self.robot.orientation
        self.initialized = True

      self.getDesiredGoaliePos()

      self.goToDesiredPos()
     
      # Look at the ball
      if self.ball.seen:
          commands.setHeadPan(self.ball.bearing, 0.25)
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
    self.y_int = self.y_int + dt*(self.robot.loc.y - y)
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
      y_cont = self.k_y[0] * (globY - y) + self.k_y[1] * self.y_int
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_int
    else:
      x_cont = self.k_x[0] * (globX - x) + self.k_x[1] * self.x_int + self.k_x[2] *(globX - self.x_prev) / dt
      y_cont = self.k_y[0] * (globY - y) + self.k_y[1] * self.y_int + self.k_y[2] *(globY - self.y_prev) / dt
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_int + self.k_t[2] *(bearing - self.theta_prev) / dt

    print("p_cont: %f, i_cont: %f, d_cont: %f"%(self.k_x[0] * (globX - x) , self.k_x[1] * self.x_int , self.k_x[2] *(globX - self.x_prev) / dt))
      
    if abs(bearing) >=0.3:
      # Control only the heading of the robot and not the velocity
      commands.setWalkVelocity(0.0, 0.0, 0.4*np.sign(bearing))
    else:
      # Control both heading and velocity
      if self.gb_line_obj.seen:
        if globX < 1800.0:
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
    self.y_prev = globY - self.y_prev
    self.theta_prev = bearing
    self.time_last = self.time_current

    return

  def getDesiredGoaliePos(self):

    goalBallXComp = self.ball.loc.x - self.goalieStartX
    goalBallYComp = self.ball.loc.y - self.goalieStartY
    goalBallTh = np.arctan2(goalBallYComp,-goalBallXComp)
    print("Ball global pose: [%f, %f]" % (self.ball.loc.x,self.ball.loc.y))
    print("Ball to goal bearing: %f" % (goalBallTh*core.RAD_T_DEG))
    print("Goal Start: [%f,%f]"%(self.goalieStartX, self.goalieStartY))
    self.roboDesiredX = self.goalieStartX - self.radius*np.cos(goalBallTh)
    self.roboDesiredY = self.goalieStartY + self.radius*np.sin(goalBallTh)
    if goalBallTh > 0:
      self.roboDesiredTh = np.pi - goalBallTh
    else:
      self.roboDesiredTh = -np.pi - goalBallTh

    print("Robot should go to [%f, %f] with bearing: %f Robot at [%f,%f] with bearing: %f" %(self.roboDesiredX,self.roboDesiredY,self.roboDesiredTh*core.RAD_T_DEG,self.robot.loc.x,self.robot.loc.y,self.robot.orientation * core.RAD_T_DEG))
    return


class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,-5.0,1.0)
    if self.getTime() > 1.5:
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

class TurnInPlace(Node):
  """Turn in place if ball not found"""
  def run(self):
    commands.setWalkVelocity(0.0,0.0,-0.4)
    if self.getTime() > 2.5:
      self.finish()

class GoToBall(Node):
  """Face the ball"""
  def __init__(self, ball, dist):
    super(GoToBall, self).__init__()
    self.ball = ball
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
    self.theta_integral = self.theta_integral + dt*(self.ball.bearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.distance - self.dist)
    if abs(self.theta_integral) >= 15.0:
      self.theta_integral = 15.0*np.sign(self.theta_integral)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    self.time_current = time.clock()
    dt = self.time_current - self.time_last
    self.calc_integral(dt)
    bearing = self.ball.bearing
    distance = self.ball.distance
    elevation = core.RAD_T_DEG * self.ball.visionElevation
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
    # print("bearing: %f, distance: %f"%(self.ball.bearing, self.ball.distance))
    if abs(distance - self.dist) < 200.0:
      self.finish()

class TurnAroundBall(Node):
  """Circle the ball while searching for the goal"""
  def __init__(self, ball, dist, dir):
    super(TurnAroundBall, self).__init__()
    self.ball = ball
    self.k_t = (0.7, 0.01, 0.1)
    self.k_d = (0.001, 0.0001, 0.0001)
    self.dist = dist
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0
    self.dir = dir
    self.time_last = time.clock()
    self.time_current = time.clock()

  def calc_integral(self, dt):
    self.theta_integral = self.theta_integral + dt*(self.ball.bearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.distance - self.dist)
    if abs(self.theta_integral) >= 15.0:
      self.theta_integral = 15.0*np.sign(self.theta_integral)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    self.time_current = time.clock()
    dt = self.time_current - self.time_last
    self.calc_integral(dt)
    bearing = self.ball.bearing
    distance = self.ball.distance
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    commands.setHeadPanTilt(bearing,-25.0,1.5)
    if dt == 0:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
    else:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral + self.k_t[2] *(bearing - self.theta_prev) / dt
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    print("dist cont: %f, dir: %f, theta cont: %f" % (dist_cont, self.dir, theta_cont))
    commands.setWalkVelocity(dist_cont, 0.4 * self.dir, theta_cont)
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current

class Align(Node):
  """Turn around the ball to find the goal"""
  def __init__(self, ball, goal, dist, tilt, des_bearing):
    super(Align, self).__init__()
    self.ball = ball
    self.goal = goal
    self.dist = dist
    self.tilt = tilt
    self.des_bearing = des_bearing
    self.k_t = (0.7, 0.01, 0.1)
    self.k_d = (0.001, 0.0001, 0.0001)
    self.k_y = (0.8, 0.001, 0.001)
    self.theta_integral_ball = 0.0
    self.theta_prev_ball = 0.0
    self.theta_integral_delta = 0.0
    self.theta_prev_delta = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0

  def calc_integral(self, dt):
    self.theta_integral_ball = self.theta_integral_ball + dt*(self.ball.bearing)
    self.theta_integral_delta = self.theta_integral_delta + dt*(self.ball.bearing - self.goal.visionBearing - self.des_bearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.distance - self.dist)
    if abs(self.theta_integral_ball) >= 15.0:
      self.theta_integral_ball = 15.0*np.sign(self.theta_integral_ball)
    if abs(self.theta_integral_delta) >= 15.0:
      self.theta_integral_delta = 15.0*np.sign(self.theta_integral_delta)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    dt = 1.0/30.0
    self.calc_integral(dt)
    bearing_ball = self.ball.bearing
    bearing_goal = self.goal.visionBearing
    delta_bearing = bearing_ball - bearing_goal - self.des_bearing
    distance = self.ball.distance
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    commands.setHeadPanTilt(bearing_ball,self.tilt,1.5)
    if dt == 0:
      theta_cont = self.k_t[0] * bearing_ball + self.k_t[1] * self.theta_integral_ball
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
      y_cont = self.k_y[0] * delta_bearing + self.k_y[1] * self.theta_integral_delta
      delta_derivative = 0.0
    else:
      theta_cont = self.k_t[0] * bearing_ball + self.k_t[1] * self.theta_integral_ball + self.k_t[2] *(bearing_ball - self.theta_prev_ball) / dt
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
      y_cont = self.k_y[0] * delta_bearing + self.k_y[1] * self.theta_integral_delta + self.k_y[2] *(delta_bearing - self.theta_prev_delta) / dt
      if ((bearing_ball - self.theta_prev_ball)/dt) > 20.0:
        theta_cont = 0.0
        dist_cont = 0.0
      if ((delta_bearing - self.theta_prev_delta)/dt) > 20.0:
        y_cont = 0.0
      delta_derivative = (delta_bearing - self.theta_prev_delta) / dt
    commands.setWalkVelocity(dist_cont, y_cont, theta_cont)
    self.theta_prev_ball = bearing_ball
    self.theta_prev_delta = delta_bearing
    self.dist_prev = distance
    print("goal distance: %f, ball distance: %f, difference = %f"%(self.goal.visionDistance, distance, abs(self.goal.visionDistance - distance)))

class Stand(Node):
  def __init__(self, time):
    super(Stand, self).__init__()
    self.time = time
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,self.time)
    if self.getTime() > (self.time + 0.3):
      self.finish()

class PositionForKick(Node):
  """Dribble ball to within 1.0 m from the goal"""
  def __init__(self, ball, bearing, dist):
    super(PositionForKick, self).__init__()
    self.ball = ball
    self.bearing = bearing
    self.dist = dist
    self.k_d = (0.01, 0.001, 0.001)
    self.k_y = (0.8, 0.001, 0.001)
    self.theta_integral_ball = 0.0
    self.theta_prev_ball = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0

  def calc_integral(self, dt):
    self.theta_integral_ball = self.theta_integral_ball + dt*(self.ball.bearing - self.bearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.distance - self.dist)
    if abs(self.theta_integral_ball) >= 15.0:
      self.theta_integral_ball = 15.0*np.sign(self.theta_integral_ball)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    dt = 1.0/30.0
    self.calc_integral(dt)
    err_bearing = self.ball.bearing - self.bearing
    distance = self.ball.distance
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    commands.setHeadPanTilt(0.0,-45.0,1.5)
    if dt == 0:
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
      y_cont = self.k_y[0] * err_bearing + self.k_y[1] * self.theta_integral_goal
      delta_derivative = 0.0
    else:
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
      y_cont = self.k_y[0] * err_bearing + self.k_y[1] * self.theta_integral_ball + self.k_y[2] *(err_bearing - self.theta_prev_ball) / dt
      if ((err_bearing - self.theta_prev_ball)/dt) > 20.0:
        dist_cont = 0.0
      if ((err_bearing - self.theta_prev_ball)/dt) > 20.0:
        y_cont = 0.0
      delta_derivative = (err_bearing - self.theta_prev_ball) / dt
    commands.setWalkVelocity(dist_cont, y_cont, 0.0)
    self.theta_prev_ball = err_bearing
    self.dist_prev = distance

class Kick(Node):
  """Kick"""
  def run(self):
    if self.getFrames() <= 3:
      memory.walk_request.noWalk()
      memory.kick_request.setFwdKick()
    if self.getFrames() > 10 and not memory.kick_request.kick_running_:
      self.finish()

class Attacking(LoopingStateMachine):
  def setup(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
    line = mem_objects.world_objects[core.WO_OWN_PENALTY]
    by = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW)
    yb = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE)
    yp = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)
    py = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
    bp = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
    pb = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE)

    rdy = GetReady()
    moveHeadLeft = MoveHead(85.0,-15.0,1.0)
    moveHeadRight = MoveHead(-85.0,-45.0,2.0)
    turnInPlace = TurnInPlace()
    goToBall = GoToBall(ball, 0.0)

    moveHeadLeftGoal = MoveHead(85.0,0.0,1.0)
    moveHeadRightGoal = MoveHead(-85.0,0.0,2.0)
    moveHeadLeftBeacon = MoveHead(35.0,0.0,1.0)
    moveHeadRightBeacon = MoveHead(-35.0,0.0,1.0)

    turnAroundBall = TurnAroundBall(ball,200.0,1.0)
    turnAroundBallLeft = TurnAroundBall(ball,200.0,1.0)
    turnAroundBallRight = TurnAroundBall(ball,200.0,-1.0)
    
    des_bearing = 0.0
    lookUp = MoveHead(0.0,-5.0,1.0)
    lookDown = MoveHead(0.0,-45.0,1.5)
    align200 = Align(ball,goal,200.0, -15.0,des_bearing)

    dribble = Align(ball,goal,-280.0, -20.0,0.0)
    wait = Stand(1.0)
    align50 = Align(ball,goal,50.0,-15.0,0.0)

    alignForKick = Align(ball,goal, 0.0, -30.0,0.1)
    positionForKick = PositionForKick(ball, 0.28, 140.0)

    stand = Stand(1.5)
    stand_find_goal = Stand(1.0)
    stand_again = Stand(1.5)
    kick = Kick()

    # Keep turning head and turning in place till ball is found and then go to ball
    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight,C,turnInPlace,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,O(ball),goToBall)
    self.add_transition(moveHeadRight,O(ball),goToBall)
    self.add_transition(turnInPlace,O(ball),goToBall)
    self.add_transition(goToBall,NO(ball),moveHeadLeft)

    # After Robot reaches near the ball, maintain distance to ball and find the goal
    self.add_transition(goToBall,OD(ball,200.0),lookUp,C,moveHeadLeftBeacon,C,moveHeadRightBeacon,C,lookDown,C,turnAroundBall,C,moveHeadLeftGoal,C,moveHeadRightGoal,C,lookDown)

    self.add_transition(lookUp,OR(bp,pb),turnAroundBallLeft)
    self.add_transition(moveHeadLeftBeacon,OR(bp,pb),turnAroundBallLeft)
    self.add_transition(moveHeadRightBeacon,OR(bp,pb),turnAroundBallLeft)

    self.add_transition(lookUp,OR(by,yb),turnAroundBallRight)
    self.add_transition(moveHeadLeftBeacon,OR(by,yb),turnAroundBallRight)
    self.add_transition(moveHeadRightBeacon,OR(by,yb),turnAroundBallRight)

    self.add_transition(moveHeadLeftGoal,O(goal,ball),align200)
    self.add_transition(moveHeadRightGoal,O(goal,ball),align200)
    self.add_transition(lookDown,O(goal,ball),align200)
    self.add_transition(turnAroundBall,O(goal,ball),align200)
    self.add_transition(turnAroundBallLeft,O(goal,ball),align200)
    self.add_transition(turnAroundBallRight,O(goal,ball),align200)
    self.add_transition(align200,NO(goal,ball),moveHeadLeftGoal)
    self.add_transition(turnAroundBall,NO(ball),rdy)

    # # moveHeadLeftGoal,C,moveHeadRightGoal,C,lookDown,C,turnAroundBall,C,moveHeadLeftGoal)
    # self.add_transition(goToBall,OD(ball,200.0),moveHeadLeftGoal,C,moveHeadRightGoal,C,lookDown,C,turnAroundBall,C,moveHeadLeftGoal)

    # If the ball and goal are aligned with the robot, proceed to stopping, judging distance and dribbling/shooting
    self.add_transition(align200,AB(ball,goal,0.2,des_bearing),dribble)
    self.add_transition(dribble,A(ball,goal,0.2).negation(),align50)
    # self.add_transition(dribble,NO(goal,ball),stand_find_goal,O(goal),dribble)
    self.add_transition(align50,A(ball,goal),dribble)
    self.add_transition(dribble,D(ball,goal,2000.0),wait)
    # self.add_transition(dribble,D(ball,goal,3000.0).negation(),stand_find_goal,C,dribble)
    self.add_transition(wait,D(ball,goal,2000.0).negation(),dribble)
    self.add_transition(dribble,LD(line,300.0),alignForKick)

    # After it's dribbled, align between ball and goal again and then shift left
    self.add_transition(wait,T(1.0),alignForKick)
    self.add_transition(alignForKick, OD(ball,150.0), positionForKick)
    self.add_transition(positionForKick, BB(ball,0.28), stand)
    self.add_transition(stand, T(1.0), kick, C, stand_again, T(3.0), rdy)

class Defending(LoopingStateMachine):
  def setup(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    robot = world_objects.getObjPtr(core.WO_TEAM5)
   
    localized = False
    poseListX = []
    poseListY = []
    poseListTh = []
    pose_index = 0
    blocker = Blocker()
    lookStraight = MoveHead(0.0,-10.0,2.5)
    moveHeadLeft = MoveHead(85.0,-10.0,2.5)
    moveHeadRight = MoveHead(-85.0,-10.0,5.0)
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
