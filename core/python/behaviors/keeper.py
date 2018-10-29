"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
import task, util
import head
import math
import random
from task import Task, MultiTask
import cfgpose, cfgstiff
import pose
#from pose import ToPoseMoveHead
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles, world_objects
from state_machine import Node, S, C, T, LoopingStateMachine, Event
import UTdebug

# class BallSeen(Event):
#   """Event that fires if Ball is seen"""
#   def __init__(self, ball):
#     super(BallSeen, self).__init__()
#     self.ball = ball
#   def ready(self):
#     return self.ball.seen

# def B(ball):
#   """Ball found"""
#   return BallSeen(ball)

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
  def __init__(self):
    super(GetReady, self).__init__()
    # robot = world_objects.getObjPtr(core.WO_TEAM5)
    # goalieStartX = robot.loc.x
    # goalieStartY = robot.loc.y
    # self.goalieStartX = goalieStartX
    # self.goalieStartY = goalieStartY
    self.initial = False
  def run(self):
    robot = world_objects.getObjPtr(core.WO_TEAM5)
    xRobo = robot.loc.x
    yRobo = robot.loc.y
    
    gb_line_obj = world_objects.getObjPtr(core.WO_OWN_PENALTY)
    if gb_line_obj.seen:
      gb_line_seg = gb_line_obj.lineLoc
      gb_line_cent = gb_line_seg.getCenter()
      gb_line_dist = gb_line_seg.getDistanceTo(gb_line_cent)
      gb_robo_dist = gb_line_seg.getPointOnSegmentClosestTo(robot.loc)
      print("Line seen: %d Center at [%f, %f] with distace: %f, but robot at [%f, %f] dist to closest point is: %f" % (gb_line_obj.seen,gb_line_cent.x,gb_line_cent.y,gb_line_dist,xRobo,yRobo,gb_robo_dist))

    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)
    if xRobo < 200.0:
      commands.setWalkVelocity(0.3,0.0,0.0)
    else:
      commands.setWalkVelocity(0.0,0.0,0.0)
    if self.getTime() >5.0:
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
  def run(self):

    commands.setHeadPanTilt(0.0,-10.0,1.0)
    robot = world_objects.getObjPtr(core.WO_TEAM5)
    check = random.randint(0,100)
    print("Check number: %d" % check)
    if (check % 2) > 0:
      print("Lost")
      self.postSignal("lost")
      return
    else:
      print("Localized")
      self.postSignal("localized")
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
  def __init__(self):
    super(MoveBtwBall, self).__init__()

  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    robot = world_objects.getObjPtr(core.WO_TEAM5)

    gb_line_obj = mem_objects.world_objects[core.WO_OWN_PENALTY]
    # if gb_line_obj.seen:
    #   gb_line_seg = gb_line_obj.lineLoc
    #   gb_line_cent = gb_line_seg.getCenter()
    #   gb_line_dist = gb_line_seg.getDistanceTo(gb_line_cent)
    #   gb_robo_dist = gb_line_seg.getPointOnSegmentClosestTo(robot.loc)
    # print("Line seen: %d " % gb_line_obj.seen) # Center at [%f, %f] with distace: %f, but robot at [%f, %f] dist to closest point is: %f" % (gb_line_obj.seen,gb_line_cent.x,gb_line_cent.y,gb_line_dist,xRobo,yRobo,gb_robo_dist))

    if ball.seen:
      commands.setHeadPan(ball.bearing, 0.2)
    # print("Ball dist: %f Ball bear: %f\n" % (ball.distance, ball.bearing))
    # print("Robot pose: [%f,%f,%f]\n" %(robot.loc.x,robot.loc.y,robot.orientation*core.RAD_T_DEG))
    
    # TODO: Change this to be a check of the state of localization
    check = random.randint(0,100)
    # print("Move check number: %d" % check)
    

    if self.getTime() > 2.0:
      if check < 90:
        signal = "localized"
      else:
        signal = "lost"
      self.postSignal(signal)



class Playing(LoopingStateMachine):
  def setup(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    robot = world_objects.getObjPtr(core.WO_TEAM5)
    blocker = Blocker()
    lookStraight = MoveHead(0.0,-10.0,1.5)
    moveHeadLeft = MoveHead(85.0,-10.0,1.5)
    moveHeadRight = MoveHead(-85.0,-10.0,3.0)
    reset = Reset()
    rdy = GetReady()
    moveBtwBall = MoveBtwBall()
    checkLoc = CheckIfLocalized()
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
