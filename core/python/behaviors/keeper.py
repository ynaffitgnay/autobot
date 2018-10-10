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
from task import Task, MultiTask
import cfgpose, cfgstiff
import pose
#from pose import ToPoseMoveHead
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles
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
    
    ball = mem_objects.world_objects[core.WO_BALL]
    commands.setStiffness(cfgstiff.OneArmsOnly)
    commands.setHeadPan(ball.bearing, 0.2)
    newPose = dict()
    newPose[core.LShoulderRoll] = 70
    newPose[core.LShoulderPitch] = -95
    newPose[core.LElbowRoll] = -1
    
    # Reset right arm to neutral
    newPose[core.RShoulderRoll] = 15
    newPose[core.RShoulderPitch] = -100
    newPose[core.RElbowRoll] = 0
    newPose[core.RHipPitch] = -46.5
    newPose[core.LHipPitch] = -46.5
    return pose.ToPoseMoveHead(pose=newPose, tilt=-15, pan = ball.bearing, time=0.1)

class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")

    ball = mem_objects.world_objects[core.WO_BALL]
    commands.setStiffness(cfgstiff.OneArmsOnly)
    commands.setHeadPan(ball.bearing, 0.2)
    newPose = dict()
    newPose[core.RShoulderRoll] = 70
    newPose[core.RShoulderPitch] = -95
    newPose[core.RElbowRoll] = -1

    # Reset Left arm to neutral
    newPose[core.LShoulderRoll] = 15
    newPose[core.LShoulderPitch] = -100
    newPose[core.LElbowRoll] = 0
    newPose[core.RHipPitch] = -46.5
    newPose[core.LHipPitch] = -46.5
    return pose.ToPoseMoveHead(pose=newPose, tilt=-15, pan = ball.bearing, time=0.1)

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking center")

    ball = mem_objects.world_objects[core.WO_BALL]
    commands.setStiffness(cfgstiff.OneArmsOnly)
    commands.setHeadPan(ball.bearing, 0.2)
    newPose = dict()
    newPose[core.LShoulderRoll] = 9.95
    newPose[core.LShoulderPitch] = -3.85
    newPose[core.LElbowRoll] = -1
    newPose[core.RShoulderRoll] = 9.95
    newPose[core.RShoulderPitch] = -3.85
    newPose[core.RElbowRoll] = -1
    newPose[core.RHipPitch] = -46.5
    newPose[core.LHipPitch] = -46.5
    return pose.ToPoseMoveHead(pose=newPose, tilt=-15, pan = ball.bearing, time=0.25)

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
    return pose.ToPoseMoveHead(pose=newPose, tilt=-15, pan = ball.bearing, time=0.5)

# class MoveHeadLeft(Node):
#   """Search for the ball to the left"""
#   def run(self):
#     commands.setHeadPanTilt(core.DEG_T_RAD*85.0,-15.0,1.0)
#     if self.getTime() > 1.2:
#       self.finish()

# class MoveHeadRight(Node):
#   """Search for the ball to the right"""
#   def run(self):
#     commands.setHeadPanTilt(-core.DEG_T_RAD*85.0,-15.0,1.0)
#     if self.getTime() > 1.2:
#       self.finish()

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
    print("X: %f, Y: %f, VThresh: %f, VMag: %f, Vx: %f, Vy: %f" % (x_ball, y_ball, v_thresh, v_mag, ball.relVel.x, ball.relVel.y))
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

class Playing(LoopingStateMachine):
  def setup(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    blocker = Blocker()
    blocks = {"left": BlockLeft(),
              "right": BlockRight(),
              "center": BlockCenter(),
              "miss": DontBlock()
              }
    reset = DontBlock()
    for name in blocks:
      b = blocks[name]
      self.add_transition(blocker, S(name), b, T(2), reset, T(3.0), blocker)