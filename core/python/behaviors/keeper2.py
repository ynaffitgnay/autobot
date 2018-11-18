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

    return pose.BlockLeft()

class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
  
    return pose.BlockRight()

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking center")
    
    return #pose.Squat(time=3.0)

class MoveBlock(Node):
  def run(self):
    UTdebug.log(15, "Moving to Blocking")
    
    return #pose.Squat(time=3.0)

class Reset(Node):
  """Go back to normal"""
  def run(self):
    UTdebug.log(15,"Resetting keeper")

    return #pose.Sit()

class DontBlock(Node):
  """Ball missed dont block"""
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
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

class Blocker(Node):

  def run(self):
    # if not ball.seen, move the head back to the center
    commands.setWalkVelocity(0.0,0.0,0.0)
    ball = mem_objects.world_objects[core.WO_BALL]
    line_obj = mem_objects.world_objects[core.WO_OWN_PENALTY]
    line = line_obj.lineLoc
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
    if ball.seen and ball.relVel.x < 0.0:
      if ball.relVel.x > v_thresh:
        """do nothing"""
        pass
      else:
        # ball moving away from the robot, reset to regular position
        y_intercept = -ball.relVel.y * x_ball /ball.relVel.x + y_ball
        print("Current Vx: %f, Current Vy: %f, x ball: %f, y ball: %f" % (ball.relVel.x, ball.relVel.y, x_ball, y_ball))
        print("Y intercept: %f" % y_intercept)
        if y_intercept > 25.0 or y_intercept < -25.0:
          rel_robot = core.Point2D(0.0,0.0)
          distance = line.getDistanceTo(rel_robot)
          if line_obj.seen and distance < 100.0:
            x_cont = 0.0
          else:
            x_cont = 0.2
          commands.setWalkVelocity(x_cont,0.4 * np.sign(y_intercept),0.0)
          return
        elif y_intercept > 18.0:
          choice = "left"
        elif y_intercept < -18.0:
          choice = "right"
        elif y_intercept <= 18.0 and y_intercept >= -18.0:
          choice = "center"
        self.postSignal(choice)
        return

      #   ## can do if xvel < yvel && xvel < 1, etc.
      #   ## maybe look at distance between robot and goal? 
      #   UTdebug.log(15, "Ball is close, blocking!")
      #   if y_intercept > 42.0 or y_intercept < -40.0 or not ball.seen:
      #     choice = "miss"
      # elif ball.seen and ball.relVel.x > v_thresh_2 and ball.relVel.x < 0.0:
      #   choice = "moveBall"
      #   # print("Move ball signal")
      #   self.postSignal(choice)
      #   return
      # else:  
      #   return

class Defending(LoopingStateMachine):
  def setup(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    
    blocker = Blocker()
    lookStraight = MoveHead(0.0,-10.0,2.5)
    moveHeadLeft = MoveHead(85.0,-10.0,2.5)
    moveHeadRight = MoveHead(-85.0,-10.0,5.0)
    reset = Reset()
    rdy = GetReady()
    blocks = {"left": BlockLeft(),
              "right": BlockRight(),
              "center": BlockCenter(),
              "miss": DontBlock()
              }

    reset = DontBlock()
    for name in blocks:
      b = blocks[name]
      self.add_transition(blocker, S(name), b, T(2), reset, T(3.0), blocker)
