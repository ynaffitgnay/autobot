"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import math
import core
import pose
import head
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, S, LoopingStateMachine, EventNode, Event, NegationEvent

class BallSeen(Event):
  """Event that fires if Ball is seen"""
  def __init__(self, ball):
    super(BallSeen, self).__init__()
    self.ball = ball
  def ready(self):
    return self.ball.seen

def B(ball=None):
  """Ball found"""
  return BallSeen(ball)

class GoalSeen(Event):
  """Event that fires if goal is seen"""
  def __init__(self, goal):
    super(GoalSeen, self).__init__()
    self.goal = goal
  def ready(self):
    return self.goal.seen

def G(goal=None):
  """Ball found"""
  return GoalSeen(goal)

class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadTilt(-30.0)
    if self.getTime() > 1.5:
      self.finish()

class WalkToBall(Node):
  """Controller node for tracking the ball"""
  def __init__(self, ball, kP, kI, kD, ):
    super(WalkToBall, self).__init__()
    self.ball = ball
    self.kP = kP
    self.kI = kI
    self.kD = kD

  def run(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    bearing = ball.visionBearing
    elevation = ball.visionElevation
    distance = ball.visionDistance
    print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f' % (bearing, distance, elevation))
    commands.setHeadPanTilt(bearing, -30.0, 0.1)
    commands.setWalkVelocity()

class MoveHeadLeft(Node):
  """Search for the ball to the left"""
  def run(self):
    commands.setHeadPanTilt(-math.pi/2,-30.0,1.5)
    commands.setHeadTilt(-30.0)
    if self.getTime() > 2.5:
      self.finish()

class MoveHeadRight(Node):
  """Search for the ball to the right"""
  def run(self):
    commands.setHeadPanTilt(math.pi/2,-30.0,1.5)
    commands.setHeadTilt(-30.0)
    if self.getTime() > 2.5:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
    stand = GetReady()
    goToBall = WalkToBall()
    moveHeadLeft = MoveHeadLeft()
    moveHeadRight = MoveHeadRight()
    sit = pose.Sit()
    self.add_transition(stand,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,C,moveHeadRight)
    self.add_transition(moveHeadRight,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,B(ball),goToBall)
    self.add_transition(moveHeadRight,B(ball),goToBall)
    self.add_transition(goToBall,B(ball).negation(),moveHeadLeft)