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
from state_machine import Node, C, T, S, LoopingStateMachine, StateMachine, EventNode, Event, NegationEvent

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

class AlignBallGoal(Event):
  """Aligning ball, goal and robot"""
  def __init__(self, ball, goal):
    super(AlignBallGoal, self).__init__()
    self.ball = ball
    self.goal = goal

  def ready():
    return aligned_boolean

def A(ball = None, goal = None):
  """Ball, Goal, Robot aligned"""
  return AlignBallGoal(ball, goal)

class ScoredGoal(Event):
  """Aligning ball, goal and robot"""
  def __init__(self, ball, goal):
    super(ScoredGoal, self).__init__()
    self.ball = ball
    self.goal = goal

  def ready():
    return scored_boolean

def S(ball = None, goal = None):
  """Goal scored"""
  return ScoredGoal(ball, goal)

class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadTilt(-30.0)
    if self.getTime() > 1.5:
      self.finish()

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

class FaceBall(Node):
  """Face the ball"""
  def __init__(self, ball):
    super(FaceBall, self).__init__()
    self.ball = ball

class WalkToBall(Node):
  """Controller node for tracking the ball"""
  integral = 0.0
  def __init__(self, ball, kP = 0.0, kI = 0.0, kD = 0.0):
    super(WalkToBall, self).__init__()
    self.ball = ball
    self.kP = kP
    self.kI = kI
    self.kD = kD

  def calc_integral(self):
    WalkToBall.integral = WalkToBall.integral + self.ball.visionDistance

  def run(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    bearing = ball.visionBearing
    elevation = core.RAD_T_DEG * ball.visionElevation
    distance = ball.visionDistance
    self.calc_integral()
    # print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f \t Integral: %.8f\n ' % (bearing, distance, elevation, WalkToBall.integral))
    print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f\t Location: (%d,%d)\n' % (bearing, distance, elevation, ball.imageCenterX, ball.imageCenterY))
    commands.setHeadPanTilt(0.0, 0.0, 0.1)
    # commands.setWalkVelocity(0.2, 0, 0)

class Align(Node):
  """Turn around the ball to find the goal"""
  def __init__(self, ball, goal):
    super(Align, self).__init__()
    self.ball = ball
    self.goal = goal

class Dribble(Node):
  """Dribble ball to within 1.0 m from the goal"""
  def __init__(self, ball, goal):
    super(Dribble, self).__init__()
    self.ball = ball
    self.goal = goal

class Kick(Node):
  """Kick"""
  def __init__(self, ball, goal):
    super(Kick, self).__init__()
    self.ball = ball
    self.goal = goal
  def run(self):
    if self.getFrames() <= 3:
      memory.walk_request.noWalk()
      memory.kick_request.setFwdKick()
    if self.getFrames() > 10 and not memory.kick_request.kick_running_:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
    stand = GetReady()
    #goToBall = WalkToBall()
    moveHeadLeft = MoveHeadLeft()
    moveHeadRight = MoveHeadRight()
    sit = pose.Sit()
    kP = 1.0
    kI = 1.0
    kD = 1.0
    walk = WalkToBall(ball, kP, kI, kD)
    self.add_transition(stand,C,walk)
    # self.add_transition(walk,T(3.0),sit)