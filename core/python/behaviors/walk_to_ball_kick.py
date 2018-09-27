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
    return ((abs(ball.visionBearing-goal.visionBearing) < 0.1) and (abs(core.joint_values[core.HeadYaw]) < 0.1))

def A(ball = None, goal = None):
  """Ball, Goal, Robot aligned"""
  return AlignBallGoal(ball, goal)

"""I don't think we need this because page 1 of the assignment says it will be manually moved to about 2 meters away"""
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
    commands.setHeadTilt(-10.0)
    if self.getTime() > 1.5:
      self.finish()

class MoveHeadLeft(Node):
  """Search for the ball to the left"""
  def run(self):
    commands.setHeadPanTilt(core.DEG_T_RAD*110.0,-15.0,1.5)
    commands.setHeadTilt(-15.0)
    if self.getTime() > 2.5:
      self.finish()

class MoveHeadRight(Node):
  """Search for the ball to the right"""
  def run(self):
    commands.setHeadPanTilt(-core.DEG_T_RAD*110.0,-15.0,1.5)
    commands.setHeadTilt(-15.0)
    if self.getTime() > 2.5:
      self.finish()

class FaceBall(Node):
  """Face the ball"""
  def __init__(self, ball):
    super(FaceBall, self).__init__()
    self.ball = ball
    self.kP = 0.1
    self.kI = 0.01
    self.kD = 0.01
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()

  def calc_theta_integral(self):
    self.theta_integral = self.theta_integral + (self.time_current - self.time_last)*(self.ball.visionBearing)
    if abs(self.theta_integral) >= 15:
      self.theta_integral = 15*np.sign(self.theta_integral)

  def run(self):
    self.time_current = time.clock()
    self.calc_theta_integral()
    bearing = self.ball.visionBearing
    commands.setHeadPanTilt(bearing, -15.0, 0.1)
    if abs(self.ball.visionBearing) > 0.1:
      commands.setWalkVelocity(0.0, 0.0, 0.5*(np.sign(self.ball.visionBearing)))
    else:
      theta_cont = self.kP * bearing + self.kI * self.theta_integral + self.kD *(bearing - self.theta_prev) / (self.time_current - self.time_last)
      print("Controller input: %.5f, Bearing: %.5f, Integral: %.5f, Difference: %.5f, Delta T: %.5f\n" % (theta_cont, bearing, self.theta_integral, (bearing - self.theta_prev), (self.time_current - self.time_last)))
      commands.setWalkVelocity(0.0, 0.0, 0.0001*theta_cont)
    self.theta_prev = self.ball.visionBearing
    self.time_last = self.time_current

class WalkToBall(Node):
  """Controller node for tracking the ball"""
  def __init__(self, ball):
    super(WalkToBall, self).__init__()
    self.ball = ball
    self.kP = 1.0
    self.kI = 1.0
    self.kD = 1.0
    self.integral = 0.0
    self.dist_prev = 0.0

  def calc_integral(self):
    self.integral = self.integral + self.ball.visionDistance

  def run(self):
    bearing = self.ball.visionBearing
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    distance = self.ball.visionDistance
    self.calc_integral()
    # print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f \t Integral: %.8f\n ' % (bearing, distance, elevation, self.integral))
    print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f\t Location: (%d,%d)\n' % (bearing, distance, elevation, self.ball.imageCenterX, self.ball.imageCenterY))
    commands.setHeadPanTilt(bearing, -elevation, 0.1)
    if abs(self.ball.visionDistance) > 1500.0:
      commands.setWalkVelocity(0.0, 0.0, -0.5*(np.sign(self.ball.visionBearing)))
    else:
      theta_cont = self.kP * self.ball.visionDistance + self.kI * self.theta_integral + self.kD * 20.0 *(self.ball.visionBearing - self.theta_prev)
      print("Controller input: %.5f\n" % (-theta_cont))
      commands.setWalkVelocity(0.0, 0.0, -theta_cont)
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
    rdy = GetReady()
    #goToBall = WalkToBall()
    moveHeadLeft = MoveHeadLeft()
    moveHeadRight = MoveHeadRight()
    faceball = FaceBall(ball)
    # walk = WalkToBall(ball, kP, kI, kD)
    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight)
    self.add_transition(moveHeadRight,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,B(ball),faceball)
    self.add_transition(moveHeadRight,B(ball),faceball)
    self.add_transition(faceball,B(ball).negation(),moveHeadLeft)
    # self.add_transition(walk,T(3.0),sit)