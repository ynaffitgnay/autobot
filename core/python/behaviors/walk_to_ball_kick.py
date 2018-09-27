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

class BallCentered(Event):
  """Event that fires if Ball is centered"""
  def __init__(self, ball, thresh):
    super(BallCentered, self).__init__()
    self.ball = ball
    self.thresh = thresh
  def ready(self):
    return abs(self.ball.visionBearing) < self.thresh

def BC(ball=None, thresh = 0.1):
  """Ball centered"""
  return BallCentered(ball, thresh)

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
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    commands.setHeadPanTilt(bearing, -elevation, 0.1)
    if abs(self.ball.visionBearing) > 0.1:
      commands.setWalkVelocity(0.0, 0.0, 0.5*(np.sign(self.ball.visionBearing)))
    else:
      theta_cont = self.kP * bearing + self.kI * self.theta_integral + self.kD *(bearing - self.theta_prev) / (self.time_current - self.time_last)
      # print("Controller input: %.5f, Bearing: %.5f, Integral: %.5f, Difference: %.5f, Delta T: %.5f\n" % (theta_cont, bearing, self.theta_integral, (bearing - self.theta_prev), (self.time_current - self.time_last)))
      commands.setWalkVelocity(0.0, 0.0, 0.01*theta_cont)
    self.theta_prev = bearing
    self.time_last = self.time_current

class WalkToBall(Node):
  """Controller node for tracking the ball"""
  def __init__(self, ball):
    super(WalkToBall, self).__init__()
    self.ball = ball
    self.kP_t = 0.1
    self.kI_t = 0.05
    self.kD_t = 0.01
    self.kP_d = 0.1
    self.kI_d = 0.05
    self.kD_d = 0.01
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()

  def calc_integral(self):
    self.theta_integral = self.theta_integral + (self.time_current - self.time_last)*(self.ball.visionBearing)
    self.dist_integral = self.dist_integral + (self.time_current - self.time_last)*(self.ball.visionDistance - 300.0)
    if abs(self.theta_integral) >= 15:
      self.theta_integral = 15*np.sign(self.theta_integral)
    if abs(self.dist_integral) >= 100:
      self.dist_integral = 100*np.sign(self.dist_integral)

  def run(self):
    self.time_current = time.clock()
    self.calc_integral()
    bearing = self.ball.visionBearing
    distance = self.ball.visionDistance
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    commands.setHeadPanTilt(bearing, -elevation, 0.1)
    if abs(distance) > 600.0:
      if abs(bearing) > 0.2:
        commands.setWalkVelocity(1.0, 0.0, 0.2*(np.sign(bearing)))
      else:
        commands.setWalkVelocity(1.0, 0.0, 0.0)
    else:
      theta_cont = self.kP_t * bearing + self.kI_t * self.theta_integral + self.kD_t *(bearing - self.theta_prev) / (self.time_current - self.time_last)
      dist_cont = self.kP_d * (distance - 300.0) + self.kI_d * self.dist_integral + self.kD_d *(distance - self.dist_prev) / (self.time_current - self.time_last)
      print("dist_cont: %.5f, Distance: %.5f, Integral: %.5f, Difference: %.5f\n" % (dist_cont, distance, self.dist_integral, (distance - self.dist_prev)))
      commands.setWalkVelocity(0.01*dist_cont, 0.0, 0.001*theta_cont)
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current

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
    walk = WalkToBall(ball)
    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight)
    self.add_transition(moveHeadRight,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,B(ball),faceball)
    self.add_transition(moveHeadRight,B(ball),faceball)
    self.add_transition(faceball,B(ball).negation(),moveHeadLeft)
    self.add_transition(faceball,BC(ball,0.1),walk)
    self.add_transition(walk,BC(ball,0.3).negation(),faceball)