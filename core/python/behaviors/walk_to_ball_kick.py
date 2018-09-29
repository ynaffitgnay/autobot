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

def NB(ball=None):
  """No ball found"""
  return NegationEvent(BallSeen(ball))

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

class BallClose(Event):
  """Event that fires if Ball is close"""
  def __init__(self, ball, dist):
    super(BallClose, self).__init__()
    self.ball = ball
    self.dist = dist
  def ready(self):
    return abs(self.ball.visionDistance) < self.dist

def CL(ball=None, dist = 200.0):
  """Ball is close, dist distance away """
  return BallClose(ball, dist)

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

class Aligned(Event):
  """Aligning ball, goal and robot"""
  def __init__(self, ball, goal):
    super(Aligned, self).__init__()
    self.ball = ball
    self.goal = goal

  def ready():
    return ((abs(ball.visionBearing-goal.visionBearing) < 0.1) and (abs(core.joint_values[core.HeadYaw]) < 0.1))

def A(ball = None, goal = None):
  """Ball, Goal, Robot aligned"""
  return Aligned(ball, goal)

class CheckDribble(Event):
  """Check whether or not robot should Dribble"""
  def __init__(self, ball, goal):
    super(CheckDribble, self).__init__()
    self.ball = ball
    self.goal = goal

  def ready():
    return self.goal.visionDistance > 1500.0

def D(ball = None, goal = None):
  """Ball, Goal, Robot aligned"""
  return CheckDribble(ball, goal)

"""I don't think we need this because page 1 of the assignment says it will be manually moved to about 2 meters away"""
class ScoredGoal(Event):
  """Aligning ball, goal and robot"""
  def __init__(self, ball, goal):
    super(ScoredGoal, self).__init__()
    self.ball = ball
    self.goal = goal

  def ready(self):
    ballBearing = self.ball.visionBearing
    ballDistance = self.ball.visionDistance
    goalBearing = self.goal.visionBearing
    goalDistance = self.goal.visionDistance
    theta = self.goal.orientation
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
  def __init__(self, tilt):
    super(MoveHeadLeft, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(core.DEG_T_RAD*110.0,self.tilt,1.5)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 2.5:
      self.finish()

class MoveHeadRight(Node):
  """Search for the ball to the right"""
  def __init__(self, tilt):
    super(MoveHeadRight, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(-core.DEG_T_RAD*110.0,self.tilt,1.5)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 2.5:
      self.finish()

class TurnInPlace(Node):
  """Turn in place if ball not found"""
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.2)
    if self.getTime() > 2.5:
      self.finish()

class GoToBall(Node):
  """Face the ball"""
  def __init__(self, ball, dist):
    super(GoToBall, self).__init__()
    self.ball = ball
    self.dist = dist
    self.k_t = (0.1, 0.01, 0.01)
    self.k_d = (0.1, 0.05, 0.01)
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()

  def calc_integral(self, dt):
    self.theta_integral = self.theta_integral + dt*(self.ball.visionBearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.visionDistance - self.dist)
    if abs(self.theta_integral) >= 15.0:
      self.theta_integral = 15.0*np.sign(self.theta_integral)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    self.time_current = time.clock()
    dt = self.time_current - self.time_last
    self.calc_integral(dt)
    bearing = self.ball.visionBearing
    distance = self.ball.visionDistance
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    print("Elevation: %.3f\t" % elevation)
    commands.setHeadPanTilt(bearing, -elevation, 1.5)
    if dt == 0:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
    else:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral + self.k_t[2] *(bearing - self.theta_prev) / dt
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    
    if abs(bearing) >=0.1:
      # Control only the heading of the robot and not the velocity
      print("Bearing only = %.5f, theta_cont = %.5f\n" %(bearing, theta_cont))
      commands.setWalkVelocity(0.0, 0.0, theta_cont)
    else:
      # Control both heading and velocity
      if abs(distance) >= 600.0:
        print("Bearing = %.5f, distance = %.5f, theta_cont = %.5f\n" %(bearing, distance, theta_cont))
        commands.setWalkVelocity(1.0, 0.0, theta_cont)
      else:
        print("theta_cont = %.5f, dist_cont = %.5f\n" %(theta_cont, dist_cont))
        commands.setWalkVelocity(dist_cont, 0.0, theta_cont)
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current

class TurnAroundBall(Node):
  """Circle the ball while searching for the goal"""
  def __init__(self, ball, dist):
    super(TurnAroundBall, self).__init__()
    self.ball = ball
    self.k_t = (0.1, 0.01, 0.01)
    self.k_d = (0.1, 0.05, 0.01)
    self.dist = dist
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()

  def calc_theta_integral(self, dt):
    self.theta_integral = self.theta_integral + dt*(self.ball.visionBearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.visionDistance - self.dist)
    if abs(self.theta_integral) >= 15.0:
      self.theta_integral = 15.0*np.sign(self.theta_integral)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    self.time_current = time.clock()
    dt = self.time_current - self.time_last
    self.calc_integral(dt)
    bearing = self.ball.visionBearing
    commands.setHeadPanTilt(bearing,-15.0,1.5)
    theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral + self.k_t[2] *(bearing - self.theta_prev) / dt
    dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    commands.setWalkVelocity(0.01*dist_cont, 0.2, 0.001*theta_cont)
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current
    if self.getTime() > 2.5:
      self.finish()

class Align(Node):
  """Turn around the ball to find the goal"""
  def __init__(self, ball, goal, dist):
    super(Align, self).__init__()
    self.ball = ball
    self.goal = goal
    self.dist = dist
    self.k_t = (0.1, 0.01, 0.01)
    self.k_d = (0.1, 0.05, 0.01)
    self.k_y = (0.1, 0.05, 0.01)
    self.theta_integral = 0.0
    self.theta_prev = 0.0
    self.dist_integral = 0.0
    self.dist_prev = 0.0
    self.time_last = time.clock()
    self.time_current = time.clock()

  def calc_theta_integral(self, dt):
    self.theta_integral = self.theta_integral + dt*(self.ball.visionBearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.visionDistance - self.dist)
    if abs(self.theta_integral) >= 15.0:
      self.theta_integral = 15.0*np.sign(self.theta_integral)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    self.time_current = time.clock()
    dt = self.time_current - self.time_las
    self.calc_integral(dt)
    bearing = 0.5 * self.ball.visionBearing + 0.5 * self.goal.visionBearing
    commands.setHeadPanTilt(bearing,-10.0,1.5)
    theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral + self.k_t[2] *(bearing - self.theta_prev) / dt
    dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    y_cont = self.k_y[0] * bearing + self.k_y[1] * self.theta_integral + self.k_y[2] *(bearing - self.theta_prev) / dt
    commands.setWalkVelocity(dist_cont, y_cont, theta_cont)
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current

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
    moveHeadLeft = MoveHeadLeft(-15.0)
    moveHeadRight = MoveHeadRight(-15.0)
    turnInPlace = TurnInPlace()
    goToBall = GoToBall(ball, 100.0)

    # moveHeadLeftGoal = MoveHeadLeft(0.0)
    # moveHeadRightGoal = MoveHeadRight(0.0)
    # turnAroundBall = TurnAroundBall(ball,100.0)
    # align = Align(ball,goal,100.0)

    # wait = commands.stand()
    # dribble = Dribble()
    # kick = Kick()
    
    # Keep turning head and turning in place till ball is found and then go to ball
    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight,C,turnInPlace,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,B(ball),goToBall)
    self.add_transition(moveHeadRight,B(ball),goToBall)
    self.add_transition(turnInPlace,B(ball),goToBall)
    self.add_transition(goToBall,NB(ball),moveHeadLeft)

    # # After Robot reaches near the ball, maintain distance to ball and find the goal
    # self.add_transition(goToBall,CL(ball,100.0),moveHeadLeftGoal,C,moveHeadRightGoal,C,turnAroundBall,C,moveHeadLeft)
    # self.add_transition(moveHeadLeftGoal,G(goal),align)
    # self.add_transition(moveHeadRightGoal,G(goal),align)
    # self.add_transition(turnInPlace,G(goal),align)

    # # If the ball and goal are aligned with the robot, proceed to stopping, judging distance and dribbling/shooting
    # self.add_transition(align,A(ball,goal),wait)
    # self.add_transition(wait,D(ball,goal),dribble)
    # self.add_transition(wait,D(ball,goal).negation(),shoot)
    # self.add_transition(dribble,D(ball,goal).negation(),shoot)




















# class WalkToBall(Node):
#   """Controller node for tracking the ball"""
#   def __init__(self, ball):
#     super(WalkToBall, self).__init__()
#     self.ball = ball
#     self.kP_t = 0.1
#     self.kI_t = 0.05
#     self.kD_t = 0.01
#     self.kP_d = 0.1
#     self.kI_d = 0.05
#     self.kD_d = 0.01
#     self.theta_integral = 0.0
#     self.theta_prev = 0.0
#     self.dist_integral = 0.0
#     self.dist_prev = 0.0
#     self.time_last = time.clock()
#     self.time_current = time.clock()

#   def calc_integral(self):
#     self.theta_integral = self.theta_integral + (self.time_current - self.time_last)*(self.ball.visionBearing)
#     self.dist_integral = self.dist_integral + (self.time_current - self.time_last)*(self.ball.visionDistance - self.dist)
#     if abs(self.theta_integral) >= 15:
#       self.theta_integral = 15*np.sign(self.theta_integral)
#     if abs(self.dist_integral) >= 100:
#       self.dist_integral = 100*np.sign(self.dist_integral)

#   def run(self):
#     self.time_current = time.clock()
#     self.calc_integral()
#     bearing = self.ball.visionBearing
#     distance = self.ball.visionDistance
#     elevation = core.RAD_T_DEG * self.ball.visionElevation
#     commands.setHeadPanTilt(bearing, -elevation, 0.1)
#     if abs(distance) > 600.0:
#       if abs(bearing) > 0.2:
#         commands.setWalkVelocity(1.0, 0.0, 0.2*(np.sign(bearing)))
#       else:
#         commands.setWalkVelocity(1.0, 0.0, 0.0)
#     else:
#       theta_cont = self.kP_t * bearing + self.kI_t * self.theta_integral + self.kD_t *(bearing - self.theta_prev) / (self.time_current - self.time_last)
#       dist_cont = self.kP_d * (distance - 300.0) + self.kI_d * self.dist_integral + self.kD_d *(distance - self.dist_prev) / (self.time_current - self.time_last)
#       print("dist_cont: %.5f, Distance: %.5f, Integral: %.5f, Difference: %.5f\n" % (dist_cont, distance, self.dist_integral, (distance - self.dist_prev)))
#       commands.setWalkVelocity(0.01*dist_cont, 0.0, 0.001*theta_cont)
#     self.theta_prev = bearing
#     self.dist_prev = distance
#     self.time_last = self.time_current