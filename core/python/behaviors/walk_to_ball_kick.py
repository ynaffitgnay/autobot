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

def B(ball):
  """Ball found"""
  return BallSeen(ball)

class BallDistance(Event):
  """Event that fires if Ball is close enough"""
  def __init__(self, ball, dist):
    super(BallDistance, self).__init__()
    self.ball = ball
    self.dist = dist
  def ready(self):
    return (self.ball.visionDistance < self.dist)

def BD(ball, dist = 50.0):
  """Ball close enough"""
  return BallDistance(ball, dist)

class BallBearing(Event):
  """Event that fires if Ball bearing is close enough"""
  def __init__(self, ball, bearing):
    super(BallBearing, self).__init__()
    self.ball = ball
    self.bearing = bearing
  def ready(self):
    return abs(self.ball.visionBearing - self.bearing) < 0.1

def BB(ball, bearing = 0.28):
  """Ball close enough"""
  return BallBearing(ball, bearing)

def NB(ball):
  """No ball found"""
  return NegationEvent(BallSeen(ball))

class GoalBallSeen(Event):
  """Event that fires if goal and ball are seen in the same frame"""
  def __init__(self, goal, ball):
    super(GoalBallSeen, self).__init__()
    self.goal = goal
    self.ball = ball
  def ready(self):
    return (self.goal.seen and self.ball.seen)

def GB(goal, ball):
  """Ball found"""
  return GoalBallSeen(goal, ball)

def NGB(goal, ball):
  """Ball found"""
  return NegationEvent(GoalBallSeen(goal, ball))

class Aligned(Event):
  """Aligning ball, goal and robot"""
  def __init__(self, ball, goal, tolerance):
    super(Aligned, self).__init__()
    self.ball = ball
    self.goal = goal
    self.tolerance = tolerance

  def ready(self):
    print("goal dist: %.3f\n" % (self.goal.visionDistance))
    if (self.goal.visionDistance < 780.0):
      return (abs(self.ball.visionBearing < self.tolerance))
    return (abs(self.ball.visionBearing) < self.tolerance and abs(self.goal.visionBearing) < self.tolerance)

def A(ball, goal, tolerance = 0.1):
  """Ball, Goal, Robot aligned"""
  return Aligned(ball, goal, tolerance)

class CheckDribble(Event):
  """Check whether or not robot should Dribble"""
  def __init__(self, ball, goal, dist):
    super(CheckDribble, self).__init__()
    self.ball = ball
    self.goal = goal
    self.dist = dist

  def ready(self):
    return (self.goal.visionDistance - self.ball.visionDistance) > self.dist

def D(ball, goal, dist = 1200.0):
  """Ball, Goal, Robot aligned"""
  return CheckDribble(ball, goal, dist)

class GetReady(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(-core.DEG_T_RAD*110.0,-10.0,1.5)
    if self.getTime() > 2.5:
      self.finish()

class MoveHeadLeft(Node):
  """Search for the ball to the left"""
  def __init__(self, tilt):
    super(MoveHeadLeft, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(core.DEG_T_RAD*85.0,self.tilt,1.0)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 1.2:
      self.finish()

class MoveHeadRight(Node):
  """Search for the ball to the right"""
  def __init__(self, tilt):
    super(MoveHeadRight, self).__init__()
    self.tilt = tilt
  def run(self):
    commands.setWalkVelocity(0.0,0.0,0.0)
    commands.setHeadPanTilt(-core.DEG_T_RAD*85.0,self.tilt,1.0)
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 1.2:
      self.finish()

class TurnInPlace(Node):
  """Turn in place if ball not found"""
  def run(self):
    commands.setWalkVelocity(0.0,0.0,-0.4)
    if self.getTime() > 2.5:
      self.finish()

class LookDown(Node):
  """Search for the ball to the right"""
  def run(self):
    commands.setHeadPanTilt(0.0,-45.0,1.5)
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
    ####print("Elevation: %.3f\t" % elevation)
    commands.setHeadPanTilt(bearing, -elevation, 1.5)
    if dt == 0:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
    else:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral + self.k_t[2] *(bearing - self.theta_prev) / dt
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    
    if abs(bearing) >=0.3:
      # Control only the heading of the robot and not the velocity
      # print("Bearing only = %.5f, theta_cont = %.5f\n" %(bearing, theta_cont))
      commands.setWalkVelocity(0.0, 0.0, 0.4*np.sign(bearing))
    else:
      # Control both heading and velocity
      if abs(distance) >= 600.0:
        # print("Bearing = %.5f, distance = %.5f, theta_cont = %.5f\n" %(bearing, distance, theta_cont))
        commands.setWalkVelocity(1.0, 0.0, theta_cont)
      else:
        # print("theta_cont = %.5f, dist_cont = %.5f\n" %(theta_cont, dist_cont))
        commands.setWalkVelocity(dist_cont, 0.0, theta_cont)
    ####print("Bearing = %.5f, distance = %.5f\n" %(bearing, distance))
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current

class TurnAroundBall(Node):
  """Circle the ball while searching for the goal"""
  def __init__(self, ball, dist):
    super(TurnAroundBall, self).__init__()
    self.ball = ball
    self.k_t = (0.7, 0.01, 0.1)
    self.k_d = (0.001, 0.0001, 0.0001)
    self.dist = dist
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
    commands.setHeadPanTilt(bearing,-45.0,1.5)
    if dt == 0:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral
    else:
      theta_cont = self.k_t[0] * bearing + self.k_t[1] * self.theta_integral + self.k_t[2] *(bearing - self.theta_prev) / dt
      dist_cont = self.k_d[0] * (distance - self.dist) + self.k_d[1] * self.dist_integral + self.k_d[2] *(distance - self.dist_prev) / dt
    commands.setWalkVelocity(dist_cont, 0.6, theta_cont)
    self.theta_prev = bearing
    self.dist_prev = distance
    self.time_last = self.time_current
    if self.getTime() > 2.0:
      self.finish()

class Align(Node):
  """Turn around the ball to find the goal"""
  def __init__(self, ball, goal, dist, pan):
    super(Align, self).__init__()
    self.ball = ball
    self.goal = goal
    self.dist = dist
    self.pan = pan
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
    self.theta_integral_ball = self.theta_integral_ball + dt*(self.ball.visionBearing)
    self.theta_integral_delta = self.theta_integral_delta + dt*(self.ball.visionBearing - self.goal.visionBearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.visionDistance - self.dist)
    if abs(self.theta_integral_ball) >= 15.0:
      self.theta_integral_ball = 15.0*np.sign(self.theta_integral_ball)
    if abs(self.theta_integral_delta) >= 15.0:
      self.theta_integral_delta = 15.0*np.sign(self.theta_integral_delta)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    dt = 1.0/30.0
    self.calc_integral(dt)
    bearing_ball = self.ball.visionBearing
    bearing_goal = self.goal.visionBearing
    delta_bearing = bearing_ball - bearing_goal
    distance = self.ball.visionDistance
    elevation = core.RAD_T_DEG * self.ball.visionElevation
    commands.setHeadPanTilt(bearing_ball,self.pan,1.5)
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

class Stand(Node):
  def run(self):
    commands.stand()
    commands.setHeadPanTilt(0.0,0.0,1.5)

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
    self.theta_integral_ball = self.theta_integral_ball + dt*(self.ball.visionBearing - self.bearing)
    self.dist_integral = self.dist_integral + dt*(self.ball.visionDistance - self.dist)
    if abs(self.theta_integral_ball) >= 15.0:
      self.theta_integral_ball = 15.0*np.sign(self.theta_integral_ball)
    if abs(self.dist_integral) >= 100.0:
      self.dist_integral = 100.0*np.sign(self.dist_integral)

  def run(self):
    dt = 1.0/30.0
    self.calc_integral(dt)
    err_bearing = self.ball.visionBearing - self.bearing
    distance = self.ball.visionDistance
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

class Playing(LoopingStateMachine):
  def setup(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

    rdy = GetReady()
    moveHeadLeft = MoveHeadLeft(-15.0)
    moveHeadRight = MoveHeadRight(-45.0)
    turnInPlace = TurnInPlace()
    goToBall = GoToBall(ball, 0.0)

    moveHeadLeftGoal = MoveHeadLeft(0.0)
    moveHeadRightGoal = MoveHeadRight(0.0)
    turnAroundBall = TurnAroundBall(ball,200.0)
    lookDown = LookDown()
    align200 = Align(ball,goal,200.0, -15.0)  

    dribble = Align(ball,goal,-200.0, -20.0)
    wait = Stand()
    align50 = Align(ball,goal,50.0,-15.0)

    alignForKick = Align(ball,goal, 0.0, -30.0)
    positionForKick = PositionForKick(ball, 0.28, 140.0)

    stand = Stand()
    stand_again = Stand()
    kick = Kick()

    # Keep turning head and turning in place till ball is found and then go to ball
    self.add_transition(rdy,C,moveHeadLeft,C,moveHeadRight,C,turnInPlace,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,B(ball),goToBall)
    self.add_transition(moveHeadRight,B(ball),goToBall)
    self.add_transition(turnInPlace,B(ball),goToBall)
    self.add_transition(goToBall,NB(ball),moveHeadLeft)

    # After Robot reaches near the ball, maintain distance to ball and find the goal
    self.add_transition(goToBall,BD(ball,200.0),moveHeadLeftGoal,C,moveHeadRightGoal,C,lookDown,C,turnAroundBall,C,moveHeadLeftGoal)
    self.add_transition(moveHeadLeftGoal,GB(goal,ball),align200)
    self.add_transition(moveHeadRightGoal,GB(goal,ball),align200)
    self.add_transition(lookDown,GB(goal,ball),align200)
    self.add_transition(turnAroundBall,GB(goal,ball),align200)
    self.add_transition(align200,NGB(goal,ball),turnAroundBall)
    self.add_transition(turnAroundBall,NB(ball),rdy)

    # If the ball and goal are aligned with the robot, proceed to stopping, judging distance and dribbling/shooting
    self.add_transition(align200,A(ball,goal),dribble)
    self.add_transition(dribble,A(ball,goal,0.2).negation(),align50)
    self.add_transition(align50,A(ball,goal),dribble)
    self.add_transition(dribble,D(ball,goal,1300.0).negation(),wait)
    self.add_transition(wait,D(ball,goal,1300.0),dribble)

    # After it's dribbled, align between ball and goal again and then shift left
    self.add_transition(wait,T(1.0),alignForKick)
    self.add_transition(alignForKick, BD(ball,150.0), positionForKick)
    self.add_transition(positionForKick, BB(ball,0.28), stand)
    self.add_transition(stand, T(1.0), kick)
    self.add_transition(kick, C, stand_again)
    self.add_transition(stand_again, T(3.0), rdy)