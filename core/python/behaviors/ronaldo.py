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
import cfgstiff
from task import Task
from state_machine import Node, C, T, S, LoopingStateMachine, StateMachine, EventNode, Event, NegationEvent

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

class ObjectDistance(Event):
  """Event that fires if Ball is close enough"""
  def __init__(self, obj, dist):
    super(ObjectDistance, self).__init__()
    self.object = obj
    self.dist = dist
  def ready(self):
    return (self.object.seen and (self.object.distance < self.dist))

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
  # def setup(self):
  #   ball = memory.world_objects.getObjPtr(core.WO_BALL)
  #   goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)

  #   rdy = GetReady()
  #   lookStraight = MoveHead(0.0,-10.0,1.0)
  #   moveHeadLeft = MoveHead(85.0,-15.0,1.0)
  #   moveHeadRight = MoveHead(-85.0,-45.0,1.0)
  #   turnInPlace = TurnInPlace()
  #   goToBall = GoToBall(ball, 0.0)


  #   moveHeadLeftGoal = MoveHead(85.0,0.0,1.0)
  #   moveHeadRightGoal = MoveHead(-85.0,0.0,1.0)
  #   alignm200Left = Align(ball,goal,-200.0, -15.0)
  #   alignm200Right = Align(ball,goal,-200.0, -15.0)
  #   dribble = Align(ball,goal,-300.0, -12.5)

  #   # dribble = Align(ball,goal,-200.0, -20.0)
  #   align50 = Align(ball,goal,50.0,-15.0)

  #   alignForKick = Align(ball,goal, 0.0, -30.0)
  #   positionForKick = PositionForKick(ball, 0.28, 140.0)

  #   kick = Kick()

  #   # Keep turning head and turning in place till ball is found and then go to ball
  #   self.add_transition(rdy,C,lookStraight,C,moveHeadLeft,C,moveHeadRight,C,turnInPlace,C,moveHeadLeft)
  #   self.add_transition(lookStraight,O(ball),goToBall)
  #   self.add_transition(moveHeadLeft,O(ball),goToBall)
  #   self.add_transition(moveHeadRight,O(ball),goToBall)
  #   self.add_transition(turnInPlace,O(ball),goToBall)
  #   self.add_transition(goToBall,NO(ball),moveHeadLeft)

  #   # Not to shoot from the front, determine whether the robot should dribble the ball away from the center


  #   # After Robot reaches near the ball, maintain distance to ball and find the goal
  #   self.add_transition(goToBall,C,lookUp,C,moveHeadLeftBeacon,C,moveHeadRightBeacon,C,moveHeadLeftBeacon)

  #   self.add_transition(lookUp,O(pb),turnAroundBallLeft)
  #   self.add_transition(lookUp,O(bp),turnAroundBallLeft)
  #   self.add_transition(lookUp,O(yb),turnAroundBallRight)
  #   self.add_transition(lookUp,O(by),turnAroundBallRight)

  #   self.add_transition(moveHeadLeftBeacon,O(pb),turnAroundBallLeft)
  #   self.add_transition(moveHeadLeftBeacon,O(bp),turnAroundBallLeft)
  #   self.add_transition(moveHeadLeftBeacon,O(yb),turnAroundBallRight)
  #   self.add_transition(moveHeadLeftBeacon,O(by),turnAroundBallRight)

  #   self.add_transition(moveHeadRightBeacon,O(pb),turnAroundBallLeft)
  #   self.add_transition(moveHeadRightBeacon,O(bp),turnAroundBallLeft)
  #   self.add_transition(moveHeadRightBeacon,O(yb),turnAroundBallRight)
  #   self.add_transition(moveHeadRightBeacon,O(by),turnAroundBallRight)

  #   self.add_transition(turnAroundBallRight,O(goal,ball),alignm200Right)
  #   self.add_transition(turnAroundBallLeft,O(goal,ball),alignm200Left)
  #   self.add_transition(alignm200Right,NO(goal,ball),turnAroundBallRight)
  #   self.add_transition(alignm200Left,NO(goal,ball),turnAroundBallLeft)

  #   # If the ball and goal are aligned with the robot, proceed to stopping, judging distance and dribbling/shooting
  #   self.add_transition(alignm200Right,A(ball,goal,0.2,0.5),dribble)
  #   self.add_transition(alignm200Left,A(ball,goal,0.2,0.5),dribble)
  #   # self.add_transition(positionForKick, BB(ball,0.28), stand)
  #   # self.add_transition(dribble,A(ball,goal,0.2).negation(),align50)
  #   # self.add_transition(align50,A(ball,goal),dribble)
  #   self.add_transition(dribble,D(ball,goal,800.0),stand)
  #   self.add_transition(stand,C,positionForKick)
  #   self.add_transition(dribble,A(ball,goal,0.2,0.2).negation(),align50)
  #   self.add_transition(align50,A(ball,goal,0.2,0.2),dribble)
  #   # self.add_transition(wait,D(ball,goal,1300.0),dribble)

  #   # # After it's dribbled, align between ball and goal again and then shift left
  #   # self.add_transition(wait,T(1.0),alignForKick)
  #   # self.add_transition(alignForKick, OD(ball,150.0), positionForKick)
  #   self.add_transition(positionForKick, BB(ball,0.28), stand_again)
  #   # self.add_transition(stand, T(1.0), kick)
  #   # self.add_transition(kick, C, stand_again)
  #   # self.add_transition(stand_again, T(3.0), rdy)

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
    
    des_bearing = 0.1
    lookUp = MoveHead(0.0,-5.0,1.0)
    lookDown = MoveHead(0.0,-45.0,1.5)
    align200 = Align(ball,goal,200.0, -15.0,des_bearing)

    dribble = Align(ball,goal,-350.0, -20.0,0.0)
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
    self.add_transition(dribble,D(ball,goal,900.0),wait)
    self.add_transition(wait,D(ball,goal,900.0).negation(),dribble)
    self.add_transition(dribble,OD(line,200.0),alignForKick)

    # After it's dribbled, align between ball and goal again and then shift left
    self.add_transition(wait,T(1.0),alignForKick)
    self.add_transition(alignForKick, OD(ball,150.0), positionForKick)
    self.add_transition(positionForKick, BB(ball,0.28), stand)
    self.add_transition(stand, T(1.0), kick, C, stand_again, T(3.0), rdy)