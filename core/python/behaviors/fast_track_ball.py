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

def NB(ball=None):
  """No ball found"""
  return NegationEvent(BallSeen(ball))

class BallNode(EventNode):
  def __init__(self, node, ball):
    super(BallNode, self).__init__(node)
    self.ball = ball

class Stand(Node):
  def run(self):
    commands.standStraight()
    if self.getTime() > 1.5:
      self.finish()

class TrackBall(Node):
  """Controller node for tracking the ball"""
  def run(self):
    print('TrackBall')
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    bearing = ball.bearing
    elevation = ball.elevation
    # print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f' % (ball.bearing, ball.distance, ball.elevation))
    commands.setHeadPanTilt(bearing, core.RAD_T_DEG* elevation, 0.2)

class MoveHeadLeft(Node):
  """Search for the ball to the left"""
  def run(self):
    commands.setHeadPan(-math.pi/2,1.5)
    if self.getTime() > 2.5:
      self.finish()

class MoveHeadRight(Node):
  """Search for the ball to the right"""
  def run(self):
    commands.setHeadPan(math.pi/2,1.5)
    if self.getTime() > 2.5:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    stand = Stand()
    track = TrackBall()
    moveHeadLeft = MoveHeadLeft()
    moveHeadRight = MoveHeadRight()
    sit = pose.Sit()
    self.add_transition(stand,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,C,moveHeadRight)
    self.add_transition(moveHeadRight,C,moveHeadLeft)
    self.add_transition(moveHeadLeft,B(ball),track)
    self.add_transition(moveHeadRight,B(ball),track)
    self.add_transition(track,NB(ball),moveHeadLeft)