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

class Stand(Node):
  def run(self):
    commands.stand()
    commands.setHeadTilt(-30.0)
    if self.getTime() > 1.5:
      self.finish()

class TrackBall(Node):
  """Controller node for tracking the ball"""
  def run(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    bearing = ball.visionBearing
    elevation = ball.visionElevation
    print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f\t Location: [%d,%d]\n' % (ball.visionBearing, ball.visionDistance, ball.visionElevation,ball.imageCenterX, ball.imageCenterY))
    commands.setHeadPanTilt(bearing, -30.0, 0.1)

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
    print('Seen: %d \t Top: %d\t Location: [%d,%d]\n' % (ball.seen, ball.fromTopCamera, ball.imageCenterX, ball.imageCenterY))
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