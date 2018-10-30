"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import core
import pose
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, StateMachine

class Stand(Node):
  def __init__(self, ball, goal):
    super(Stand, self).__init__()
    self.ball = ball
    self.goal = goal

  def run(self):
    commands.setHeadPanTilt(0.0,-5.0,1.0)
    commands.stand()
    print("ball x: %f, ball y: %f, goal x: %f, goal y: %f" % (self.ball.loc.x,self.ball.loc.y,self.goal.loc.x,self.goal.loc.y))
    # print("ball bearing: %.3f\n" % self.ball.visionBearing)
    # if self.getTime() > 1.5:
    #     self.finish()

class Playing(StateMachine):
  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    goal = memory.world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
    stand = Stand(ball, goal)
    sit = pose.Sit()
    off = self.Off()
    self.trans(sit, C, stand, C, stand)