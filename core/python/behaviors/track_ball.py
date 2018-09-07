"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import memory
import core
import pose
import head
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, LoopingStateMachine

class Stand(Node):
  def run(self):
    commands.standStraight()
    if self.getTime() > 1.5:
      self.finish()

class DetectBall(Node):
  def __init__(self):
    super(DetectBall, self).__init__()
    self.old_bearing = 0.0
    self.old_elevation = 0.0
    self.elevation_filtered = 0.0
    self.bearing_filtered = 0.0
    self.alpha_b = 1.0
    self.alpha_e = 1.0
  def run(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      self.bearing_filtered = self.alpha_b*ball.visionBearing #+ (1-self.alpha_b)*self.bearing_filtered
      self.elevation_filtered = self.alpha_e*ball.visionElevation #+ (1-self.alpha_e)*self.elevation_filtered
      if (abs(self.bearing_filtered-self.old_bearing) <= 0.7):
        bearing = self.bearing_filtered
      else:
        bearing = self.old_bearing
      if (abs(self.elevation_filtered-self.old_elevation) <= 0.5):
        elevation = self.elevation_filtered
      else:
        elevation = self.old_elevation
      commands.setHeadPanTilt(bearing, core.RAD_T_DEG* elevation, 0.2)
      # print('Ball!')
      print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f' % (ball.visionBearing, ball.visionDistance, ball.visionElevation))
      self.old_bearing = bearing
      self.old_elevation = elevation
    else:
      commands.setHeadPanTilt(self.old_bearing, core.RAD_T_DEG* self.old_elevation, 0.2)
      print('No Ball!')
    if self.getTime() > 10.0:
      self.finish()

class Playing(LoopingStateMachine):
  def setup(self):
    stand = Stand()
    detect = DetectBall()
    self.trans(stand,C,detect,C,detect)