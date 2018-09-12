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
from state_machine import Node, C, S, T, LoopingStateMachine

class Stand(Node):
  def run(self):
    commands.standStraight()
    if self.getTime() > 1.5:
      self.finish()

class SearchBall(Node):
  def __init__(self):
    super(SearchBall, self).__init__()
    self.head_pos = False
  def run(self):
    print('SearchBall')
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    commands.setHeadTilt(-30.0)
    if ball.seen:
      print('Ball!')
      self.postSignal('seen')
    else:
      print('No Ball!')
      if self.head_pos:
        commands.setHeadPan(-math.pi/2,1.0)
        self.head_pos = False
      else:
        commands.setHeadPan(math.pi/2,1.0)
        self.head_pos = True
      if self.getTime() > 1.5:
        self.postSignal('notSeen')

class TrackBall(Node):
  # def __init__(self):
  #   super(TrackBall, self).__init__()
  #   self.kp_pan = 1.0
  #   self.ki_pan = 0.1
  #   self.kd_pan = 0.1
  #   self.kp_tilt = 1.0
  #   self.ki_tilt = 0.1
  #   self.kd_tilt = 0.1
  #   self.old_pan = 0.0
  #   self.int_pan = 0.0
  #   self.d_dt_pan = 0.0
  #   self.old_tilt = -core.DEG_T_RAD*22.0
  #   self.int_tilt = -core.DEG_T_RAD*22.0
  #   self.d_dt_tilt = 0.0
  #   self.e_pan = 0.0
  #   self.e_tilt = 0.0
      # self.e_pan = core.joint_values[core.HeadYaw] - ball.bearing
      # self.e_tilt = core.joint_values[core.HeadPitch] - ball.elevation
      # self.int_pan = self.int_pan + self.e_pan
      # self.int_tilt = self.int_tilt + self.e_tilt
  def run(self):
    print('TrackBall')
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
      bearing = ball.bearing
      elevation = ball.elevation
      print('Ball!\t Bearing: %f \t Distance: %f\t Elevation: %.8f' % (ball.bearing, ball.distance, ball.elevation))
      commands.setHeadPanTilt(bearing, core.RAD_T_DEG* elevation, 0.2)
    else:
      print('No Ball!')
      self.postSignal('notSeen')


class Playing(LoopingStateMachine):
  def setup(self):
    stand = Stand()
    search = SearchBall()
    track = TrackBall()
    self.add_transition(stand,C,search)
    self.add_transition(search,S('notSeen'),search)
    self.add_transition(search,S('seen'),track)
    self.add_transition(track,S('seen'),track)
    self.add_transition(track,S('notSeen'),search)