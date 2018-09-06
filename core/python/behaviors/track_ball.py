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
from state_machine import Node, C, T, StateMachine

class Playing(StateMachine):
  class TrackBall(Node):
    def run(self,ballPose):
      pose.ToPoseMoveHead(ballPose,1.0)
      if self.getTime() > 1.0:
        memory.speech.say("moved my head")
        self.finish()

  class FindBall(Node):
    def run(self,ballPose):
      ballPose = ball.loc
      if self.getTime() > 1.0:
      postSignal(ballPose)
        # Light up ears? 
        self.finish()


  class Shutdown(Node):
    def run(self):
      head.moveHead()
      if self.getTime() > 2.0:
        memory.speech.say("turning off")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    findBall = self.FindBall()
    trackBall= self.TrackBall()
    shutdown = self.Shutdown()
    sit = pose.Sit()
    off = self.Off()
    while ball.seen
      ball = memory.world_objects.getObjPtr(core.WO_BALL)  
      self.trans(findBall,C,trackBall,S(ballPose))

