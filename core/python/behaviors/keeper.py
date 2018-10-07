"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
import task, util
import head
from task import Task, MultiTask
import cfgpose, cfgstiff
import pose
#from pose import ToPoseMoveHead
from memory import walk_request, walk_response, kick_request, joint_commands, behavior_mem, joint_angles
from state_machine import Node, S, T, LoopingStateMachine
import UTdebug


class BlockLeft(Node):
    def run(self):
        UTdebug.log(15, "Blocking left")
        print("Left")
        #newPose = util.deepcopy(cfgpose.sittingPoseV3)
        newPose = dict()
        newPose[core.LShoulderRoll] = 70
        newPose[core.LElbowYaw] = -85
        return pose.ToPoseMoveHead(tilt = 0.0, pose = newPose)

class BlockRight(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")
        print("Right")
        #newPose = util.deepcopy(cfgpose.sittingPoseV3)
        newPose = dict()
        newPose[core.RShoulderRoll] = 70
        newPose[core.RElbowYaw] = -85
        return pose.ToPoseMoveHead(tilt = 0.0, pose = newPose)

class BlockCenter(Node):
    def run(self):
        UTdebug.log(15, "Blocking center")


class Blocker(Node):
    def run(self):
        newPose = dict()
        newPose[core.LShoulderRoll] = 0
        newPose[core.LElbowYaw] = 0
        newPose[core.RShoulderRoll] = 0
        newPose[core.RElbowYaw] = 0
        ball = mem_objects.world_objects[core.WO_BALL]
        #commands.setHeadPan(ball.bearing, 0.1)
        if ball.distance < 500:
            UTdebug.log(15, "Ball is close, blocking!")
            print("Ball distance: %f Ball bearing: %f X,Y vel: %f, %f" % (ball.distance,ball.bearing,ball.relVel.x,ball.relVel.y))
            if ball.bearing > 30 * core.DEG_T_RAD:
                choice = "left"
            elif ball.bearing < -30 * core.DEG_T_RAD:
                choice = "right"
            else:
                choice = "center"
            self.postSignal(choice)


class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": BlockLeft(),
                  "right": BlockRight(),
                  "center": BlockCenter()
                  }
        sit = pose.Sit()
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(2), sit, T(0.5), blocker)
