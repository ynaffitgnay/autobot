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
        
        commands.setStiffness(cfgstiff.One)
        newPose = dict()
        newPose[core.RShoulderRoll] = 70
        newPose[core.RShoulderPitch] = -95
        newPose[core.RElbowRoll] = -1

        # Reset Left arm to neutral
        newPose[core.LShoulderRoll] = 3
        newPose[core.LShoulderPitch] = -100
        newPose[core.LElbowRoll] = 0
        return pose.ToPoseMoveHead(pose=newPose, tilt=-15, time = 1.0)

class BlockRight(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")

        commands.setStiffness(cfgstiff.One)
        newPose = dict()
        newPose[core.LShoulderRoll] = 70
        newPose[core.LShoulderPitch] = -95
        newPose[core.LElbowRoll] = -1
        
        # Reset right arm to neutral
        newPose[core.RShoulderRoll] = 3
        newPose[core.RShoulderPitch] = -100
        newPose[core.RElbowRoll] = 0
        return pose.ToPoseMoveHead(pose=newPose, tilt=-15, time=1.0)

class BlockCenter(Node):
    def run(self):
        UTdebug.log(15, "Blocking center")
        
        commands.setStiffness(cfgstiff.One)
        newPose = dict()
        newPose[core.LShoulderRoll] = 9.95
        newPose[core.LShoulderPitch] = -3.85
        newPose[core.LElbowRoll] = -1
        newPose[core.RShoulderRoll] = 9.95
        newPose[core.RShoulderPitch] = -3.85
        newPose[core.RElbowRoll] = -1
        return pose.ToPoseMoveHead(pose=newPose, tilt=-15, time=1.0)

class Blocker(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setHeadPan(ball.bearing, 0.1)
        print("Ball distance: %f Ball bearing: %f X,Y vel: %f, %f" % (ball.distance,ball.bearing,ball.relVel.x,ball.relVel.y))
        if ball.distance < 500:
            # ball moving away from the robot, reset to regular position
            if (ball.relVel.x >= 0):
                newPose = dict()
                newPose[core.LShoulderRoll] = 3
                newPose[core.LShoulderPitch] = -100
                newPose[core.LElbowRoll] = 0
                newPose[core.RShoulderRoll] = 3
                newPose[core.RShoulderPitch] = -100
                newPose[core.RElbowRoll] = 0
                pose.ToPoseMoveHead(pose=newPose, tilt=-15, time=1.0)
                return
            ## can do if xvel < yvel && xvel < 1, etc.
            ## maybe look at distance between robot and goal? 
            UTdebug.log(15, "Ball is close, blocking!")
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
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(2), blocker)
