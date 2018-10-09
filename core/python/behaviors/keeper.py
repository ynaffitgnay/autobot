"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
import task, util
import head
import math
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
        return pose.ToPoseMoveHead(pose=newPose, tilt=-15, time =0.1)

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
        return pose.ToPoseMoveHead(pose=newPose, tilt=-15, time=0.1)

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
        return pose.ToPoseMoveHead(pose=newPose, tilt=-15, time=0.1)

class Blocker(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setHeadPan(0.0, 1.0)
        # print("Ball distance: %f Ball bearing: %f\n" % (ball.distance,ball.bearing))
        vel_mag = math.sqrt(ball.relVel.x*ball.relVel.x + ball.relVel.y*ball.relVel.y)
        x_ball = ball.distance*math.cos(ball.bearing)/10.0
        y_ball = ball.distance*math.sin(ball.bearing)/10.0
        print("X: %f, Y: %f, Vmag: %f, Vx: %f, Vy: %f" % (x_ball, y_ball, vel_mag, ball.relVel.x, ball.relVel.y))
        if vel_mag > 3.0 and ball.relVel.x < 0.0 and ball.seen:
            # ball moving away from the robot, reset to regular position
            y_intercept = -ball.relVel.y*x_ball/ball.relVel.x + y_ball
            print("Y intercept: %f" % y_intercept)
            ## can do if xvel < yvel && xvel < 1, etc.
            ## maybe look at distance between robot and goal? 
            UTdebug.log(15, "Ball is close, blocking!")
            if y_intercept < -15.0:
                choice = "left"
            elif ball.bearing > 15.0:
                choice = "right"
            else:
                choice = "center"
            self.postSignal(choice)
        commands.setStiffness(cfgstiff.One)
        newPose = dict()
        newPose[core.LShoulderRoll] = 3
        newPose[core.LShoulderPitch] = -100
        newPose[core.LElbowRoll] = 0
        newPose[core.RShoulderRoll] = 3
        newPose[core.RShoulderPitch] = -100
        newPose[core.RElbowRoll] = 0
        print("Hands down\n")
        return pose.ToPoseMoveHead(pose=newPose, tilt=-15, time=0.1)


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
