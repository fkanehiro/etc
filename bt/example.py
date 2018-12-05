#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees.demos.lifecycle
   :func: command_line_argument_parser
   :prog: py-trees-demo-behaviour-lifecycle

.. image:: images/lifecycle.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import time
import cv2
import numpy as np
import math
import Series
import random

import py_trees.console as console
from py_trees.blackboard import Blackboard

##############################################################################
# Classes
##############################################################################


def description():
    content = "Demonstrates a typical day in the life of a behaviour.\n\n"
    content += "This behaviour will count from 1 to 3 and then reset and repeat. As it does\n"
    content += "so, it logs and displays the methods as they are called - construction, setup,\n"
    content += "initialisation, ticking and termination.\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Behaviour Lifecycle".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    return argparse.ArgumentParser(description=description(),
                                   epilog=epilog(),
                                   formatter_class=argparse.RawDescriptionHelpFormatter,
                                   )

class Pose2D:
    def __init__(self, x=0, y=0, th=0):
        self.x = x
        self.y = y
        self.th = th

def distance(pose1, pose2):
    dx = pose1.x - pose2.x
    dy = pose1.y - pose2.y
    return math.sqrt(dx*dx+dy*dy)

def direction(this, target):
    dx = target.x - this.x
    dy = target.y - this.y
    th = math.atan2(dy,dx)
    dth = th - this.th
    while dth > math.pi:
        dth -= 2*math.pi
    while dth < -math.pi:
        dth += 2*math.pi
    return dth

class BallFound(py_trees.behaviour.Behaviour):
    def __init__(self, name="BallFound"):
        super(BallFound, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        ball = self.blackboard.get("ball")
        th = direction(robot, ball)
        d = distance(robot, ball)
        if math.fabs(th) > 1 and d > 10:
            new_status = py_trees.common.Status.FAILURE
        else:
            new_status = py_trees.common.Status.SUCCESS
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class BallClose(py_trees.behaviour.Behaviour):
    def __init__(self, name="BallClose"):
        super(BallClose, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        ball = self.blackboard.get("ball")
        d = distance(robot, ball)
        if d < 10:
            new_status = py_trees.common.Status.SUCCESS
        else:
            new_status = py_trees.common.Status.FAILURE
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class BallGrasped(py_trees.behaviour.Behaviour):
    def __init__(self, name="BallGrasped"):
        super(BallGrasped, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.SUCCESS
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class BinFound(py_trees.behaviour.Behaviour):
    def __init__(self, name="BinFound"):
        super(BinFound, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        bin = self.blackboard.get("bin")
        th = direction(robot, bin)
        if math.fabs(th) > 1:
            new_status = py_trees.common.Status.FAILURE
        else:
            new_status = py_trees.common.Status.SUCCESS
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class BinClose(py_trees.behaviour.Behaviour):
    def __init__(self, name="BinClose"):
        super(BinClose, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        bin = self.blackboard.get("bin")
        d = distance(robot, bin)
        if d < 10:
            new_status = py_trees.common.Status.SUCCESS
        else:
            new_status = py_trees.common.Status.FAILURE
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class FindBall(py_trees.behaviour.Behaviour):
    def __init__(self, name="FindBall"):
        super(FindBall, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        robot.th += 0.5
        new_status = py_trees.common.Status.RUNNING
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class ApproachBall(py_trees.behaviour.Behaviour):
    def __init__(self, name="ApproachBall"):
        super(ApproachBall, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        ball = self.blackboard.get("ball")
        dx = ball.x - robot.x
        dy = ball.y - robot.y
        d = math.sqrt(dx*dx+dy*dy)
        robot.x += int(10/d*dx)
        robot.y += int(10/d*dy)
        new_status = py_trees.common.Status.RUNNING
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class GraspBall(py_trees.behaviour.Behaviour):
    def __init__(self, name="GraspBall"):
        super(GraspBall, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        new_status = py_trees.common.Status.SUCCESS
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class FindBin(py_trees.behaviour.Behaviour):
    def __init__(self, name="FindBin"):
        super(FindBin, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        robot.th += 0.5
        new_status = py_trees.common.Status.RUNNING
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

class ApproachBin(py_trees.behaviour.Behaviour):
    def __init__(self, name="ApproachBin"):
        super(ApproachBin, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = Blackboard()

    def setup(self, unused_timeout=15):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        robot = self.blackboard.get("robot")
        ball = self.blackboard.get("ball")
        bin = self.blackboard.get("bin")
        dx = bin.x - robot.x
        dy = bin.y - robot.y
        d = math.sqrt(dx*dx+dy*dy)
        robot.x += int(10/d*dx)
        robot.y += int(10/d*dy)
        ball.x += int(10/d*dx)
        ball.y += int(10/d*dy)
        new_status = py_trees.common.Status.RUNNING
        return new_status

    def terminate(self, new_status):
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

        
def create_tree():
    root = Series.Series("Series 1")
    selector1 = py_trees.composites.Selector("Selector 1")
    ball_found = BallFound("BallFound")
    find_ball = FindBall("FindBall")
    selector2 = py_trees.composites.Selector("Selector 2")
    ball_close = BallClose("BallClose")
    approach_ball = ApproachBall("ApproachBall")
    selector3 = py_trees.composites.Selector("Selector 3")
    ball_grasped = BallGrasped("BallGrasped")
    grasp_ball = GraspBall("GraspBall")
    selector4 = py_trees.composites.Selector("Selector 4")
    bin_found = BinFound("BinFound")
    find_bin = FindBin("FindBin")
    selector5 = py_trees.composites.Selector("Selector 5")
    bin_close = BinClose("BinClose")
    approach_bin = ApproachBin("ApproachBin")
    root.add_children([selector1, selector2, selector3, selector4, selector5])
    selector1.add_children([ball_found, find_ball])
    selector2.add_children([ball_close, approach_ball])
    selector3.add_children([ball_grasped, grasp_ball])
    selector4.add_children([bin_found, find_bin])
    selector5.add_children([bin_close, approach_bin])
    return root
    
##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    command_line_argument_parser().parse_args()

    print(description())

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    tree = create_tree()
    
    tree.setup(timeout=15)

    robot = Pose2D(250,250);
    ball = Pose2D(int(random.random()*500),
                  int(random.random()*500))
    bin = Pose2D(int(random.random()*500),
                 int(random.random()*500))

    blackboard = Blackboard()
    blackboard.set("robot", robot)
    blackboard.set("ball", ball)
    blackboard.set("bin", bin)

    try:
        while tree.status != py_trees.common.Status.SUCCESS:
            print("tick")
            tree.tick_once()

            img = np.full((500,500,3), 0, dtype=np.uint8)
            cv2.circle(img, (robot.x,robot.y), 10, (0,0,255), thickness=-1)
            cv2.line(img, (robot.x, robot.y),
                     (robot.x + int(50*math.cos(robot.th+1.0)),
                      robot.y + int(50*math.sin(robot.th+1.0))),
                     (0,0,255), 1, cv2.LINE_AA)
            cv2.line(img, (robot.x, robot.y),
                     (robot.x + int(50*math.cos(robot.th-1.0)),
                      robot.y + int(50*math.sin(robot.th-1.0))),
                     (0,0,255), 1, cv2.LINE_AA)
            cv2.circle(img, (ball.x,ball.y), 10, (0,255,0), thickness=-1)
            cv2.circle(img, (bin.x,bin.y), 10, (255,0,0), thickness=-1)
            cv2.imshow('image', img)
            cv2.waitKey(100)

            if random.random() < 0.05:
                ball.x = int(random.random()*500)
                ball.y = int(random.random()*500)
        print("\n")
    except KeyboardInterrupt:
        print("")
        pass

if __name__ == '__main__':
    main()
