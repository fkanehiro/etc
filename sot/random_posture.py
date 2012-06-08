#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

import sys

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.dynamics.solver import Solver
from dynamic_graph.sot.core import FeatureGeneric, Task, RobotSimu
from dynamic_graph.sot.motion_planner import PostureError
from dynamic_graph.sot.core.feature_joint_limits import FeatureJointLimits
from dynamic_graph.sot.core.feature_position import FeaturePosition
from dynamic_graph.sot.core import Task
from numpy.linalg import norm

#robot = Hrp2Laas("robot")
solver = Solver(robot)
timeStep = 0.05
#clt = None

postureTask = None
postureError = None

# create custom operational point
def setupCustomOpPointTask(op):
    robot.dynamic.createCustomOpPoint(op, 27, (0,0,-0.2))
    robot.dynamic.signal(op).recompute(0)
    robot.dynamic.signal('J'+op).recompute(0)
    robot.features[op] = \
        FeaturePosition(robot.name + '_feature_' + op,
                        robot.dynamic.signal(op),
                        robot.dynamic.signal('J' + op),
                        robot.dynamic.signal(op).value)
    robot.tasks[op] = Task(robot.name + '_task_' + op)
    robot.tasks[op].add(robot.name + '_feature_' + op)
    robot.tasks[op].controlGain.value = .2
    robot.OperationalPoints.append(op)

def setupJointLimitTask():
    featureJl = FeatureJointLimits('featureJl')
    featureJl.actuate()
    dyn = robot.dynamic
    plug(dyn.signal('position'), featureJl.signal('joint'))
    plug(dyn.signal('upperJl'), featureJl.signal('upperJl'))
    plug(dyn.signal('lowerJl'), featureJl.signal('lowerJl'))
    taskJl = Task('taskJl')
    taskJl.add('featureJl')
    taskJl.signal('controlGain').value = .1

def setupPostureTask():
    global postureTask, postureError, postureFeature
    initialGain = 0.1

    postureTask = Task(robot.name + '_posture')
    postureFeature = FeatureGeneric(robot.name + '_postureFeature')
    postureError = PostureError('PostureError')

    posture = list(robot.halfSitting)
    posture[6+17] -= 1
    posture[6+24] += 1
#    postureError.setPosture(tuple(posture))
    plug(robot.device.state, postureError.state)
    plug(postureError.error, postureFeature.errorIN)
    postureFeature.jacobianIN.value = computeJacobian()
    postureTask.add(postureFeature.name)
    postureTask.controlGain.value = initialGain

def oneVector(i):
    r = [0.,] * 36
    r[i] = 1.
    return tuple(r)

def computeJacobian():
    j = []
    for i in xrange(36):
        if i >= 6 + 2 * 6: # waist,neck,arms
            j.append(oneVector(i))
    return tuple(j)

def computeDesiredValue():
    e = list(robot.halfSitting)
    e[23] -= 1.
    e[25] -= 1.
    e[30] += 1.
    e[32] -= 1.
    return e

# Push tasks
def pushTasksForPosture():
    solver.sot.push(robot.tasks['right-ankle'].name)
    solver.sot.push(robot.tasks['left-ankle'].name)
    solver.sot.push(robot.comTask.name)
#    solver.sot.push('taskJl')
    solver.sot.push(postureTask.name)

def removeTasksForPosture():
    solver.sot.remove(postureTask.name)
#    solver.sot.remove('taskJl')
    solver.sot.remove(robot.comTask.name)
    solver.sot.remove(robot.tasks['left-ankle'].name)
    solver.sot.remove(robot.tasks['right-ankle'].name)

# Push tasks
def pushTasksForAvoidance(op):
    solver.sot.push(robot.tasks['right-ankle'].name)
    solver.sot.push(robot.tasks['left-ankle'].name)
    solver.sot.push(robot.comTask.name)
    solver.sot.push(robot.tasks[op].name)

def removeTasksForAvoidance(op):
    solver.sot.remove(robot.tasks[op].name)
    solver.sot.remove(robot.tasks['left-ankle'].name)
    solver.sot.remove(robot.tasks['right-ankle'].name)
    solver.sot.remove(robot.comTask.name)

def setPosture(robot, posture):
    robot.device.set(posture)

    robot.dynamic.com.recompute(robot.device.state.time+1)
    robot.dynamic.Jcom.recompute(robot.device.state.time+1)
    robot.featureComDes.errorIN.value = robot.dynamic.com.value

    for op in robot.OperationalPoints:
        robot.dynamic.signal(op).recompute(robot.device.state.time+1)
        robot.dynamic.signal('J'+op).recompute(robot.device.state.time+1)
        robot.features[op].reference.value = robot.dynamic.signal(op).value

# Main.
#  Main loop
def generate(stopthd = 0.02):
    c = 0
    while 1:
        c+=1
        robot.device.increment(timeStep)
        if clt:
            clt.updateElementConfig(
                'hrp', robot.smallToFull(robot.device.state.value))
#        print '[',len(postureFeature.errorIN.value),']',postureFeature.errorIN.value
#        print postureFeature.errorIN.value[5+7]
#        print postureFeature.jacobianIN.value
#        break;
#        print solver.sot.signal('control').value
        if norm(solver.sot.signal('control').value) < stopthd:
            break
    print c,"frames"
    print "final posture = ",robot.device.state.value

setupPostureTask()
#setupJointLimitTask()

target = computeDesiredValue()

print "target[",len(target),"]=",target

def setup(initial, target):
    pushTasksForPosture()
    setPosture(robot, initial)
    postureError.setPosture(tuple(target))

def cleanup():
    removeTasksForPosture()
    
setup(robot.halfSitting, target)
generate()
cleanup()

#setup(robot.halfSitting, target)
#generate()
#cleanup()

print "finish"

