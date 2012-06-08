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
from dynamic_graph.sot.core.feature_position import FeaturePosition
from dynamic_graph.sot.core import Task
from dynamic_graph.sot.dynamics.tools import *

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

def pushTasks(op):
    solver.sot.push(robot.tasks['right-ankle'].name)
    solver.sot.push(robot.tasks['left-ankle'].name)
    solver.sot.push(robot.comTask.name)
    solver.sot.push(robot.tasks[op].name)

def removeTasks(op):
    solver.sot.remove(robot.tasks[op].name)
    solver.sot.remove(robot.tasks['left-ankle'].name)
    solver.sot.remove(robot.tasks['right-ankle'].name)
    solver.sot.remove(robot.comTask.name)

# Main.
#  Main loop
def generate():
    for i in xrange(1000):
        robot.device.increment(timeStep)

        if clt:
            clt.updateElementConfig(
                'hrp', robot.smallToFull(robot.device.state.value))


def setPosture(robot, posture):
    robot.device.set(posture)

    robot.dynamic.com.recompute(robot.device.state.time+1)
    robot.dynamic.Jcom.recompute(robot.device.state.time+1)
    robot.featureComDes.errorIN.value = robot.dynamic.com.value

    for op in robot.OperationalPoints:
        robot.dynamic.signal(op).recompute(robot.device.state.time+1)
        robot.dynamic.signal('J'+op).recompute(robot.device.state.time+1)
        robot.features[op].reference.value = robot.dynamic.signal(op).value

if __name__ == "__main__":
#op = 'right-wrist'
    op = 'custom'
    setupCustomOpPointTask('custom')
    reach(robot, op, 0.1, 0, 0)

    print op
    v = robot.dynamic.signal(op).value
    print v[0][3],v[1][3],v[2][3]
    pushTasks(op)
    generate()
    removeTasks(op)
    v = robot.dynamic.signal(op).value
    print v[0][3],v[1][3],v[2][3]
    
#    op = 'left-wrist'
    op = 'custom'
    robot.dynamic.setCustomOpPoint(op, 34, (0,0,-0.2))
    setPosture(robot, robot.halfSitting)
    reach(robot, op, 0.0, 0, 0.1)

    print op
    v = robot.dynamic.signal(op).value
    print v[0][3],v[1][3],v[2][3]
    pushTasks(op)
    generate()
    removeTasks(op)
    v = robot.dynamic.signal(op).value
    print v[0][3],v[1][3],v[2][3]

