#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot)


    #### YOUR CODE HERE ####
    handles = []
    # red lines around the tables width = 0.6 length = 1.2
    # (3.5, 1.2)
    handles.append(env.drawlinestrip(points=array(((3.2,0.6,0.74), (3.2,1.8,0.74), (3.8,1.8,0.74), (3.8,0.6,0.74), (3.2,0.6,0.74))),
                                     linewidth=5.0,
                                     colors=array(((1,0,0), (1,0,0), (1,0,0), (1,0,0), (1,0,0)))))
    # (3.5, -1.2)
    handles.append(env.drawlinestrip(points=array(((3.2,-1.8,0.74), (3.2,-0.6,0.74), (3.8,-0.6,0.74), (3.8,-1.8,0.74), (3.2,-1.8,0.74))),
                                     linewidth=5.0,
                                     colors=array(((1,0,0), (1,0,0), (1,0,0), (1,0,0), (1,0,0)))))
    # (-2.3, 0.3)
    handles.append(env.drawlinestrip(points=array(((-1.7,0.6,0.74), (-1.7,0,0.74), (-2.9,0,0.74), (-2.9,0.6,0.74), (-1.7,0.6,0.74))),
                                     linewidth=5.0,
                                     colors=array(((1,0,0), (1,0,0), (1,0,0), (1,0,0), (1,0,0)))))
    # (-2.3, -0.3)
    handles.append(env.drawlinestrip(points=array(((-1.7,0,0.74), (-1.7,-0.6,0.74), (-2.9,-0.6,0.74), (-2.9,0,0.74), (-1.7,0,0.74))),
                                     linewidth=5.0,
                                     colors=array(((1,0,0), (1,0,0), (1,0,0), (1,0,0), (1,0,0)))))
    # (-1.1, -0.3)
    handles.append(env.drawlinestrip(points=array(((-0.5,0,0.74), (-0.5,-0.6,0.74), (-1.7,-0.6,0.74), (-1.7,0,0.74), (-0.5,0,0.74))),
                                     linewidth=5.0,
                                     colors=array(((1,0,0), (1,0,0), (1,0,0), (1,0,0), (1,0,0)))))
    # (-1.1, 0.3)
    handles.append(env.drawlinestrip(points=array(((-0.5,0.6,0.74), (-0.5,0,0.74), (-1.7,0,0.74), (-1.7,0.6,0.74), (-0.5,0.6,0.74))),
                                     linewidth=5.0,
                                     colors=array(((1,0,0), (1,0,0), (1,0,0), (1,0,0), (1,0,0)))))
    # 35 blue circles
    R = 5
    theta = 2 * pi / 35
    for i in range(35):
        handles.append(env.plot3(points=array(((R * sin(i * theta), R * cos(i * theta), 0))),
                                 pointsize=0.1,
                                 colors=array(((0,0,1))),
                                 drawstyle=1))
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
