#!/usr/bin/env python3

from __future__ import division
import rospy
from geometry_msgs.msg import Twist


from pygame.locals import *
import pygame
import sys
import os


os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

class joystick_handler(object):
    def __init__(self, id):
        self.id = id
        self.joy = pygame.joystick.Joystick(id)
        self.name = self.joy.get_name()
        self.joy.init()
        self.numaxes = self.joy.get_numaxes()

        self.axis = []
        for i in range(self.numaxes):
            self.axis.append(self.joy.get_axis(i))


class joystick_controller(object):
    class program:
        "Program metadata"
        name = "Remote Control Joystick"
        version = "1.0.0"
        author = "TOURE Vassindou && ZADI Jonathan" 
        description = "Based on denilsonsa/pygame-joystick-test"
        nameversion = name + " " + version


    def init(self):
        pygame.init()
        # self.clock = pygame.time.Clock()
        self.joycount = pygame.joystick.get_count()
        if self.joycount == 0:
            #print(
            #    "This program only works with at least one joystick plugged in. No joysticks were detected.")
            self.quit(1)
        self.joy = []
        for i in range(self.joycount):
            self.joy.append(joystick_handler(i))


    def run(self):
        #print(self.program.nameversion)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('thrustmaster_node', anonymous=True)
        rate = rospy.Rate(100) # 10hz
        cmd = Twist()
        speed = 0
        while True:
            for event in [pygame.event.wait(), ] + pygame.event.get():
                if event.type == QUIT:
                    self.quit()
                elif event.type == KEYDOWN and event.key in [K_ESCAPE, K_q]:
                    self.quit()
                elif event.type == JOYAXISMOTION:
                    self.joy[event.joy].axis[event.axis] = event.value
                    axis = event.axis
                    value = event.value
                    if axis == 1:
                        if(abs(value) < 0.09):
                            cmd.linear.x = 0
                        else:
                            cmd.linear.x = value * speed
                    elif axis == 2:
                        if(abs(value) < 0.09):
                            cmd.angular.z = 0
                        else:
                            cmd.angular.z = value * speed
                    elif axis == 3:
                        speed = -1 * (1 - value)
                    rospy.loginfo(cmd)
                    pub.publish(cmd)
                    #rate.sleep()
                    
                    
    def quit(self, status=0):
        pygame.quit()
        sys.exit()
        
        
if __name__ == "__main__":
    program = joystick_controller()
    program.init()
    program.run()