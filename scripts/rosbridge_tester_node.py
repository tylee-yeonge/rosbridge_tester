#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from rosbridge_tester.rosbridge_tester import RosbridgeTester

if __name__=='__main__':
    print("Start Rosbridge Tester")
    rospy.init_node("rosbridge_tester", anonymous=False)
    
    rosbridge_tester = RosbridgeTester()
    rospy.spin()
    
