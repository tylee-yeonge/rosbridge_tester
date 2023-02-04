#!/usr/bin/env python
import rospy

class TimeChecker():
    def __init__(self, timeout):
        self.timeout = timeout
        self.check_time = int(rospy.get_time())
        self.save_time = int(rospy.get_time())        

    def set_timeout(self, timeout):
        self.timeout = timeout

    def set_time(self, result):
        if result:
            self.check_time = rospy.get_time()            
        else:
            self.check_time = rospy.get_time()
            self.save_time = rospy.get_time()        
        if int(self.check_time - self.save_time) > self.timeout:
            self.save_time = rospy.get_time()
            return True
        else:
            return False

    def init_time(self):
        self.save_time = rospy.get_time()

    def check_timeout(self):
        return True if int(rospy.get_time() - self.save_time) > self.timeout else False
