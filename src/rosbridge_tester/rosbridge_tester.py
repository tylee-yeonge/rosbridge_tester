#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from accessories_msgs.srv import SetString
from thira_task_msgs.msg import Task
from thira_task_msgs.msg import TaskFeedback
from thira_task_msgs.msg import TaskState
from thira_task_msgs.msg import TaskEvent

from rosbridge_tester.enums import eTaskName
from rosbridge_tester.enums import eTaskState
from rosbridge_tester.enums import eRunType
from rosbridge_tester.enums import eEventState
from rosbridge_tester.utility.time_checker import TimeChecker
from rosbridge_tester.utility.task_object import TaskObject
from rosbridge_tester.utility.task_queue import TaskQueue

class RosbridgeTester:
    def __init__(self):
        self.set_params()
        self.set_variables()
        self.set_subscriber()
        self.set_pub()
        self.set_service()
        self.set_timer()

    def set_params(self):
        pass

    def set_variables(self):
        self.task_name = eTaskName.INIT
        self.task_state = eTaskState.IDLE
        self.task_id = ""
        self.time_checker_idle = TimeChecker(1)
        self.time_checker_running = TimeChecker(5)
        self.time_checker_task_cancel = TimeChecker(3)
        self.time_checker_init_pose = TimeChecker(3)
        self.task_queue = TaskQueue(10)
        self.em_state = eEventState.DEACTIVATED
        self.manual_state = eEventState.DEACTIVATED
        self.task_cancel_state = eEventState.DEACTIVATED
        self.init_pose_state = eEventState.DEACTIVATED

    def set_subscriber(self):
        rospy.Subscriber("rosbridge_tester/run_task", Task, self.callback_get_task)        

    def set_pub(self):
        self.pub_task_state = rospy.pub("rosbridge_tester/task_state", TaskState, queue_size=10)
        self.pub_task_feedback = rospy.pub("rosbridge_tester/task_feedback", TaskFeedback, queue_size=10)
        self.pub_event_state = rospy.pub("rosbridge_tester/task_event", TaskEvent, queue_size=10)

    def set_service(self):
        rospy.Service("rosbridge_tester/set_name", SetString, self.handle_set_task_id)
        rospy.Service("rosbridge_tester/get_name", Trigger, self.handle_get_task_id)
        rospy.Service("rosbridge_tester/set_em", SetBool, self.handle_set_em)
        rospy.Service("rosbridge_tester/set_manual", SetBool, self.handle_set_manual)
        rospy.Service("rosbridge_tester/set_task_cancel", Trigger, self.handle_set_task_cancel)
        rospy.Service("rosbridge_tester/set_init_pose", Trigger, self.handle_set_init_pose)

    def set_timer(self):
        rospy.Timer(rospy.Duration(1), self.callback_timer)

    def callback_timer(self, event):
        self.check_task_queue()
        self.update_state()
        self.publish_state()

    def callback_get_task(self, msg):
        self.task_queue.put_task(TaskObject(msg.task_name, msg.task_id, msg.run_type, msg.data))

    def handle_set_task_id(self, msg):
        self.task_id = msg.data
        return True, "Success, id: {0}".format(self.task_id)

    def handle_get_task_id(self, msg):
        return True, "Success, id: {0}".format(self.task_id)

    def handle_set_em(self, msg):
        if msg.data:
            self.task_state = self.get_state(eRunType.PAUSE_BY_BUTTON)
            self.em_state = eEventState.ACTIVATING
        else:
            self.task_state = self.get_state(eRunType.RESUME)
            self.em_state = eEventState.DEACTIVATED

    def handle_set_manual(self, msg):
        if msg.data:
            self.task_state = self.get_state(eRunType.PAUSE_BY_BUTTON)
            self.manual_state = eEventState.ACTIVATING
        else:
            self.task_state = self.get_state(eRunType.RESUME)
            self.manual_state = eEventState.DEACTIVATED

    def handle_set_task_cancel(self, msg):
        self.task_cancel_state = eEventState.ACTIVATING

    def handle_set_init_pose(self, msg):
        self.init_pose_state = eEventState.ACTIVATING

    def check_task_queue(self):
        if self.task_state == eTaskState.IDLE:
            if not self.task_queue.check_empty_task_queue():
                task_object = self.task_queue.get_task()
                self.task_name = task_object.get_task_name()
                self.task_id = task_object.get_task_id()
                self.task_state = self.get_state(task_object)
        
        if self.time_checker_task_cancel.set_time(self.task_cancel_state == eEventState.ACTIVATING):
            self.task_cancel_state = eEventState.DEACTIVATED

        if self.time_checker_init_pose.set_time(self.init_pose_state == eEventState.ACTIVATING):
            self.init_pose_state = eEventState.DEACTIVATED


    def get_state(self, task_object):
        if task_object.get_run_type() == eRunType.RUNNING:
            return eTaskState.WAITING_RESPONSE
        elif task_object.get_run_type() == eRunType.STOP:
            return eTaskState.STOP
        elif task_object.get_run_type() == eRunType.STOP_BY_TP:
            return eTaskState.STOP
        elif task_object.get_run_type() == eRunType.PAUSE:
            return eTaskState.PAUSE
        elif task_object.get_run_type() == eRunType.PAUSE_BY_BUTTON:
            return eTaskState.PAUSE
        elif task_object.get_run_type() == eRunType.RESUME:
            return eTaskState.RUNNING
        elif task_object.get_run_type() == eRunType.DONE:
            return eTaskState.DONE

    def update_state(self):
        if self.time_checker_idle.set_time(self.task_state == eTaskState.WAITING_RESPONSE):
            self.task_state = eTaskState.RUNNING
            self.time_checker_running.init_time()
        elif self.time_checker_running.set_time(self.task_state == eTaskState.RUNNING):
            self.task_state = eTaskState.DONE
        elif self.task_state == eTaskState.DONE:
            self.time_checker_idle.init_time()
            self.time_checker_running.init_time()

    def publish_state(self):
        self.pub_task_state.publish(self.get_task_state_msg())
        self.pub_event_state.publish(self.get_event_msg())

    def get_task_state_msg(self):
        task_state = TaskState()
        task_state.task_name = self.task_name.value
        task_state.task_id = self.task_id
        task_state.task_state = self.task_state.value

        return task_state

    def get_event_msg(self):
        task_event = TaskEvent()
        task_event.em = self.em_state.value
        task_event.manual = self.manual_state.value
        task_event.task_cancel = self.task_cancel_state.value
        task_event.init_pose = self.init_pose_state.value
        task_event.chameleon_param = eEventState.DEACTIVATED.value
        task_event.charge = eEventState.DEACTIVATED.value

        return task_event