#!/usr/bin/env python
import rospy
from rosbridge_tester.enums import eRunType
from rosbridge_tester.enums import eTaskName
from copy import deepcopy

class TaskObject:
    def __init__(self, task_name, task_id, run_type, data):
        self.task_name = task_name
        if type(task_name) == type(""):
            self.task_name = eTaskName[task_name.upper()]
        elif type(task_name) == type(eTaskName.INIT):
            self.task_name = task_name
        self.task_id = task_id
        if type(run_type) == type(""):
            self.run_type = eRunType[run_type.upper()]
        elif type(run_type) == type(eRunType.RUNNING):
            self.run_type = run_type
        if type(data) == type([]):
            self.data_table = self.get_table_from_list(data)            
        elif type(data) == type({}):
            self.data_table = deepcopy(data)
        rospy.loginfo("in TaskObject, task_name: {0}, task_id: {1}, run_type: {2}".format(task_name, task_id, run_type))

    def get_task_name(self):
        return self.task_name

    def get_task_id(self):
        return self.task_id

    def get_run_type(self):
        return self.run_type

    def get_data_table(self):
        return self.data_table

    def get_table_from_list(self, data_list):
        table = {}
        for item in data_list:
            table[item.key] = item.value
            rospy.loginfo("in TaskObject, get_table_from_list, data: {0}".format(table))
        return table

    def set_table_from_list(self, data_list):
        self.data_table = {}
        self.data_table = self.get_table_from_list(data_list)    

    def set_data_table(self, table):
        self.data_table = table
        rospy.loginfo("in TaskObject, set_data_table, data: {0}".format(table))