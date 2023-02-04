#!/usr/bin/env python
import sys
if sys.version.split('.')[0] == "2":
    from Queue import Queue
if sys.version.split('.')[0] == "3":
    from queue import Queue
from copy import deepcopy

class TaskQueue:
    def __init__(self, max_size):
        self.queue = Queue(max_size)

    def put_task(self, task):
        self.queue.put(task)

    def get_task(self):
        return self.queue.get()

    def check_full_task_queue(self):
        return self.queue.full()

    def check_empty_task_queue(self):
        return self.queue.empty()

    def get_task_queue_size(self):
        return self.queue.qsize()

    def get_task_queue(self):
        return deepcopy(self.queue)

    def clear_task_queue(self):
        while not self.queue.empty():
            self.queue.get()
