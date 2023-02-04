#!/usr/bin/env python
# -*- coding: utf-8 -*-
from enum import Enum
from enum import IntEnum

class eTaskName(Enum):
    INIT = "init"
    WAIT = "wait"
    MOVE = "move"
    LOAD = "load"
    UNLOAD = "unload"
    DOCKING = "docking"    

class eTaskState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    DONE = "done"
    WAITING_RESPONSE = "waiting_response"
    STOP = "stop"
    PAUSE = "pause"

class eRunType(Enum):
    RUNNING = "running"
    STOP = "stop"
    STOP_BY_TP = "stop_by_tp"
    PAUSE = "pause"
    PAUSE_BY_BUTTON = "pause_by_button"
    RESUME = "resume"    
    NONE = "none"
    DONE = "done"

class eEventState(Enum):
    ACTIVATING = "activating"
    DEACTIVATED = "deactivated"

class eCheckerIndex(IntEnum):
    TASK_NAME = 0
    TASK_ID = 1
    TASK_STATE = 2
    EM_STATE = 3
    MANUAL_STATE = 4
    TASK_CANCEL_STATE = 5
    INIT_POSE_STATE = 6