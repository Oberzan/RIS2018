#!/usr/bin/env python


ACTION_CLIENT_STATUSES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST"
}

ROTATE_SPEED = 30
ROTATE_ANGLE = 390

GOAL_RESULT_TIMEOUT = 30