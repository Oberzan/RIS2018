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

ROTATE_SPEED = 20
ROTATE_ANGLE = 360

GOAL_RESULT_TIMEOUT = 300



ROTATING = "rotating"
DEFAULT = "default"
OBSERVING = "observing"
MOVING = "moving"
CIRCLE_APPROACHING = "circle_approaching"
CIRCLE_APPROACHED = "circle_approaching"