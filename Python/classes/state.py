from enum import Enum

class State(Enum):
    STANDBY = 0
    PROG_SCANNING = 1
    DONE_SCANNING = 2
    PROG_LASER = 3
    DONE_LASER = 4
    TRANSITION = 80
    REORIENTING = 81
    ABORT = 82
