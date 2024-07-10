import serial
import time
import struct

class Cybergear:

    def __init__(self, master_canid, motor_canid):

        self.Master_CANID = master_canid
        self.Motor_CANID = motor_canid