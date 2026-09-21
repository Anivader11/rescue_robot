import time
import logging
from control.motors import BLDCMotorDriver
import time
import sys
import board
import struct
import busio
import select
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

motor = [None] * 8

#Motor1
motor1 = PowerfulBLDCDriver(i2c, 26)
motor1.set_current_limit_foc(262144)  # max 8 amps is 524288
motor1.set_id_pid_constants(1500, 200)
motor1.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
motor1.set_position_pid_constants(275, 0, 0)
motor1.set_ELECANGLEOFFSET(1161314304)
motor1.set_SINCOSCENTRE(1244)

#Motor2
motor2 = PowerfulBLDCDriver(i2c, 26)
motor2.set_current_limit_foc(262144)  # max 8 amps is 524288
motor2.set_id_pid_constants(1500, 200)
motor2.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
motor2.set_position_pid_constants(275, 0, 0)
motor2.set_ELECANGLEOFFSET(1161314304)
motor2.set_SINCOSCENTRE(1244)

#Motor3
motor3 = PowerfulBLDCDriver(i2c, 26)
motor3.set_current_limit_foc(262144)  # max 8 amps is 524288
motor3.set_id_pid_constants(1500, 200)
motor3.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
motor3.set_position_pid_constants(275, 0, 0)
motor3.set_ELECANGLEOFFSET(1161314304)
motor3.set_SINCOSCENTRE(1244)

#Motor 4
motor4 = PowerfulBLDCDriver(i2c, 26)
motor4.set_current_limit_foc(262144)  # max 8 amps is 524288
motor4.set_id_pid_constants(1500, 200)
motor4.set_speed_pid_constants(4e-2, 4e-4, 3e-2)
motor4.set_position_pid_constants(275, 0, 0)
motor4.set_ELECANGLEOFFSET(1161314304)
motor4.set_SINCOSCENTRE(1244)

motor[0x20] = motor1(0x20)
motor[0x1B] = motor2(0x1B)
motor[0x19] = motor3(0x19)
motor[0x1A] = motor4(0x1A)

motor[0x20].set_speed(30000000)
motor[0x1B].set_speed(30000000)
motor[0x19].set_speed(30000000)
motor[0x1A].set_speed(30000000)
