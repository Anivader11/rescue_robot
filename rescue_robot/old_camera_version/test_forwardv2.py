import time
import board
import busio
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver

# Two wheel drive only -- rear right motor failed, rear left is now unused
# too so the robot drives on one wheel per side (front left, front right).
MOTOR_CONFIG = {
    "LEFT_FRONT":  {"address": 0x20, "elec_angle_offset": 1166513920, "sincos_centre": 1236},
    "RIGHT_FRONT": {"address": 0x19, "elec_angle_offset": 1540904960, "sincos_centre": 1250},
}   

SPEED_LIMIT = 546133333
DRIVE_SPEED = 50000000   # start low, increase once direction/sign is confirmed
RUN_TIME = 15            # seconds

i2c = busio.I2C(board.SCL, board.SDA)

def setup_motor(cfg):
    m = PowerfulBLDCDriver(i2c, cfg["address"])
    m.set_current_limit_foc(65536)  # 1 amp -- only matters in FOC run mode
    m.set_id_pid_constants(1100, 150)
    m.set_iq_pid_constants(1100, 150)
    m.set_speed_pid_constants(2.5e-2, 2.5e-4, 2e-2)
    m.set_position_region_boundary(250000)
    m.set_ELECANGLEOFFSET(cfg["elec_angle_offset"])
    m.set_SINCOSCENTRE(cfg["sincos_centre"])
    m.set_speed_limit(SPEED_LIMIT)
    m.configure_operating_mode_and_sensor(3, 1)
    m.configure_command_mode(12)
    m.clear_faults()
    return m

left_front  = setup_motor(MOTOR_CONFIG["LEFT_FRONT"])
right_front = setup_motor(MOTOR_CONFIG["RIGHT_FRONT"])

try:
    print("Driving forward (2 wheel drive: front left + front right only)...")
    end_time = time.time() + RUN_TIME
    while time.time() < end_time:
        right_front.set_speed(DRIVE_SPEED)
        left_front.set_speed(-DRIVE_SPEED)

        time.sleep(0.02)

finally:
    left_front.set_speed(0)
    right_front.set_speed(0)
    for m in (left_front, right_front):
        m.clear_faults()
    print("Stopped.")
