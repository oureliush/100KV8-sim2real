import sys
from ODrive_Tools import ODrive

import os
import time
import math
import can

#TODO ask if ready to commence, and add timer before operation starts

Calibrated = False
Offset = 0.0

print("This script ONLY is for SYMMETRICAL based endstop calibration.")
time.sleep(1.5)

bus = can.interface.Bus(interface='socketcan', channel='can0')
node_id = int(input("What is the Node_ID of the ODrive you want to calibrate?"))

ODrive = ODrive(bus=bus, node_id=node_id)

# Initalize ODrive 
ODrive.set_idle()
ODrive.clear_errors()

# Save Parameters
old_vel_limit = ODrive.read_parameter(path='axis0.controller.config.vel_limit')
old_neg_torque_limit = ODrive.read_parameter(path='axis0.config.torque_soft_min')
old_pos_torque_limit = ODrive.read_parameter(path='axis0.config.torque_soft_max')

# SAFETY
ODrive.write_parameter(path='axis0.controller.config.vel_limit', value=5)
ODrive.write_parameter(path='axis0.config.torque_soft_min', value=-1)
ODrive.write_parameter(path='axis0.config.torque_soft_max', value=1)

ODrive.set_closed_loop_control()
ODrive.set_torque_control()

# Main Calibration Logic 
while Calibrated == False:
    ODrive.set_input_torque_value(torque_value=0.5)
    if math.fabs(ODrive.get_measured_motor_current_test) >= 5.8:
        ODrive.set_absolute_pos(pos=0.0)
        Calibrated = True 

ODrive.set_input_torque_value(torque_value=0.0)
Calibrated = False

while Calibrated == False:
    ODrive.set_input_torque_value(torque_value=-0.5)
    if math.fabs(ODrive.get_measured_motor_current_test) >= 5.8:
        ODrive.set_absolute_pos(pos = (ODrive.read_parameter(path='odrv.axis0.pos_estimate'))/2)
        Calibrated = True 
ODrive.set_input_torque_value(torque_value=0.0)
# Main Calibration Logic 

# TODO Decide weather to go to 0 pos after calibration or just go idle
ODrive.set_idle()

# Restore Parameters
ODrive.write_parameter(path = 'axis0.controller.config.vel_limit', value = old_vel_limit)
ODrive.write_parameter(path = 'axis0.config.torque_soft_min', value = old_neg_torque_limit)
ODrive.write_parameter(path = 'axis0.config.torque_soft_max', value = old_pos_torque_limit)

