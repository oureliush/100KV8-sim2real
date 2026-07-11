import sys
import os
import time
from ODrive_Tools import ODrive
import can

bus = can.interface.Bus(interface='socketcan', channel='can0')

Knee_ODrive = ODrive(bus=bus, node_id=1)
Foot_ODrive = ODrive(bus=bus, node_id=2)

Knee_ODrive.clear_errors(identify=0)
Foot_ODrive.clear_errors(identify=0)

Knee_ODrive.set_absolute_pos(pos=-0.429)
Knee_ODrive.write_parameter("axis0.is_homed", True)
Foot_ODrive.set_absolute_pos(pos=-2.416)
Foot_ODrive.write_parameter("axis0.is_homed", True)

bus.shutdown()