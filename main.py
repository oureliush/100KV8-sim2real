from ODrive_Tools import ODrive
from super_useful_functions import *
from preflight import *

import can
import threading
import numpy as np
import onnxruntime as ort


#----------------------
# Parameters!!!
#----------------------
session = ort.InferenceSession("2J_100KV8_dummy.onnx")

# sudo ip link set can0 type can bitrate 1000000
# sudo ip link set up can0
bus = can.interface.Bus(interface='socketcan', channel='can0')

knee_odrive_node_id = 1 
foot_odrive_node_id = 2
imu_id = 0x12

CTRL_HZ = 200  # ~5 ms control loop
DECIMATION_FACTOR = 4

# keep in mind this is using odrive units, Nm
motor_torque_limit = 2.0
# keep in mind this is using odrive units, turns/s
motor_velocity_limit = 20

mock_values = {
    "axis0.config.torque_soft_max": {"value": 0, "writable": True},
    "axis0.config.torque_soft_min": {"value": 0, "writable": True},
    "axis0.config.enable_watchdog": {"value": True, "writable": True},
    "axis0.config.watchdog_timeout": {"value": 0.5, "writable": True},
    "axis0.config.can.heartbeat_msg_rate_ms": {"value": 100, "writable": True},
    "axis0.config.can.encoder_msg_rate_ms": {"value": 3, "writable": True},
    "axis0.config.can.version_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.iq_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.error_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.temperature_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.bus_voltage_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.torques_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.powers_msg_rate_ms": {"value": 0, "writable": True},
}

real_values = {
    "axis0.config.torque_soft_max": {"value": motor_torque_limit, "writable": False},
    "axis0.config.torque_soft_min":  {"value": -motor_torque_limit, "writable": False},
    "axis0.is_homed": {"value": True, "writable": False},
    "axis0.controller.config.vel_limit": {"value": motor_velocity_limit, "writable": False},
    "axis0.config.enable_watchdog": {"value": True, "writable": True},
    "axis0.config.watchdog_timeout": {"value": 0.5, "writable": True},
    #set msg intervals, to prevent CANBUS flooding
    "axis0.config.can.heartbeat_msg_rate_ms": {"value": 100, "writable": True},
    "axis0.config.can.encoder_msg_rate_ms": {"value": 3, "writable": True},
    "axis0.config.can.version_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.iq_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.error_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.temperature_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.bus_voltage_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.torques_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.powers_msg_rate_ms": {"value": 0, "writable": True},
}
#----------------------
# Parameters!!!
#----------------------

#---------------------------------------------------------
#variables that keep the program flowing as it should | Not frequently touched, or at all.
#---------------------------------------------------------
knee_gearbox_ratio = 8/1
foot_gearbox_ratio = 8/1

# whatever the value the actions was multiplied by in training
trained_model_motor_torque_limitscale = 5.0

Knee_ODrive = ODrive(bus=bus, node_id=knee_odrive_node_id)
Foot_ODrive = ODrive(bus=bus, node_id=foot_odrive_node_id)

can_bus_flushed = threading.Event()
stop_keep_alive = threading.Event()

observation_array = np.zeros((1, 7), dtype=np.float32)

obs_lock = threading.Lock()

actions_high = np.array([1.0, 1.0])
actions_low = np.array([-1.0, -1.0])

read_thread = threading.Thread(target=can_read_thread, kwargs={
    "bus": bus, 
    "observation_array": observation_array, 
    "knee_ratio": knee_gearbox_ratio, 
    "foot_ratio": foot_gearbox_ratio,
    "knee_id": knee_odrive_node_id,
    "foot_id": foot_odrive_node_id,
    "imu_id": imu_id,
    "lock": obs_lock,
    "flag": can_bus_flushed
})


control_loop_thread = threading.Thread(target=run_control_loop, kwargs={
    "CTRL_HZ": CTRL_HZ, 
    "DECIMATION_FACTOR": DECIMATION_FACTOR, 
    "onnx_model": session, 
    "obs": observation_array,
    "actions_low": actions_low,
    "actions_high": actions_high,
    "Knee_ODrive": Knee_ODrive,
    "Foot_ODrive": Foot_ODrive,
    "motor_torque_scale": trained_model_motor_torque_limitscale,
    "lock": obs_lock
})

decimation_control_loop_thread = threading.Thread(target=run_decimation_control_loop, kwargs={
    "CTRL_HZ": CTRL_HZ, 
    "DECIMATION_FACTOR": DECIMATION_FACTOR, 
    "onnx_model": session, 
    "obs": observation_array,
    "actions_low": actions_low,
    "actions_high": actions_high,
    "Knee_ODrive": Knee_ODrive,
    "Foot_ODrive": Foot_ODrive,
    "motor_torque_scale": trained_model_motor_torque_limitscale,
    "lock": obs_lock
})

keep_alive_thread = threading.Thread(target=keep_odrives_alive_by_sending_zero_pos, kwargs={
    "stop_flag": stop_keep_alive, 
    "Knee_ODrive": Knee_ODrive, 
    "Foot_ODrive": Foot_ODrive, 
})

read_thread.daemon = True
keep_alive_thread.daemon = True # this is important because if Ctrl-C is done at commence sim2real prompt, the keep alive thread wont stop!

#---------------------------------------------------------
#variables that keep the program flowing as it should | Not frequently touched
#---------------------------------------------------------

#--------------------------------------------------------------------------------------------------------------------
#BIG ASS DIVIDER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#--------------------------------------------------------------------------------------------------------------------

#-------------------------------------------------------
# ACTUAL PROGRAM
#---------------------------------------------------------
print("Running first 50 inferences to make subsequent runs faster", end= ".. ")
for _ in range(50):
    session.run(None, {"obs": np.empty((1, 7), dtype=np.float32)})

print("Done")
print("")

mock_test = input("Is this a mock test? (y/n) ")
#input handling
if mock_test.strip().lower() == 'y':
    do_preflight_checks([Knee_ODrive, Foot_ODrive], mock_values)
elif mock_test.strip().lower() == 'n':
    confirmation = input("Please confirm that this is a real test! ")
    if confirmation.strip().lower() == 'y':
        print("Confirmed")
        do_preflight_checks([Knee_ODrive, Foot_ODrive], real_values)
    else:
        bus.shutdown()
        quit()
else:
    bus.shutdown()
    quit()


initalize = input("Joints will be set to Closed Loop Control and positions set to 0, Continue? (y/n) ")
#input handling
if initalize.strip().lower() == 'y':
    print("Waiting on Knee Joint")
    Knee_ODrive.set_closed_loop_control()
    Knee_ODrive.set_position_control()
    print("Knee Joint Ready")

    print("Waiting on Foot Joint")
    Foot_ODrive.set_closed_loop_control()
    Foot_ODrive.set_position_control()
    print("Foot Joint Ready")

    print("Waiting for IMU")
    for msg in bus:
        if msg.arbitration_id == imu_id:
            break
    print("All controllers working properly")
else:
    bus.shutdown()
    quit()

# we start the read thread here because we are no longer calling functions which expect responses from the odrives.
read_thread.start()

keep_alive_thread.start()

print("Flushing CAN BUS before resuming operation")
while can_bus_flushed.is_set() == False:
    pass
print("Flushed!")

print("")
commence = input("Commence sim2real? (y/n) ")
# input handling
if commence.strip().lower() == 'y':
    stop_keep_alive.set()
    keep_alive_thread.join() 
else:
    stop_keep_alive.set()
    keep_alive_thread.join()
    bus.shutdown()
    quit()

Knee_ODrive.set_torque_control()
Foot_ODrive.set_torque_control()

#small delay
time.sleep(0.1)

# This is where the real magic happens! If you looking at this script
# as reference on how to achieve sim2real, this function is what your looking for!
run_control_loop(CTRL_HZ=CTRL_HZ,
                 DECIMATION_FACTOR=DECIMATION_FACTOR,
                 onnx_model=session,
                 obs=observation_array,
                 actions_high=actions_high,
                 actions_low=actions_low,
                 Knee_ODrive=Knee_ODrive,
                 Foot_ODrive=Foot_ODrive,
                 motor_torque_scale=trained_model_motor_torque_limitscale,
                 lock=obs_lock
                 )

# if going for thread approach, make sure you disable daemon mode for the can_read_thread
#control_loop_thread.start()