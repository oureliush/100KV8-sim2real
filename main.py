from ODrive_Tools import ODrive
import can
from preflight import *
import threading
import numpy as np
import onnxruntime as ort
from super_useful_functions import *

#----------------------
# Parameters!!!
#----------------------
CTRL_HZ = 200  # ~5 ms control loop
DECIMATION_FACTOR = 4

bus = can.interface.Bus(interface='socketcan', channel='can0')

knee_odrive_node_id = 1 
foot_odrive_node_id = 2
imu_id = 0x12

# keep in mind this is using odrive units, Nm
motor_torque_limit = 3.0
# keep in mind this is using odrive units, turns/s
motor_velocity_limit = 20


mock_values = {
    "axis0.config.torque_soft_max": 0,
    "axis0.config.torque_soft_min": 0,
}
real_values = {
    "axis0.config.torque_soft_max": motor_torque_limit,
    "axis0.config.torque_soft_min": -motor_torque_limit,
    "axis0.is_homed": True,
    "axis0.controller.config.vel_limit" : motor_velocity_limit,
    "axis0.config.enable_watchdog": True,
    #set msg intervals, to prevent CANBUS flooding
    "axis0.config.can.heartbeat_msg_rate_ms": 100,
    "axis0.config.can.encoder_msg_rate_ms": 3,
    "axis0.config.can.version_msg_rate_ms": 0,
    "axis0.config.can.iq_msg_rate_ms": 0,
    "axis0.config.can.error_msg_rate_ms": 0,
    "axis0.config.can.temperature_msg_rate_ms": 0,
    "axis0.config.can.bus_voltage_msg_rate_ms": 0,
    "axis0.config.can.torques_msg_rate_ms": 0,
    "axis0.config.can.powers_msg_rate_ms": 0,
}

session = ort.InferenceSession("Leg2Lite.onnx")
#----------------------
# Parameters!!!
#----------------------

#---------------------------------------------------------
#variables that keep the program flowing as it should | Not frequently touched, or at all.
#---------------------------------------------------------

knee_gearbox_ratio = 8
foot_gearbox_ratio = 8

# whatever the value the actions was multiplied by in training
trained_model_motor_torque_limitscale = 5.0

Knee_ODrive = ODrive(bus=bus, node_id=knee_odrive_node_id)
Foot_ODrive = ODrive(bus=bus, node_id=foot_odrive_node_id)

user_answer = None
input_received = False

observation_array = np.empty((7), dtype=np.float32)

obs_lock = threading.Lock()

actions_high = np.array([1.0, 1.0])
actions_low = np.array([-1.0, -1.0])

read_thread = threading.Thread(target=can_read_thread, args=(bus,observation_array,knee_gearbox_ratio,foot_gearbox_ratio,knee_odrive_node_id,foot_odrive_node_id,obs_lock,imu_id))
control_loop_thread = threading.Thread(target=run_control_loop, args=(CTRL_HZ,DECIMATION_FACTOR,session,observation_array,actions_low,actions_high,Knee_ODrive,Foot_ODrive,trained_model_motor_torque_limitscale,obs_lock))
decimation_control_loop_thread = threading.Thread(target=run_decimation_control_loop, args=(CTRL_HZ,DECIMATION_FACTOR,session,observation_array,actions_low,actions_high,Knee_ODrive,Foot_ODrive,trained_model_motor_torque_limitscale, obs_lock))

#---------------------------------------------------------
#variables that keep the program flowing as it should | Not frequently touched
#---------------------------------------------------------

#--------------------------------------------------------------------------------------------------------------------
#BIG ASS DIVIDER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#--------------------------------------------------------------------------------------------------------------------

#-------------------------------------------------------
# ACTUAL PROGRAM
#---------------------------------------------------------

mock_test = input("Is this a mock test? ")
print(mock_test)

if mock_test.strip().lower() == 'y':
    do_preflight_checks([Knee_ODrive, Foot_ODrive], mock_values)
elif mock_test.strip().lower() == 'n':
    
    confirmation = input("Please confirm that this is a real test! ")
    print(confirmation)
    if confirmation.strip().lower() == 'y':
        print("Confirmed")
        do_preflight_checks([Knee_ODrive, Foot_ODrive], real_values)
    else:
        quit()
else:
    quit()
    

initalize = input("Joints will be set to Closed Loop Control and positions set to 0.0, Continue? (y/n) ")
print(initalize)

if initalize == 'y':
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
        if msg.arbitration_id == 0x12:
            break
else:
    quit()

def input_thread_func(prompt):
    global user_answer, input_received
    user_answer = input(prompt)
    input_received = True

# Start the thread that asks about sim2real
thread = threading.Thread(
    target=input_thread_func,
    args=('Commence sim2real? (y/n) ',)
)
thread.start()

# keep the robot from timing out by sending zero velocity while waiting
while not input_received:
    Knee_ODrive.set_input_position_value(0.0)
    Foot_ODrive.set_input_position_value(0.0)

    # lets not flood the CAN bus with nonsense.
    time.sleep(0.1)

if user_answer.strip().lower() != 'y':
    quit()


Knee_ODrive.set_torque_control()
Foot_ODrive.set_torque_control()