from ODrive_Tools import ODrive
from super_useful_functions import *
from preflight import *

import gc

import can
import threading
import numpy as np
import onnxruntime as ort

#----------------------
# Parameters!!!
#----------------------
session = ort.InferenceSession("2J_100KV8_trained.onnx")

skip_taskset_check = False

can_interface = "socketcan"
can_channel = "can0"

knee_odrive_node_id = 1 
foot_odrive_node_id = 2
imu_id = 0x12

CTRL_HZ = 200  # ~5 ms control loop
DECIMATION_FACTOR = 4

# keep in mind this is using odrive units, Nm
motor_torque_limit = 2.5
# keep in mind this is using odrive units, turns/s
motor_velocity_limit = 20

mock_values = {
    "axis0.config.torque_soft_max": {"value": 0, "writable": True},
    "axis0.config.torque_soft_min": {"value": 0, "writable": True},
    "axis0.config.enable_watchdog": {"value": True, "writable": True},
    "axis0.config.watchdog_timeout": {"value": 0.025, "writable": True},
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
    "axis0.config.watchdog_timeout": {"value": 0.025, "writable": True},
    #set msg intervals, to prevent CANBUS flooding
    "axis0.config.can.heartbeat_msg_rate_ms": {"value": 100, "writable": True},
    "axis0.config.can.encoder_msg_rate_ms": {"value": 3, "writable": True},
    "axis0.config.can.version_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.iq_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.error_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.temperature_msg_rate_ms": {"value": 100, "writable": True},
    "axis0.config.can.bus_voltage_msg_rate_ms": {"value": 100, "writable": True},
    "axis0.config.can.torques_msg_rate_ms": {"value": 0, "writable": True},
    "axis0.config.can.powers_msg_rate_ms": {"value": 0, "writable": True},
}
#----------------------
# Parameters!!!
#----------------------

#----------------------------------------------------------------------------------------------------
# These are self-serving tasks. As in they take care of OS level things 
# and can manage setting up themselves from a fresh boot off of a properly configured system
#----------------------------------------------------------------------------------------------------

if skip_taskset_check == False:
    if check_if_ran_with_taskset(offset=1) != True:
        # we set an offset because on the rpi a cpu is isolated,
        # so it gets subtracted from the total cpu count
        print("The script detected that taskset may not have been used to run this program.")
        input("Continue with Caution... ")

bus = initialize_canbus(interfacef=can_interface, channelf=can_channel)
#----------------------------------------------------------------------------------------------------

#---------------------------------------------------------
#variables that keep the program flowing as it should | Not frequently touched, or at all.
#---------------------------------------------------------
knee_gearbox_ratio = 8/1
foot_gearbox_ratio = -8/1 

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
#-------------------------------------------------------
try:
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
        keep_alive_thread.start()

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

    print("Flushing CAN BUS before resuming operation")
    while can_bus_flushed.is_set() == False:
        pass
    print("Flushed!")

    print("")
    commence = input("Commence sim2real? (y/n) ")
    # input handling
    if commence.strip().lower() == 'y':
        gc.disable() # Disabling automatic garbage collection to reduce jitter. The loop will run garbage collection when its NOT time to do inference.
        gc.collect()

        stop_keep_alive.set()
        keep_alive_thread.join() 
    else:
        stop_keep_alive.set()
        keep_alive_thread.join()
        bus.shutdown()
        quit()

    Knee_ODrive.set_torque_control()
    Foot_ODrive.set_torque_control()


    print("Starting Control Loop!")

    print(f'Encoder Data from the Knee Joint will appear as a CAN_ID with a decimal of {knee_odrive_node_id << 5 | 0x09}')
    print(f'Actions being sent to the Knee Joint will appear as a CAN_ID with a decimal {knee_odrive_node_id << 5 | 0x0e}') 
    print(f'Heartbeat Data from the Knee Joint will appear as a CAN_ID with a decimal of {knee_odrive_node_id << 5 | 0x01}')
    print(f'Temperature Data from the Knee Joint will appear as a CAN_ID with a decimal of {knee_odrive_node_id << 5 | 0x15}')
    print(f'Bus Voltage Data from the Knee will appear as a CAN_ID with a decimal of {knee_odrive_node_id << 5 | 0x17}')

    print(f'Encoder Data from the Foot Joint will appear as a CAN_ID with a decimal of {foot_odrive_node_id << 5 | 0x09}')
    print(f'Actions being sent to the Foot Joint will appear as a CAN_ID with a decimal {foot_odrive_node_id << 5 | 0x0e}')
    print(f'Heartbeat Data from the Knee Joint will appear as a CAN_ID with a decimal of {foot_odrive_node_id << 5 | 0x01}')
    print(f'Temperature Data from the Knee Joint will appear as a CAN_ID with a decimal of {foot_odrive_node_id << 5 | 0x15}')
    print(f'Bus Voltage Data from the Knee will appear as a CAN_ID with a decimal of {foot_odrive_node_id << 5 | 0x17}')

    print(f'IMU Data from the Foot will appear as a CAN_ID with a decimal of {imu_id}') 


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
except KeyboardInterrupt:
    print("\nCaught Ctrl-C, setting ODrives to IDLE, and gracefully shutting down.")
    Knee_ODrive.set_idle()
    Knee_ODrive.clear_errors()
    Foot_ODrive.set_idle()
    Foot_ODrive.clear_errors()
    bus.shutdown()