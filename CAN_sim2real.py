import onnxruntime as ort
import can
import struct
import numpy as np
import time
import threading

import super_useful_functions as suf
from preflight import *

CTRL_HZ = 200  # ~5 ms control loop
dt = 1.0 / CTRL_HZ
 
knee_pos = None
knee_vel = None
foot_pos = None
foot_vel = None

LaccelX = None
LaccelY = None
LaccelZ = None

Pitch = None
GyroX = None
GyroY = None
GyroZ = None

IMU_DETECTED = False


actions_high = np.array([1.0, 1.0])
actions_low = np.array([-1.0, -1.0])

policy_counter = 0
DECIMATION_FACTOR = 4

grace_time = 0.0
grace_time_ended = False

print("Flushing CAN buffer...")

# Flush the buffer: read and discard all existing messages
while bus.recv(timeout=0) is not None:
    pass

print("Flushed")

read_thread = threading.Thread(target=can_read_thread)

session = ort.InferenceSession("Leg2Lite.onnx")  # update with your model path

preflight_checks()

initalize = input("Joints will be set to Closed Loop Control and positions set to 0.0, Continue? (y/n) ")
print(initalize)

if initalize == 'y':
    # Switch knee to CLOSED_LOOP_CONTROL
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x07),  # 0x07: Set_Axis_State
        data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    print("Waitng on Knee Joint")
    for msg in bus:
        if msg.arbitration_id == (knee_id << 5 | 0x01):  # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                break

    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 3, 1),  # 3, Position Control, 1 Passthrough
        is_extended_id=False
    ))
    print("Knee Joint ready")

    # Switch foot to CLOSED_LOOP_CONTROL
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x07),
        data=struct.pack('<I', 8),
        is_extended_id=False
    ))
    
    print("Waiting on Foot Joint")
    for msg in bus:
        if msg.arbitration_id == (foot_id << 5 | 0x01):
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
                break

    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
        data=struct.pack('<II', 3, 1),  # 3, Position Control, 1 Passthrough
        is_extended_id=False
    ))
    print("Foot Joint ready")

    print("Waitng for Foot IMU")
    for msg in bus:
        if msg.arbitration_id == 0x12:
            break
    print("Foot IMU ready")

    print("Waitng for Orientation IMU")
    for msg in bus:
        if msg.arbitration_id == 0x10:
            break
    print("Orientation IMU ready")

else:
    quit()

read_thread.start() #start CAN read thread after we except responses from our own actions so that the can thread doesnt steal and dump the responses before our actions expcect/need them to continue

# -----------------------
# Non-Blocking User Prompt (Commence sim2real?)
# -----------------------
user_answer = None
input_received = False

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
    # Send zero position to knee
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x0c),  # 0x0c: Set_Input_Pos
        data=struct.pack('<f', 0.0),
        is_extended_id=False
    ))
    # Send zero position to foot
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x0c),
        data=struct.pack('<f', 0.0),
        is_extended_id=False
    ))
    # adjust sleep to match watchdog’s requirement
    time.sleep(0.1)

if user_answer.strip().lower() != 'y':
    quit()




bus.send(can.Message(
    arbitration_id=(knee_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
    data=struct.pack('<II', 1, 1),  # 1, Torque Control, 1 Passthrough
    is_extended_id=False
))

bus.send(can.Message(
    arbitration_id=(foot_id << 5 | 0x0b),  # 0x0b: Set_Controller_Mode
    data=struct.pack('<II', 1, 1),  # 1, Torque Control, 1 Passthrough
    is_extended_id=False
))



# -----------------------
# Main Control Loop
# -----------------------
print(f"Starting control loop at {CTRL_HZ} Hz...")

while True:
    loop_start_time = time.time()

    #CAN Read is handled in another thread and variables are updated in super_useful_functions.pyt

    obs_build = time.time()
    # 2. Build the observation vector
    observation = np.array([
        suf.knee_pos,
        suf.knee_vel,
        -suf.foot_pos,
        -suf.foot_vel,
        suf.LaccelX,
        suf.LaccelY,
        suf.LaccelZ,
        suf.GyroX,
        suf.GyroY,
        suf.GyroZ
    ], dtype=np.float32).reshape(1, -1)
    obs_elapsed = time.time() - obs_build

    print(observation)

    '''
    print("Current Observations:")
    print(f"Knee Position: {suf.knee_pos} rad")
    print(f"Knee Velocity: {suf.knee_vel} rad/s")
    print(f"Foot Position: {-suf.foot_pos} rad")
    print(f"Foot Velocity: {-suf.foot_vel} rad/s")
    print(f"Linear Acceleration X: {suf.LaccelX} m/s²")
    print(f"Linear Acceleration Y: {suf.LaccelY} m/s²")
    print(f"Linear Acceleration Z: {suf.LaccelZ} m/s²")
    print(f"Gyroscope X: {suf.GyroX} rad/s")
    print(f"Gyroscope Y: {suf.GyroY} rad/s")
    print(f"Gyroscope Z: {suf.GyroZ} rad/s")
    print(f"Pitch: {suf.Pitch} rad/s")
    print("-" * 40)
    '''

    inference = time.time()
    if policy_counter % DECIMATION_FACTOR == 0:
        actions = session.run(None, {"obs": observation})
        actions = actions[0][0]  # e.g. array([torque_knee, torque_foot])
        clamped_actions = np.clip(actions, -1.0, 1.0)
        rescaled_actions = rescale_actions(actions_low, actions_high, clamped_actions)
        #torque_commands = rescaled_actions * torque_limits
        #send_joint_commands(torque_commands)
    inference_elapsed = time.time() - inference

    print(actions)

    torque_multi = time.time()
    torque_commands = rescaled_actions * policy_torque
    torque_elapsed = time.time() - torque_multi

    #print(torque_commands)

    send = time.time()
    # 4. Send commands
    #send_joint_commands(torque_commands)
    send_elapsed = time.time() - send

    '''
    print(f"CAN Time {CAN_elapsed}")
    print(f"Obs + {obs_elapsed}")
    print(f"inference {inference_elapsed}")
    print(f"torque {torque_elapsed}")
    print(f"send {send_elapsed}")
    '''
    
    policy_counter += 1

    # 5. Sleep to maintain the desired loop rate
    elapsed = time.time() - loop_start_time
    time.sleep(max(0.0, dt - elapsed))
    # Optionally log or print the loop rate timing here
    if dt-elapsed < 0.0:
        print("Control Loop is LATE! Stopping Operation.")
        print(f"loop time {elapsed}")
        break