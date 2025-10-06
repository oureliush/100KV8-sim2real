import onnxruntime as ort
import can
import struct
import numpy as np
import time
import threading

from super_useful_functions import *
from preflight import *

CTRL_HZ = 200  # ~5 ms control loop
dt = 1.0 / CTRL_HZ
 
knee_pos = 0.0
knee_vel = 0.0
foot_pos = 0.0
foot_vel = 0.0

LaccelX = 0.0
LaccelY = 0.0
LaccelZ = 0.0
GyroX   = 0.0
GyroY   = 0.0
GyroZ   = 0.0

actions_high = np.array([1.0, 1.0])
actions_low = np.array([-1.0, -1.0])

policy_counter = 0
DECIMATION_FACTOR = 4

grace_time = 0.0
grace_time_ended = False

rlgames = True

IMU_DETECTED = False

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

    print("Waitng for IMU")
    for msg in bus:
        if msg.arbitration_id == 0x12:
            break
    print("IMU ready")

else:
    quit()

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

# Keep the robot from timing out by sending zero velocity while waiting
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
    # Adjust sleep to match your watchdog’s requirement
    time.sleep(0.1)

if user_answer.strip().lower() != 'y':
    quit()



'''
grace_time = time.time()
#-----------------------
# Grace period before robot starts roboting
#-----------------------
while grace_time_ended == False:
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
    elapsed = time.time() - grace_time
    if elapsed > 5.0:
        grace_time_ended = True
'''


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

    CAN_read = time.time()
    # 1. Read all CAN messages available right now
    while True:
        msg = bus.recv(timeout=0)  # Non-blocking read
        if msg is None:
            break
        recv_process_obs(msg)  # Update the global variables
    CAN_elapsed = time.time() - CAN_read


    obs_build = time.time()
    # 2. Build the observation vector
    observation = np.array([
        knee_pos,
        knee_vel,
        -foot_pos,
        -foot_vel,
        LaccelX,
        LaccelY,
        LaccelZ,
        GyroX,
        GyroY,
        GyroZ
    ], dtype=np.float32).reshape(1, -1)
    obs_elapsed = time.time() - obs_build

    '''
    print("Current Observations:")
    print(f"Knee Position: {knee_pos} rad")
    print(f"Knee Velocity: {knee_vel} rad/s")
    print(f"Foot Position: {foot_pos} rad")
    print(f"Foot Velocity: {foot_vel} rad/s")
    print(f"Linear Acceleration X: {LaccelX} m/s²")
    print(f"Linear Acceleration Y: {LaccelY} m/s²")
    print(f"Linear Acceleration Z: {LaccelZ} m/s²")
    print(f"Gyroscope X: {GyroX} rad/s")
    print(f"Gyroscope Y: {GyroY} rad/s")
    print(f"Gyroscope Z: {GyroZ} rad/s")
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

    #print(actions)

    torque_multi = time.time()
    torque_commands = rescaled_actions * policy_torque
    torque_elapsed = time.time() - torque_multi

    #print(torque_commands)

    send = time.time()
    # 4. Send commands
    send_joint_commands(torque_commands)
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