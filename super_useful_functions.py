import can
import struct
import numpy as np
import time
import onnxruntime as ort
from ODrive_Tools import ODrive

#TODO: Fix the arduino firmware to send just sent integer values and not bytes
#to elminate the function below

def bytes_to_signed_int(high_byte, low_byte):
    value = (high_byte << 8) | low_byte
    if value >= 0x8000:  # >= 32768
        value -= 0x10000
    return value

def recv_process_obs(message: can.Message, observation_array: np.array, knee_ratio: int, foot_ratio: int, knee_id: int, foot_id: int, lock, imu_id):
    data = message.data
    with lock:
        # Knee
        if message.arbitration_id == (knee_id << 5 | 0x09):  # 0x09: Get_Encoder_Estimates
            knee_pos, knee_vel = struct.unpack('<ff', bytes(data))
            knee_pos *= (2 * np.pi) / knee_ratio
            knee_vel *= (2 * np.pi) / knee_ratio
            observation_array[0, 0] = knee_pos
            observation_array[0, 1] = knee_vel

        # Foot
        elif message.arbitration_id == (foot_id << 5 | 0x09):  # 0x09: Get_Encoder_Estimates
            foot_pos, foot_vel = struct.unpack('<ff', bytes(data))
            foot_pos *= (2 * np.pi) / foot_ratio
            foot_vel *= (2 * np.pi) / foot_ratio
            observation_array[0, 2] = foot_pos
            observation_array[0, 3] = foot_vel

        # Accelerometer
        elif message.arbitration_id == imu_id:
            Int_LaccelX = bytes_to_signed_int(data[0], data[1])
            Int_LaccelY = bytes_to_signed_int(data[2], data[3])
            Int_LaccelZ = bytes_to_signed_int(data[4], data[5])
            LaccelX = Int_LaccelX / 100
            LaccelY = Int_LaccelY / 100
            LaccelZ = Int_LaccelZ / 100
            observation_array[0, 4] = LaccelX
            observation_array[0, 5] = LaccelY
            observation_array[0, 6] = LaccelZ


def can_read_thread(bus: can.interface.Bus, observation_array: np.array, knee_ratio: int, foot_ratio: int, knee_id: int, foot_id: int, lock, imu_id):
    while bus.recv(timeout=0) is not None:
        pass
    
    while True:
        msg = bus.recv()  # non blocking read
        if msg is not None:
            recv_process_obs(
                message=msg,
                observation_array=observation_array,
                knee_ratio=knee_ratio,
                foot_ratio=foot_ratio,
                knee_id=knee_id,
                foot_id=foot_id,
                lock=lock,
                imu_id=imu_id,
                )

def send_joint_commands(actions: np.ndarray, Knee_ODrive: ODrive, Foot_ODrive: ODrive, trained_model_motor_torque_limitscale):
    # Convert from full gear torque to motor torque
    knee_action = actions[0] * trained_model_motor_torque_limitscale
    foot_action = actions[1] * trained_model_motor_torque_limitscale

    Knee_ODrive.set_input_torque_value(knee_action)
    Foot_ODrive.set_input_torque_value(foot_action)


def rescale_actions(low, high, action):
    d = (high - low) / 2.0
    m = (high + low) / 2.0
    scaled_action = action * d + m
    return scaled_action

def run_control_loop(CTRL_HZ: int, DECIMATION_FACTOR: int, onnx_model: ort.InferenceSession, obs: np.array, actions_low: np.array, actions_high: np.array, Knee_ODrive, Foot_ODrive, motor_torque_scale, lock):
    dt = 1.0 / CTRL_HZ
    while True:
        with lock:
            loop_start_time = time.perf_counter()

            if policy_counter % DECIMATION_FACTOR == 0:
                actions = onnx_model.run(None, {"obs": obs})
                actions = actions[0][0]
                clamped_actions = np.clip(actions, -1.0, 1.0)
                rescaled_actions = rescale_actions(actions_low, actions_high, clamped_actions)
                send_joint_commands(rescaled_actions, Knee_ODrive, Foot_ODrive,motor_torque_scale)
            
            policy_counter += 1

            elapsed = time.perf_counter() - loop_start_time
            time.sleep(max(0.0, dt - elapsed))


def run_decimation_control_loop(CTRL_HZ: int, DECIMATION_FACTOR: int, onnx_model: ort.InferenceSession, obs: np.array, actions_low: np.array, actions_high: np.array, Knee_ODrive, Foot_ODrive, motor_torque_scale, lock):
    dt = 1.0 / (CTRL_HZ/DECIMATION_FACTOR)
    while True:
        with lock:
            loop_start_time = time.perf_counter()

            actions = onnx_model.run(None, {"obs": obs})
            actions = actions[0][0]
            clamped_actions = np.clip(actions, -1.0, 1.0)
            rescaled_actions = rescale_actions(actions_low, actions_high, clamped_actions)
            send_joint_commands(rescaled_actions, Knee_ODrive, Foot_ODrive,motor_torque_scale)

            elapsed = time.perf_counter() - loop_start_time
            time.sleep(max(0.0, dt - elapsed))

def flush_can_bus(bus: can.interface.Bus):
    while not (bus.recv(timeout=0) is None): pass