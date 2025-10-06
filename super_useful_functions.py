import can
import struct
import numpy as np

bus = can.interface.Bus(interface='socketcan', channel='can0')

knee_id = 1
foot_id = 2

knee_ratio = 6
foot_ratio = 2

def bytes_to_signed_int(high_byte, low_byte):
    value = (high_byte << 8) | low_byte
    if value >= 0x8000:  # >= 32768
        value -= 0x10000
    return value




def recv_process_obs(message):
    global knee_pos, knee_vel, foot_pos, foot_vel
    global LaccelX, LaccelY, LaccelZ, GyroX, GyroY, GyroZ

    data = message.data
    # Knee
    if message.arbitration_id == (knee_id << 5 | 0x09):  # 0x09: Get_Encoder_Estimates
        knee_pos, knee_vel = struct.unpack('<ff', bytes(data))
        knee_pos *= (2 * np.pi) / knee_ratio
        knee_vel *= (2 * np.pi) / knee_ratio

    # Foot
    elif message.arbitration_id == (foot_id << 5 | 0x09):  # 0x09: Get_Encoder_Estimates
        foot_pos, foot_vel = struct.unpack('<ff', bytes(data))
        foot_pos *= (2 * np.pi) / foot_ratio
        foot_vel *= (2 * np.pi) / foot_ratio

    # Accelerometer
    elif message.arbitration_id == 0x12:
        Int_LaccelX = bytes_to_signed_int(data[0], data[1])
        Int_LaccelY = bytes_to_signed_int(data[2], data[3])
        Int_LaccelZ = bytes_to_signed_int(data[4], data[5])
        LaccelX = Int_LaccelX / 100
        LaccelY = Int_LaccelY / 100
        LaccelZ = Int_LaccelZ / 100

    # Gyroscope
    elif message.arbitration_id == 0x13:
        Int_GyroX = bytes_to_signed_int(data[0], data[1])
        Int_GyroY = bytes_to_signed_int(data[2], data[3])
        Int_GyroZ = bytes_to_signed_int(data[4], data[5])
        GyroX = Int_GyroX / 100
        GyroY = Int_GyroY / 100
        GyroZ = Int_GyroZ / 100





def send_joint_commands(actions: np.ndarray):
    # Convert from full gear torque to motor torque
    # actions[0] = actions[0] / knee_ratio
    # actions[1] = actions[1] / foot_ratio

    # Send knee torque
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x00e),  # 0x00e: Set_Input_Torque
        data=struct.pack('<f', actions[0]),
        is_extended_id=False
    ))

    # Send foot torque (negative sign if needed)
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x00e),  # 0x00e: Set_Input_Torque
        data=struct.pack('<f', -actions[1]),
        is_extended_id=False
    ))





def rescale_actions(low, high, action):
    d = (high - low) / 2.0
    m = (high + low) / 2.0
    scaled_action = action * d + m
    return scaled_action


def check_for_odrive_errors(node_id):
    print("placeholder for later")