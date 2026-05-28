import can
import struct
import numpy as np
import time
import onnxruntime as ort
from ODrive_Tools import ODrive
import threading
import pyudev
import subprocess
import os
import gc

#Safety Threads??

#TODO: eliminate bytes to signed function and just use struct.unpack.. eventuallys

def check_if_ran_with_taskset(offset: int):
    # false positives can occur if OS limits the allowed cpus for some other reason.
    total_cpus = os.cpu_count()
    total_cpus = total_cpus - offset
    allowed_cpus = len(os.sched_getaffinity(0))
    return allowed_cpus < total_cpus

def initialize_canbus(interfacef: str = "socketcan", channelf: str = "can0"):
    can_initalized = False  

    try:
        can.interface.Bus(interface=interfacef, channel=channelf)
    except OSError as e:
        if e.errno == 19:
            context = pyudev.Context()
            monitor = pyudev.Monitor.from_netlink(context)
            monitor.filter_by(subsystem='usb')

            print("Waiting on USB-CAN adapter")
            for device in iter(monitor.poll, None):
                if device.action == 'add' and device.get('ID_MODEL') == "USB_to_CAN_Adapter":
                    time.sleep(0.1)
                    can.interface.Bus(interface=interfacef, channel=channelf) # If this fails, USB Connection is likely intermittent
                    break

    print("USB-CAN adapter Connected")


    print("Checking if CAN interface is initalized...")

    result0 = subprocess.run(["cat", "/sys/class/net/can0/operstate"], capture_output=True, text=True)

    if result0.stdout == "up\n":
        can_initalized = True

    if can_initalized == True:
        print("CAN interface detected!")
        buss = can.interface.Bus(interface=interfacef, channel=channelf)
        return buss
    else:
        print("CAN interface not detected! Attempting to initalize!")
        result1 = subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"], capture_output=True, text=True)
        if result1.returncode == 0:
            result1 = subprocess.run(["sudo", "ip", "link", "set", "up", "can0"], capture_output=True, text=True)
        else:
            raise OSError("Something went wrong: Command did not run sucessfully. Code: 1") # command "sudo ip link set can0 type can bitrate 1000000" did not run successfully

        result3 = subprocess.run(["cat", "/sys/class/net/can0/operstate"], capture_output=True, text=True)
        if result3.stdout == "up\n" and result3.returncode == 0:
            can_initalized = True
            buss = can.interface.Bus(interface=interfacef, channel=channelf)
            print("CAN interface has been initalized!")
            return buss
        else:
            raise OSError("Something went wrong: Returned value was different from expected value. Code: 2") # CAN interface was expected to be up, but was not.


def bytes_to_signed_int(high_byte, low_byte):
    value = (high_byte << 8) | low_byte
    if value >= 0x8000:  # >= 32768
        value -= 0x10000
    return value

def recv_process_obs(message: can.Message, observation_array: np.array, knee_ratio: int, foot_ratio: int, knee_id: int, foot_id: int, lock, imu_id: int):
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



def can_read_thread(bus: can.interface.Bus, observation_array: np.array, knee_ratio: int, foot_ratio: int, knee_id: int, foot_id: int, lock: threading.Lock, imu_id: int, flag: threading.Event):
    while bus.recv(timeout=0) is not None:
        pass
    
    # alerts that canbus has been flushed
    flag.set()

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

def send_joint_commands(actions: np.ndarray, Knee_ODrive: ODrive, Foot_ODrive: ODrive, trained_model_motor_torque_limitscale: float):
    # Convert from full gear torque to motor torque
    knee_action = actions[0] * trained_model_motor_torque_limitscale
    foot_action = actions[1] * trained_model_motor_torque_limitscale

    Knee_ODrive.set_input_torque_value(knee_action)
    Foot_ODrive.set_input_torque_value(-foot_action)


def rescale_actions(low, high, action):
    d = (high - low) / 2.0
    m = (high + low) / 2.0
    scaled_action = action * d + m

    return scaled_action

def run_control_loop(CTRL_HZ: int, DECIMATION_FACTOR: int, onnx_model: ort.InferenceSession, obs: np.array, actions_low: np.array, actions_high: np.array, Knee_ODrive: ODrive, Foot_ODrive: ODrive, motor_torque_scale: float, lock: threading.Lock):
    dt = 1.0 / CTRL_HZ
    policy_counter = 0
    while True:
        loop_start_time = time.perf_counter()

        if policy_counter % DECIMATION_FACTOR == 0:
            with lock:
                observations = obs

            actions = onnx_model.run(None, {"obs": observations})
            actions = actions[0][0]
            clamped_actions = np.clip(actions, -1.0, 1.0)
            rescaled_actions = rescale_actions(actions_low, actions_high, clamped_actions)
            send_joint_commands(rescaled_actions, Knee_ODrive, Foot_ODrive, motor_torque_scale)
        
        policy_counter += 1
        gc.collect()

        elapsed = time.perf_counter() - loop_start_time
        time.sleep(max(0.0, dt - elapsed))


def run_decimation_control_loop(CTRL_HZ: int, DECIMATION_FACTOR: int, onnx_model: ort.InferenceSession, obs: np.array, actions_low: np.array, actions_high: np.array, Knee_ODrive: ODrive, Foot_ODrive: ODrive, motor_torque_scale: float, lock: threading.Lock):
    dt = 1.0 / (CTRL_HZ/DECIMATION_FACTOR)
    while True:
        loop_start_time = time.perf_counter()
        with lock:
            observations = obs

        actions = onnx_model.run(None, {"obs": observations}) 
        actions = actions[0][0]
        clamped_actions = np.clip(actions, -1.0, 1.0)
        rescaled_actions = rescale_actions(actions_low, actions_high, clamped_actions)
        send_joint_commands(rescaled_actions, Knee_ODrive, Foot_ODrive,motor_torque_scale)

        gc.collect()

        elapsed = time.perf_counter() - loop_start_time
        time.sleep(max(0.0, dt - elapsed))

def flush_can_bus(bus: can.interface.Bus):
    while not (bus.recv(timeout=0) is None): pass

def keep_odrives_alive_by_sending_zero_pos(stop_flag: threading.Event, Knee_ODrive: ODrive, Foot_ODrive: ODrive):
    print("Keeping ODrives alive by sending zero pos!")
    while stop_flag.is_set() == False:
        Knee_ODrive.set_input_position_value(0.0)
        Foot_ODrive.set_input_position_value(0.0)

        stop_flag.wait(0.01)
    print("Stopped keeping ODrives Alive!")