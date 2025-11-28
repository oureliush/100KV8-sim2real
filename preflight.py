from super_useful_functions import *
from safty import *
import json
import time

#sorry to anyone who must go thru/read this later.. including myself

policy_torque = np.array([3.86, 3.86], dtype=np.float32)

def preflight_checks():
        
    OPCODE_READ = 0x00
    OPCODE_WRITE = 0x01

    positive_torque_limit_path = 'axis0.config.torque_soft_max'
    negative_torque_limit_path = 'axis0.config.torque_soft_min'


    # See https://docs.python.org/3/library/struct.html#format-characters
    format_lookup = {
        'bool': '?',
        'uint8': 'B', 'int8': 'b',
        'uint16': 'H', 'int16': 'h',
        'uint32': 'I', 'int32': 'i',
        'uint64': 'Q', 'int64': 'q',
        'float': 'f'
    }


    knee_active_errors = []
    knee_v_check_it_count = 0
    foot_v_check_it_count = 0

    #chatgpt wrote the error determination logic, def do not understand that
    error_flags = {
        "INITIALIZING": 0x1,
        "SYSTEM_LEVEL": 0x2,
        "TIMING_ERROR": 0x4,
        "MISSING_ESTIMATE": 0x8,
        "BAD_CONFIG": 0x10,
        "DRV_FAULT": 0x20,
        "MISSING_INPUT": 0x40,
        "DC_BUS_OVER_VOLTAGE": 0x100,
        "DC_BUS_UNDER_VOLTAGE": 0x200,
        "DC_BUS_OVER_CURRENT": 0x400,
        "DC_BUS_OVER_REGEN_CURRENT": 0x800,
        "CURRENT_LIMIT_VIOLATION": 0x1000,
        "MOTOR_OVER_TEMP": 0x2000,
        "INVERTER_OVER_TEMP": 0x4000,
        "VELOCITY_LIMIT_VIOLATION": 0x8000,
        "POSITION_LIMIT_VIOLATION": 0x10000,
        "WATCHDOG_TIMER_EXPIRED": 0x1000000,
        "ESTOP_REQUESTED": 0x2000000,
        "SPINOUT_DETECTED": 0x4000000,
        "BRAKE_RESISTOR_DISARMED": 0x8000000,
        "THERMISTOR_DISCONNECTED": 0x10000000,
        "CALIBRATION_ERROR": 0x40000000,
    }



    with open('flat_endpoints.json', 'r') as f:
        endpoint_data = json.load(f)
        endpoints = endpoint_data['endpoints']


    print("Preflight Checks")

    # Send read command to knee
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x00), # 0x00: Get_Version
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (knee_id << 5 | 0x00): # 0x00: Get_Version
            break

    import struct
    _, knee_hw_product_line, hw_version, hw_variant, knee_fw_major, knee_foot_fw_minor, knee_foot_fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

    # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
    assert endpoint_data['fw_version'] == f"{knee_fw_major}.{knee_foot_fw_minor}.{knee_foot_fw_revision}"
    assert endpoint_data['hw_version'] == f"{knee_hw_product_line}.{hw_version}.{hw_variant}"



    #Torque Check

    # Convert path to endpoint ID
    endpoint_id = endpoints[positive_torque_limit_path]['id']
    endpoint_type = endpoints[positive_torque_limit_path]['type']

    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass

    # Send read command
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (knee_id << 5 | 0x05): # 0x05: TxSdo
            break

    # Unpack and print reply
    _, _, _, knee_positive_torque_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)




    # Convert path to endpoint ID
    endpoint_id = endpoints[negative_torque_limit_path]['id']
    endpoint_type = endpoints[negative_torque_limit_path]['type']

    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass

    # Send read command
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (knee_id << 5 | 0x05): # 0x05: TxSdo
            break

    # Unpack and print reply
    _, _, _, knee_negative_torque_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)










    # Send read command to foot
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x00), # 0x00: Get_Version
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (foot_id << 5 | 0x00): # 0x00: Get_Version
            break

    import struct
    _, hw_product_line, hw_version, hw_variant, foot_fw_major, foot_fw_minor, foot_fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

    # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
    assert endpoint_data['fw_version'] == f"{foot_fw_major}.{foot_fw_minor}.{foot_fw_revision}"
    assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"



    #Torque Check

    # Convert path to endpoint ID
    endpoint_id = endpoints[positive_torque_limit_path]['id']
    endpoint_type = endpoints[positive_torque_limit_path]['type']

    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass

    # Send read command
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (foot_id << 5 | 0x05): # 0x05: TxSdo
            break

    # Unpack and print reply
    _, _, _, foot_positive_torque_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)




    # Convert path to endpoint ID
    endpoint_id = endpoints[negative_torque_limit_path]['id']
    endpoint_type = endpoints[negative_torque_limit_path]['type']

    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass

    # Send read command
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (foot_id << 5 | 0x05): # 0x05: TxSdo
            break

    # Unpack and print reply
    _, _, _, foot_negative_torque_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)


    # Send read command to knee
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x03), # 0x03: Get_Errors
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (knee_id << 5 | 0x03): # 0x03: Get_Errors
            break


    knee_error_code, knee_disarm_reason = struct.unpack('<II', msg.data)
    knee_active_errors = [name for name, bit in error_flags.items() if knee_error_code & bit]


    while "DC_BUS_UNDER_VOLTAGE" in knee_active_errors:

        if knee_v_check_it_count == 0:
            print("Waiting for battery to be connected, if you see this and the battery is connected, then it is dead.")

        # Send read command to knee
        bus.send(can.Message(
            arbitration_id=(knee_id << 5 | 0x03), # 0x03: Get_Errors
            data=b'',
            is_extended_id=False
        ))

        # Await reply
        for msg in bus:
            if msg.is_rx and msg.arbitration_id == (knee_id << 5 | 0x03): # 0x03: Get_Errors
                break


        knee_error_code, knee_disarm_reason = struct.unpack('<II', msg.data)
        knee_active_errors = [name for name, bit in error_flags.items() if knee_error_code & bit]

        knee_v_check_it_count =+ 1
        time.sleep(0.1)

        

    # Send read command to foot
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x03), # 0x03: Get_Errors
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (foot_id << 5 | 0x03): # 0x03: Get_Errors
            break


    foot_error_code, foot_disarm_reason = struct.unpack('<II', msg.data)
    foot_active_errors = [name for name, bit in error_flags.items() if foot_error_code & bit]

    while "DC_BUS_UNDER_VOLTAGE" in foot_active_errors:

        if foot_v_check_it_count == 0:
            print("If you see this then you have a wiring issue and the foot joint isnt receiving power")

        # Send read command to knee
        bus.send(can.Message(
            arbitration_id=(knee_id << 5 | 0x03), # 0x03: Get_Errors
            data=b'',
            is_extended_id=False
        ))

        # Await reply
        for msg in bus:
            if msg.is_rx and msg.arbitration_id == (knee_id << 5 | 0x03): # 0x03: Get_Errors
                break


        foot_error_code, foot_disarm_reason = struct.unpack('<II', msg.data)
        foot_active_errors = [name for name, bit in error_flags.items() if foot_error_code & bit]

        foot_v_check_it_count =+ 1
        time.sleep(0.1)



    print("battery good\n")


    test_mode = input("M - Mock Test\nR - Real Test\n\n")

    if test_mode == 'M' or test_mode == 'm':
        assert knee_positive_torque_value == 0, "Knee Torque Limit is a non zero value"
        assert knee_negative_torque_value == 0, "Knee -Torque Limit is a non zero value"

        assert foot_positive_torque_value == 0, "Foot Torque Limit is a non zero value"
        assert foot_negative_torque_value == 0, "Foot -Torque Limit is a non zero value"
        print("Torque Checks Okay")
    elif test_mode == 'R' or test_mode == 'r':
        assert knee_positive_torque_value != 0, "Knee Torque Limit is a zero value"
        assert knee_negative_torque_value != 0, "Knee -Torque Limit is a zero value"

        assert foot_positive_torque_value != 0, "Foot Torque Limit is a zero value"
        assert foot_negative_torque_value != 0, "Foot -Torque Limit is a zero value"
        print("Torque Checks Okay")
    else:
        print("Not an option! Assuming Operator Lacks BASIC Intellect and Shutting Down")
        quit()

    print("Attempting to clear errors")



    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x18), # 0x18: Clear Errors
        data=b'',
        is_extended_id=False
    ))

    # Send read command to knee
    bus.send(can.Message(
        arbitration_id=(knee_id << 5 | 0x03), # 0x03: Get_Errors
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (knee_id << 5 | 0x03): # 0x03: Get_Errors
            break


    knee_error_code, knee_disarm_reason = struct.unpack('<II', msg.data)
    knee_active_errors = [name for name, bit in error_flags.items() if knee_error_code & bit]


    if knee_active_errors == []:
        print("Watchdog is already being fed by another program. Quitting") # we expect watchdog to be the only error so if its not there thats a bad thing
        quit()
    elif "WATCHDOG_TIMER_EXPIRED" in knee_active_errors and len(knee_active_errors) != 1:
        print("Persistent Error Detected.")
        quit()




    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x18), # 0x18: Clear Errors
        data=b'',
        is_extended_id=False
    ))

    # Send read command to knee
    bus.send(can.Message(
        arbitration_id=(foot_id << 5 | 0x03), # 0x03: Get_Errors
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (foot_id << 5 | 0x03): # 0x03: Get_Errors
            break


    foot_error_code, foot_disarm_reason = struct.unpack('<II', msg.data)
    foot_active_errors = [name for name, bit in error_flags.items() if foot_error_code & bit]


    if foot_active_errors == []:
        print("Watchdog is already being fed by another program. Quitting") # we expect watchdog to be the only error so if its not there thats a bad thing
        quit()
    elif "WATCHDOG_TIMER_EXPIRED" in foot_active_errors and len(foot_active_errors) != 1:
        print("Persistent Error Detected.")
        quit()


