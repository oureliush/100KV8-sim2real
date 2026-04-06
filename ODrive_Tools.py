import struct
import can
import json

#TODO allow script to automagically determine odrive version and type and 
# figure out what flat_endpoints file to download
# add warnings if multiple odrives are initalized and versions dont match

# add find odrive by serial number

#make this a base class then make ODrive S1 and ODrive Pro classes for future proofing


with open('flat_endpoints.json', 'r') as f:
    endpoint_data = json.load(f)
    endpoints = endpoint_data['endpoints']

OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

# See https://docs.python.org/3/library/struct.html#format-characters
format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

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


class ODrive():

    def __init__(self, bus, node_id):
        self.bus = bus
        self.node_id = node_id

        while not (self.bus.recv(timeout=0) is None): pass

        # Send read command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x00), # 0x00: Get_Version
            data=b'',
            is_extended_id=False
        ))

        # Await reply
        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.node_id << 5 | 0x00): # 0x00: Get_Version
                break
        
        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

        # these check if the ODrive Hardware and Firmware Verison match the endpoints file
        assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
        assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"



    def clear_errors(self, identify: int =0):
        if identify == 0:
            self.bus.send(can.Message(
                arbitration_id = (self.node_id << 5 | 0x18),
                data=struct.pack('<I', identify),
                is_extended_id=False
            ))
        elif identify == 1:
            self.bus.send(can.Message(
                arbitration_id = (self.node_id << 5 | 0x18),
                data=struct.pack('<I', identify),
                is_extended_id=False
            ))
        else:
            print("Identify Variable Must be 1 or 0 ")

    def write_parameter(self, path, value):
        endpoint_id = endpoints[path]['id']
        endpoint_type = endpoints[path]['type']

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB' + format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value),
            is_extended_id=False
        ))


        
    def read_parameter(self, path):
        endpoint_id = endpoints[path]['id']
        endpoint_type = endpoints[path]['type']

        while not (self.bus.recv(timeout=0) is None): pass

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))

        # Await reply
        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.node_id << 5 | 0x05): # 0x05: TxSdo
                break

        # Unpack and print reply
        _, _, _, return_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)

        return return_value
    

    def set_axis_state(self, requested_state):
        while not (self.bus.recv(timeout=0) is None): pass

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x07),  # 0x07: Set_Axis_State
            data=struct.pack('<I', requested_state),  # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False
        ))

        for msg in self.bus:
            if msg.arbitration_id == (self.node_id << 5 | 0x01):  # 0x01: Heartbeat
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == requested_state:  # 8: AxisState.CLOSED_LOOP_CONTROL
                    break

    def set_closed_loop_control(self):
        self.set_axis_state(8)


    def set_idle(self):
        self.set_axis_state(1)


    def set_controller_mode(self, control_mode, input_mode):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0b), # 0x04: RxSdo
            data=struct.pack('<II', control_mode, input_mode),
            is_extended_id=False
        ))

    def set_position_control(self, input_mode=1):
        self.set_controller_mode(3, input_mode)

    def set_velocity_control(self, input_mode=1):
        self.set_controller_mode(2, input_mode)

    def set_torque_control(self, input_mode=1):
        self.set_controller_mode(1, input_mode)
        
    def set_input_position_value(self, pos_value: float):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0c),
            data=struct.pack('<f', pos_value),
            is_extended_id=False
        ))

    def set_input_velocity_value(self, vel_value: float, input_torque_ff=0.001): 
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0d),
            data=struct.pack('<ff', vel_value, input_torque_ff),
            is_extended_id=False
        ))
    
    def set_input_torque_value(self, torque_value: float):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0e),
            data=struct.pack('<f', torque_value),
            is_extended_id=False
        ))

    def set_absolute_pos(self, pos: float):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x19),
            data=struct.pack('<f', pos),
            is_extended_id=False
        ))

    
    def get_measured_motor_current_test(self):
        return self.read_parameter('axis0.motor.foc.Iq_measured')
    
    def get_measured_motor_torque_test(self):
        return self.read_parameter('axis0.motor.torque_estimate')
    
    def reboot(self, action):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x19),
            data=struct.pack('<B', action),
            is_extended_id=False
        ))

    def get_errors(self):
        while not (self.bus.recv(timeout=0) is None): pass

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x03), # 0x03: Get_Errors
            data=b'',
            is_extended_id=False
        ))

        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.node_id << 5 | 0x03): # 0x03: Get_Errors
                break

        errors, disarm_reason = struct.unpack('<II', msg.data)
        active_errors = [name for name, bit in error_flags.items() if errors & bit]

        return [active_errors, disarm_reason]


    
    '''
    def get_measured_current(self):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x14),
            data='',
            is_extended_id= False
        ))

        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.bus << 5 | 0x14): # 0x14: Get_IQ
                iq_setpoint, iq_measured = struct.unpack_from('<FF', msg.data)
                break

        return iq_measured, iq_setpoint
            
    def get_torques(self):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x1c),
            data='',
            is_extended_id= False
        ))

        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.bus << 5 | 0x1c): # 0x1c  Get_IQ
                torque_target, torque_estimate = struct.unpack_from('<FF', msg.data)
                break

        return torque_estimate, torque_target
        '''