import struct

import can

import logging

import enum

import math

import time




class CANMotorController:

    PARAM_TABLE = {

        "motorOverTemp": {"feature_code": 0x200D, "type": "int16"},

        "overTempTime": {"feature_code": 0x200E, "type": "int32"},

        "limit_torque": {"feature_code": 0x2007, "type": "float"},

        "cur_kp": {"feature_code": 0x2012, "type": "float"},

        "cur_ki": {"feature_code": 0x2013, "type": "float"},

        "spd_kp": {"feature_code": 0x2014, "type": "float"},

        "spd_ki": {"feature_code": 0x2015, "type": "float"},

        "loc_kp": {"feature_code": 0x2016, "type": "float"},

        "spd_filt_gain": {"feature_code": 0x2017, "type": "float"},

        "limit_spd": {"feature_code": 0x2018, "type": "float"},

        "limit_cur": {"feature_code": 0x2019, "type": "float"},

    }


    PARAMETERS = {

        "run_mode": {"index": 0x7005, "format": "u8"},

        "iq_ref": {"index": 0x7006, "format": "f"},

        "spd_ref": {"index": 0x700A, "format": "f"},

        "limit_torque": {"index": 0x700B, "format": "f"},

        "cur_kp": {"index": 0x7010, "format": "f"},

        "cur_ki": {"index": 0x7011, "format": "f"},

        "cur_filt_gain": {"index": 0x7014, "format": "f"},

        "loc_ref": {"index": 0x7016, "format": "f"},

        "limit_spd": {"index": 0x7017, "format": "f"},

        "limit_cur": {"index": 0x7018, "format": "f"},

        "rotation": {"index": 0x701D, "format": "s16"},

    }

    TWO_BYTES_BITS = 16


    def __init__(self, serial, motor_id=127, main_can_id=254):

        self.serial = serial

        self.MOTOR_ID = motor_id

        self.MAIN_CAN_ID = main_can_id

        self.P_MIN = -12.5

        self.P_MAX = 12.5

        self.V_MIN = -30.0

        self.V_MAX = 30.0

        self.T_MIN = -12.0

        self.T_MAX = 12.0

        self.KP_MIN, self.KP_MAX = 0.0, 500.0  # 0.0 ~ 500.0

        self.KD_MIN, self.KD_MAX = 0.0, 5.0  # 0.0 ~ 5.0


    class CmdModes:

        GET_DEVICE_ID = 0

        MOTOR_CONTROL = 1

        MOTOR_FEEDBACK = 2

        MOTOR_ENABLE = 3

        MOTOR_STOP = 4

        SET_MECHANICAL_ZERO = 6

        SET_MOTOR_CAN_ID = 7

        PARAM_TABLE_WRITE = 8

        SINGLE_PARAM_READ = 17

        SINGLE_PARAM_WRITE = 18

        FAULT_FEEDBACK = 21


    class RunModes(enum.Enum):

        CONTROL_MODE = 0

        POSITION_MODE = 1

        SPEED_MODE = 2

        CURRENT_MODE = 3


    def _uint_to_float(self, x, x_min, x_max, bits):

        span = (1 << bits) - 1

        offset = x_max - x_min

        x = max(min(x, span), 0)  # Clamp x to the range [0, span]

        return offset * x / span + x_min


    def _linear_mapping(

        self, value, value_min, value_max, target_min=0, target_max=65535

    ):

        return int(

            (value - value_min) / (value_max -

                                   value_min) * (target_max - target_min)

            + target_min

        )


    def format_data(self, data=[], format="f f", type="decode"):

        format_list = format.split()

        rdata = []

        if type == "decode":

            p = 0

            for f in format_list:

                s_f = []

                if f == "f":

                    s_f = [4, "f"]

                elif f == "u16":

                    s_f = [2, "H"]

                elif f == "s16":

                    s_f = [2, "h"]

                elif f == "u32":

                    s_f = [4, "I"]

                elif f == "s32":

                    s_f = [4, "i"]

                elif f == "u8":

                    s_f = [1, "B"]

                elif f == "s8":

                    s_f = [1, "b"]

                ba = bytearray()

                if len(s_f) == 2:

                    for i in range(s_f[0]):

                        ba.append(data[p])

                        p = p + 1

                    rdata.append(struct.unpack(s_f[1], ba)[0])

                else:

                    logging.info("unknown format in format_data(): " + f)

                    return []

            return rdata

        elif type == "encode" and len(format_list) == len(data):

            for i in range(len(format_list)):

                f = format_list[i]

                s_f = []

                if f == "f":

                    s_f = [4, "f"]

                elif f == "u16":

                    s_f = [2, "H"]

                elif f == "s16":

                    s_f = [2, "h"]

                elif f == "u32":

                    s_f = [4, "I"]

                elif f == "s32":

                    s_f = [4, "i"]

                elif f == "u8":

                    s_f = [1, "B"]

                elif f == "s8":

                    s_f = [1, "b"]

                if f != "f":

                    data[i] = int(data[i])

                if len(s_f) == 2:

                    bs = struct.pack(s_f[1], data[i])

                    for j in range(s_f[0]):

                        rdata.append(bs[j])

                else:

                    logging.info("unkown format in format_data(): " + f)

                    return []

            if len(rdata) < 4:

                for i in range(4 - len(rdata)):

                    rdata.append(0x00)

            return rdata


    def send_receive_can_message(self, cmd_mode, data2, data1):

        """

        send message to motor and receive message from motor

        """

        can_id = (cmd_mode << 27) | (data2 << 11) | (self.MOTOR_ID << 3) | (0x01 << 2)

        # print("can_mode:", cmd_mode)

        # print("data2:", data2)

        # print("MOTOR_ID:", self.MOTOR_ID)

        # # print can_id in hex

        # print("can_id in hex:", hex(can_id))


        # print("data1:", data1)


        message = self.construce_AT_message(can_id, data1)

        # print("sent message:", message.hex())

        # Send the bytearray message to the serial port

        self.serial.write(message)

        self.serial.flush()


        received_msg = self.serial.read_until(expected=b'\r\n')

        # if msg is not start with AT, it is not a valid msg


        counter = 0

        while not received_msg.startswith(b'AT'):

            # print("not start with AT received_msg:", received_msg)

            received_msg = self.serial.read_until(expected=b'\r\n')

            counter += 1

            if counter > 3:

                return None


        # print("received_msg:", received_msg.hex())

        return received_msg


    def construce_AT_message(self, can_id, data1):

        can_id = struct.pack(">I", can_id)

        # print("can_id:", can_id)

        data1 = bytearray(data1)

        len_data1 = len(data1)

        # len_data1 to 1 byte

        len_data1 = len_data1.to_bytes(1, byteorder='big')


        msg = bytearray()

        msg.append(0x41)

        msg.append(0x54)

        msg += can_id

        msg += len_data1

        msg += data1

        msg.append(0x0D)

        msg.append(0x0A)


        return msg


    def parse_received_msg(self, msg):

        if msg is None:

            return None, None

        data = None

        msg = msg[2:-2]

        arbitration_id = msg[:4]

        data = msg[5:]

        # print("arbitration_id:", arbitration_id.hex())

        # print("data:", data.hex())

        if data is not None:

            arbitration_id = int.from_bytes(arbitration_id, byteorder='big')

            motor_id = (arbitration_id >> 11) & 0xFF

            # print("motor_id:", motor_id)


            data = data[-4:]

            # print("data:", data.hex())


        return motor_id, data


    def _write_single_param(self, index, value, format="u32"):

        encoded_data = self.format_data(

            data=[value], format=format, type="encode")

        data1 = [b for b in struct.pack("<I", index)] + encoded_data


        # self.clear_can_rx()  # 空CAN接收缓存, 避免读到老数据

        self.serial.flush()

        received_msg_data =self.send_receive_can_message(

                                                            cmd_mode=self.CmdModes.SINGLE_PARAM_WRITE,

                                                            data2=self.MAIN_CAN_ID,

                                                            data1=data1,

                                                        )

        return self.parse_received_msg(received_msg_data)


    def write_single_param(self, param_name, value):

        self.serial.flush()

        param_info = self.PARAMETERS.get(param_name)

        if param_info is None:

            logging.info(f"Unknown parameter name: {param_name}")

            return


        index = param_info["index"]

        format = param_info["format"]

        # print("index:", index)

        # print("format:", format)


        return self._write_single_param(index=index, value=value, format=format)


    def _read_single_param(self, index, format="u32"):

        value = 0

        encoded_data = self.format_data(

            data=[value], format=format, type="encode")

        data1 = [b for b in struct.pack("<I", index)] + encoded_data


        self.serial.flush()

        received_msg_data =self.send_receive_can_message(

                                                            cmd_mode=self.CmdModes.SINGLE_PARAM_READ,

                                                            data2=self.MAIN_CAN_ID,

                                                            data1=data1,

                                                        )

        return self.parse_received_msg(received_msg_data)


    def read_single_param(self, param_name):

        self.serial.flush()

        param_info = self.PARAMETERS.get(param_name)

        if param_info is None:

            logging.info(f"Unknown parameter name: {param_name}")

            return


        index = param_info["index"]

        format = param_info["format"]

        # print("index:", index)

        # print("format:", format)


        return self._read_single_param(index=index, format=format)


    def get_motor_rots(self):

        motor_id, data = self.read_single_param("rotation")

        if data is not None:

            data = self.format_data(data[:2], "s16", "decode")[0]

            # print(f"{data}", end='\t')

            return motor_id, data

        return None, None


    def write_param_table(self, param_name, value):

        # Get the parameter info from PARAM_TABLE

        param_info = self.PARAM_TABLE.get(param_name)

        if param_info is None:

            logging.info(f"Unknown parameter name: {param_name}")

            return None, None, None


        feature_code = param_info["feature_code"]

        param_type = param_info["type"]


        # Type code mapping

        type_code_mapping = {

            "float": 0x06,

            "int16": 0x03,

            "int32": 0x04,

        }


        type_code = type_code_mapping.get(param_type)

        if type_code is None:

            logging.info(f"Unknown parameter type: {param_type}")

            return None, None, None


        # Encode the value based on the type

        format_mapping = {

            "float": "f",

            "int16": "s16",

            "int32": "s32",

        }


        format = format_mapping.get(param_type)

        encoded_value = self.format_data(

            data=[value], format=format, type="encode")


        # Construct data1

        data1 = [b for b in struct.pack("<H", feature_code)]

        data1.extend([type_code, 0x00])

        data1.extend(encoded_value)


        # Clear the CAN receive buffer

        # self.clear_can_rx()


        # Send the CAN message

        cmd_mode = self.CmdModes.PARAM_TABLE_WRITE

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(

            cmd_mode=cmd_mode,

            data2=self.MAIN_CAN_ID,

            data1=data1,

        )


        return received_msg_data, received_msg_arbitration_id


    def set_0_pos(self):

        # self.clear_can_rx()  # 清空CAN接收缓存, 避免读到老数据


        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(

            cmd_mode=self.CmdModes.SET_MECHANICAL_ZERO,

            data2=self.MAIN_CAN_ID,

            data1=[1],

        )


        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)


    def enable(self):

        # self.clear_can_rx(0)  # 清空CAN接收缓存, 避免读到老数据


        # received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(

        self.send_receive_can_message(

            cmd_mode=self.CmdModes.MOTOR_ENABLE, data2=self.MAIN_CAN_ID, data1=[0, 0, 0, 0, 0, 0, 0, 0]

        )

        # return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)


    def disable(self):

        # received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(

        self.send_receive_can_message(

            cmd_mode=self.CmdModes.MOTOR_STOP,

            data2=self.MAIN_CAN_ID,

            data1=[0, 0, 0, 0, 0, 0, 0, 0],

        )

        # return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)


    def set_run_mode(self, mode):

        if not isinstance(mode, self.RunModes):

            raise ValueError(

                f"Invalid mode: {mode}. Must be an instance of RunModes enum.")

        return self.write_single_param("run_mode", value=mode.value)


    def set_motor_position_control(self, limit_spd, loc_ref):

        self.write_single_param(param_name="limit_spd", value=limit_spd)

        self.write_single_param(param_name="loc_ref", value=loc_ref)


    def send_motor_control_command(

        self, torque, target_angle, target_velocity, Kp, Kd

    ):

        cmd_mode = self.CmdModes.MOTOR_CONTROL

        torque_mapped = self._linear_mapping(

            torque, -12.0, 12.0, target_min=0, target_max=65535)

        data2 = torque_mapped


        target_angle_mapped = self._linear_mapping(

            target_angle, -4 * math.pi, 4 * math.pi)


        target_velocity_mapped = self._linear_mapping(

            target_velocity, -30.0, 30.0)

        Kp_mapped = self._linear_mapping(Kp, 0.0, 500.0)

        Kd_mapped = self._linear_mapping(Kd, 0.0, 5.0)


        # data1_tmp = bytearray()

        # data1_tmp += target_angle_mapped.to_bytes(2, byteorder='big')

        # data1_tmp += target_velocity_mapped.to_bytes(2, byteorder='big')

        # data1_tmp += Kp_mapped.to_bytes(2, byteorder='big')

        # data1_tmp += Kd_mapped.to_bytes(2, byteorder='big')

        # print("data1_tmp:", data1_tmp)




        data1_bytes = struct.pack(

            "HHHH", ((target_angle_mapped & 0xff) << 8)|( target_angle_mapped >> 8 ),

            ((target_velocity_mapped & 0xff) << 8)|( target_velocity_mapped >> 8 ),

            ((Kp_mapped & 0xff) << 8)|( Kp_mapped >> 8 ),

            ((Kd_mapped & 0xff) << 8)|( Kd_mapped >> 8 )

        )

        data1 = [b for b in data1_bytes]


        # received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(

        self.send_receive_can_message(

            cmd_mode=cmd_mode,

            data2=data2,

            data1=data1

        )


        # return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)


