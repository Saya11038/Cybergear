import serial
import time
import struct
import math

frame_head = "4154" #hex_str
frame_tail = "0d0a" #hex_str

index_angle = "1670"
index_speed = "1770"
index_speed_mode = "0a70"
index_max_current = "1870"

index_dict = {
    "run_mode" : "0570",
    "iq_ref" : "0670",
    "id_ref" : "0770",
    "spd_ref" : "0a70",
    "limit_torque" : "0b70",
    "cur_kp" : "1070",
    "cur_ki" : "1170",
    "cur_filt_gain" : "1470",
    "loc_ref" : "1670",
    "limit_spd" : "1770",
    "limit_cur" : "1870",
    "mechPos" : "1970",
    "iqf" : "1a70",
    "mechVel" : "1b70",
    "VBUS" : "1c70",
    "rotation" : "1d70",
    "loc_kp" : "1e70",
    "spd_kp" : "1f70",
    "spd_ki" : "2070"
}

run_mode = {
    "motion_control" : 0,
    "location" : 1,
    "speed" : 2,
    "current" : 3
}


master_CANID = 0b0000000011111101 #bin_num
motor_CANID = 0b01111111 #bin_num

frame_enable_motor = 0b00000011
frame_stop_motor = 0b00000100
frame_read_param = 0b00010001
frame_write_param = 0b00010010
frame_homing_mode = 0b00000110
frame_motion_control = 0b00000001
frame_power_on = 0b00010011
frame_get_device = 0b00000000


class Cybergear:


    def __init__(self, master_can, motor_can):
        self.master = int_to_bin(master_can)
        self.motor = int_to_bin(motor_can)
        self.angle = 0.0


    def enable_motor(self):
        
        bin_num = frame_enable_motor << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:]
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)


    def stop_motor(self):

        bin_num = frame_stop_motor << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:]
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)


    def position_control(self, target_angle, speed):

        bin_num = frame_write_param << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr

        hex_angle_param = struct.unpack('!I', struct.pack('!f', target_angle))[0]
        hex_angle_param = reverse_hex(format(hex_angle_param, "08x"))
        #print(hex_angle_param)

        hex_speed_param = struct.unpack('!I', struct.pack('!f', speed))[0]
        hex_speed_param = reverse_hex(format(hex_speed_param, "08x"))
        #print(hex_speed_param)

        target_angle_hex = frame_head + hex_str + "08" + index_angle + "0000" + hex_angle_param + frame_tail
        target_speed_hex = frame_head + hex_str + "08" + index_speed + "0000" + hex_speed_param + frame_tail

        ser.write(bytes.fromhex(target_speed_hex))
        ser.flush()
        print(target_speed_hex)

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)

        ser.write(bytes.fromhex(target_angle_hex))
        ser.flush()
        print(target_angle_hex)

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)


    def homing_mode(self):

        bin_num = frame_homing_mode << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:]
        hex_can = frame_head + hex_str + "080100000000000000" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)


    def speed_control(self, target_speed, max_current):

        bin_num = frame_write_param << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr

        hex_speed_param = struct.unpack('!I', struct.pack('!f', target_speed))[0]
        hex_speed_param = reverse_hex(format(hex_speed_param, "08x"))
        #print(hex_angle_param)

        hex_max_current_param = struct.unpack('!I', struct.pack('!f', max_current))[0]
        hex_max_current_param = reverse_hex(format(hex_max_current_param, "08x"))
        #print(hex_speed_param)

        target_speed_hex = frame_head + hex_str + "08" + index_speed_mode + "0000" + hex_speed_param + frame_tail
        max_current_hex = frame_head + hex_str + "08" + index_max_current + "0000" + hex_max_current_param + frame_tail

        ser.write(bytes.fromhex(max_current_hex))
        print(max_current_hex)
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)

        ser.write(bytes.fromhex(target_speed_hex))
        print(target_speed_hex)
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)


    def read_param(self, index):

        bin_num = frame_read_param << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr

        ser.flushInput()
        ser.flushOutput()

        read_param = frame_head + hex_str + "08" + index + "000000000000" + frame_tail
        ser.write(bytes.fromhex(read_param))
        ser.flush()
        print(read_param)

        count = 0

        while True:

            data = ser.read_until(expected=b'\r\n')
            # if msg is not start with AT, it is not a valid msg
            counter = 0
            while not data.startswith(b'AT'):
                print("not start with AT received data:", data)
                data = ser.read_until(expected=b'\r\n')
                counter += 1
                if counter > 3:
                    return 1
            
            #print(data)
            data = int.from_bytes(data, "little")
            #print(data)
            received_data = reverse_hex("0"+hex(data)[2:])
            print(">>" + received_data)

            type_of_data = int(received_data[4:12], 16) >> 3
            #print(type_of_data)

            frame_type = type_of_data >> 24
            print(frame_type)
        
            if(frame_type == 17):
                self.get_single_param(received_data)
                return 0
            else:
                count += 1
            
            if count > 3:
                print("Error")
                return 1


    def get_motor_state(self, received_data):

        status = int(received_data[4:12], 16) >> 3

        frame_type = status >> 24

        if(frame_type != 2):
            print("Received data Error")
            return 1

        motor_can_id = status >> 8
        motor_can_id = bin(motor_can_id)[-8:]
        motor_can_id = int(motor_can_id, 2)

        under_voltage = status >> 16
        under_voltage = bin(under_voltage)[-1:]
        under_voltage = int(under_voltage, 2)

        over_current = status >> 17
        over_current = bin(over_current)[-1:]
        over_current = int(over_current, 2)

        over_temp = status >> 18
        over_temp = bin(over_temp)[-1:]
        over_temp = int(over_temp, 2)

        magnetic_encoding_failure = status >> 19
        magnetic_encoding_failure = bin(magnetic_encoding_failure)[-1:]
        magnetic_encoding_failure = int(magnetic_encoding_failure, 2)

        HALL_encoding_failure = status >> 20
        HALL_encoding_failure = bin(HALL_encoding_failure)[-1:]
        HALL_encoding_failure = int(HALL_encoding_failure, 2)

        not_calibrated = status >> 21
        not_calibrated = bin(not_calibrated)[-1:]
        not_calibrated = int(not_calibrated, 2)

        print("Motor ID: ", motor_can_id)

        if(under_voltage == 1):
            print("Under Voltage Fault")
        if(over_current == 1):
            print("Over Current")
        if(over_temp == 1):
            print("Over Temperature")
        if(magnetic_encoding_failure == 1):
            print("Magnetic Encoding Failure")
        if(HALL_encoding_failure == 1):
            print("HALL Encoding Failure")
        if(not_calibrated == 1):
            print("Not Calibrated")

        current_angle = int(received_data[14:18], 16)
        current_angle_vel = int(received_data[18:22], 16)
        current_torque = int(received_data[22:26], 16)
        current_temp = int(received_data[26:30], 16)

        angle = linear_mapping(current_angle, -4*math.pi, 4*math.pi)
        angle_vel = linear_mapping(current_angle_vel, -30.0, 30.0)
        torque = linear_mapping(current_torque, -12.0, 12.0)
        temp = current_temp / 10.0

        print("Angle:", angle, "rad")
        print("Angle Velocity:", angle_vel, "rad/s")
        print("Torque:", torque, "Nm")
        print("Temperature:", temp, "℃")

        self.angle = angle


    def get_single_param(self, received_data):
        
        can_id = int(received_data[4:12], 16) >> 3

        motor_can_id = can_id >> 8
        motor_can_id = bin(motor_can_id)[-8:]
        motor_can_id = int(motor_can_id, 2)

        print("Motor ID: ", motor_can_id)

        index = received_data[14:18]
        data = received_data[22:30]
        data = reverse_hex(data)
        print(data)

        if(index == index_dict["run_mode"]):
            print("Run_mode")
            return 1
        
        if(index == index_dict["rotation"]):
            print("Rotation")
            return 1
        
        for key, value in index_dict.items():
            if(value == index):
                param = hex_to_float(data)
                print(key, ":", param)
                return 0
        
        print("Error")
        return 1


    def set_run_mode(self, mode):

        bin_num = frame_write_param << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr

        for key in run_mode:
            if(key == mode):
                mode_data = run_mode[key]
                break
        
        hex_can = frame_head + hex_str + "0805700000" + "0" + str(mode_data) + "000000" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)


    def motion_control(self, torque, angle, velocity, kp, kd):

        torque_param = int(linear_mapping(torque, min_data=-12.0, max_data=12.0))
        target_angle = int(linear_mapping(angle, min_data=-4*math.pi, max_data=4*math.pi))
        target_velocity = int(linear_mapping(velocity, min_data=-30.0, max_data=30.0))
        kp_param = int(linear_mapping(kp, min_data=0.0, max_data=500.0))
        kd_param = int(linear_mapping(kd, min_data=0.0, max_data=5.0))

        bin_num = frame_motion_control << 24 | self.master << 8 | self.motor #2進数のまま結合 int_to_bin(torque_param) << 16 |
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:]

        hex_angle = hex(target_angle)[2:]
        hex_velocity = hex(target_velocity)[2:]
        hex_kp = hex(kp_param)[2:]
        hex_kd = hex(kd_param)[2:]

        hex_can = frame_head + hex_str + "08" + hex_angle + hex_velocity + hex_kp + hex_kd + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)


    def power_on(self):

        ser.write(bytes.fromhex('41 54 2b 41 54 0d 0a'))
        ser.flush()
        print('41542b41540d0a')

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        bin_num = self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_can = frame_head + "000" + hex_str + "0100" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        bin_num = frame_power_on << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_can = frame_head + hex_str + "084066313130333103" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        # self.get_motor_state(received_data)


    def current_control(self, iq, id = 0.0):

        bin_num = frame_write_param << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr

        hex_iq_param = struct.unpack('!I', struct.pack('!f', iq))[0]
        hex_iq_param = reverse_hex(format(hex_iq_param, "08x"))
        #print(hex_angle_param)

        hex_id_param = struct.unpack('!I', struct.pack('!f', id))[0]
        hex_id_param = reverse_hex(format(hex_id_param, "08x"))
        #print(hex_speed_param)

        target_iq_hex = frame_head + hex_str + "08" + index_dict["iq_ref"] + "0000" + hex_iq_param + frame_tail
        target_id_hex = frame_head + hex_str + "08" + index_dict["id_ref"] + "0000" + hex_id_param + frame_tail

        ser.write(bytes.fromhex(target_iq_hex))
        ser.flush()
        print(target_iq_hex)

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)

        ser.write(bytes.fromhex(target_id_hex))
        ser.flush()
        print(target_id_hex)

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        print(">>" + received_data)

        self.get_motor_state(received_data)



#入力された整数値を2進数の数値に変える関数
def int_to_bin(num_deci):

    num = 0
    num_bin = 0b0

    while num_deci > 1:
        if num_deci % 2 == 0:
            num_deci = int(num_deci / 2)
            num_bin = num_bin | 0b0 << num
        else:
            num_deci = int(num_deci / 2)
            num_bin = num_bin | 0b1 << num
        num += 1

    if num_deci == 0:
        num_bin = num_bin | 0b0 << num
    elif num_deci == 1:
        num_bin = num_bin | 0b1 << num

    return num_bin


#小数の小数部分を23bitの２進数の数値に変換
def decimal_to_binary_fraction(decimal_fraction):
    i = 0
    binary_fraction = 0b0
    while decimal_fraction != 0:
        decimal_fraction *= 2
        if decimal_fraction >= 1:
            binary_fraction = binary_fraction << 1 | 0b1
            decimal_fraction -= 1
        else:
            binary_fraction = binary_fraction << 1 | 0b0
        i += 1
        if i >= 23:  # 23ビットまで丸める
            break
    return binary_fraction


#16進数の数字を2文字セットで逆向きにする
def reverse_hex(hex_string):
     # 文字列を2文字ずつに分割してリストに格納
    pairs = [hex_string[i:i+2] for i in range(0, len(hex_string), 2)] 
    # 逆順に並べ替えて結合
    result = ''.join(pairs[::-1])  
    return result


def linear_mapping(data, min_num = 0.0, max_num = 65535.0, min_data = 0.0, max_data = 65535.0):
    return (data - min_data + 1.0) / (max_data - min_data + 1.0) * (max_num - min_num) + min_num


def hex_to_float(hex_str):
    # 16進数文字列をバイト列に変換
    byte_array = bytes.fromhex(hex_str)    
    # バイト列を浮動小数点数に変換
    float_num = struct.unpack('!f', byte_array)[0]
    return float_num



forward = '41 54 90 07 eb fc 08 05 70 00 00 07 01 95 54 0d 0a'
stop = '41 54 90 07 eb fc 08 05 70 00 00 00 00 00 00 0d 0a'

halt = '41 54 a0 07 eb fc 08 00 00 00 00 00 00 00 00 0d 0a'

start_position_control_1 = '41 54 90 07 eb fc 08 05 70 00 00 04 00 00 00 0d 0a'
start_position_control_2 = '41 54 18 07 eb fc 08 00 00 00 00 00 00 00 00 0d 0a'

run_mode_position = '41 54 90 07 eb fc 08 05 70 00 00 01 00 00 00 0d 0a'

#enable_motor = '41541807ebfc0800000000000000000d0a'
#stop_motor = '41 54 20 07 eb fc 08 00 00 00 00 00 00 00 00 0d 0a'

# ser.write(bytes.fromhex(forward))

# time.sleep(10)


# ser.write(bytes.fromhex(stop))

# a = '1A'
# print(bytes.fromhex(stop))

# ser.write(bytes.fromhex(enable_motor))

# # #position_control(10, 10)

# time.sleep(10)

#ser.write(bytes.fromhex(stop_motor))
# frame_id_bin_str = format(frame_enable_motor, "08b")
# print(frame_id_bin_str)

#'41 54 00 07 ea 44 01 00 0d 0a'

# # シリアルポートとボーレートを設定
ser = serial.Serial('COM3', 921600, timeout = 2.0)


motor_1 = Cybergear(253, 126)
motor_2 = Cybergear(253, 127)

motor_1.power_on()
motor_2.power_on()

motor_1.set_run_mode("current")
motor_2.set_run_mode("current")

motor_1.enable_motor()
motor_2.enable_motor()

motor_1.homing_mode()
motor_2.homing_mode()

#motor_1.read_param(index_dict["mechVel"])

#motor_1.motion_control(3.0, 3.0, 3.0, 10.0, 1.0)

#time.sleep(5)

motor_1.current_control(0.3)
motor_2.current_control(0.3)
#speed_control(master_CANID, motor_CANID, 3.0, 20.0)

time.sleep(10)

#position_control(master_CANID, motor_CANID, 6.0, 4.0)
#speed_control(master_CANID, motor_CANID, -3.0, 20.0)

# time.sleep(1)

#motor_1.read_param(index_dict["mechVel"])

motor_1.current_control(-0.3)
motor_2.current_control(-0.3)

time.sleep(5)

# data = "41541403ffec0880007ff07fff01390d0a"
# get_motor_state(data, master_CANID, motor_CANID)

motor_1.stop_motor()
motor_2.stop_motor()




# def decimal_to_float_binary(decimal_value):
#     # 浮動小数点数に変換
#     float_value = float(decimal_value)
#     # 浮動小数点数をバイナリデータに変換
#     binary_data = struct.pack('f', float_value)
#     # バイナリデータを16進数文字列に変換
#     hex_str = binary_data.hex()
#     # 16進数文字列を2進数文字列に変換
#     binary_str = bin(int(hex_str, 16))[2:]
#     return binary_str

# # テスト用の10進数の値
# decimal_value = 3.14

# # 浮動小数点数の2進数表現を取得
# binary_str = decimal_to_float_binary(decimal_value)

# # 結果の表示
# print("浮動小数点数の2進数表現:", binary_str)


# i = bin(128)
# print(i)
# deci = int(i, 2)
# print(deci)

# i = 15
# print(bin(int_to_bin(i)))


# 0.1を23ビットの2進数に変換する例
# decimal_fraction = 0.1
# binary_fraction = decimal_to_binary_fraction(decimal_fraction)
# print(bin(binary_fraction))


# target_angle = 3.0
# # hex_value = struct.unpack('!I', struct.pack('!f', float_value))[0]
# # print(type(hex_value))
# # print(f"Hex value of 3.0: {hex(hex_value)}")

# hex_angle_param = struct.unpack('!I', struct.pack('!f', target_angle))[0]
# hex_angle_param = reverse_hex(format(hex_angle_param, "08x"))
# print(hex_angle_param)