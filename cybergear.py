import serial
import time
import struct
import math
import pandas as pd
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


frame_enable_motor = 3
frame_stop_motor = 4
frame_read_param = 17
frame_write_param = 18
frame_homing_mode = 6
frame_motion_control = 1
frame_power_on = 19
frame_get_device = 0
frame_set_canid = 7


<<<<<<< HEAD
ser = serial.Serial('/dev/ttyUSB0', 921600, timeout = 2.0)
=======
ser = serial.Serial('COM3', 921600, timeout = 0.5)
>>>>>>> daff9e060ba9a53c4ed6c6c001c28da69d5a6457


class Cybergear:


    def __init__(self, master_can, motor_can, torque_diff_max=1.0, torque_diff_min=0.5, torque_const=0.0):
        self.master = master_can
        self.motor = motor_can
        self.angle = 0.0
        self.torque_before = 0.0
        self.torque = 0.0
        self.torque_gap = 0.0
        self.torque_diff_max = torque_diff_max
        self.torque_diff_min = torque_diff_min
        self.torque_const = torque_const
        self.real_torque = self.torque - torque_const * self.angle


    def enable_motor(self):
        
        bin_num = frame_enable_motor << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        # hex_str = hex(bin_num)[2:]
        hex_str = format(bin_num, "08x")
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
        # hex_str = hex(bin_num)[2:]
        hex_str = format(bin_num, "08x")
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
        # hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_str = format(bin_num, "08x")

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
        # print(target_speed_hex)

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        self.get_motor_state(received_data)

        ser.write(bytes.fromhex(target_angle_hex))
        ser.flush()
        # print(target_angle_hex)

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        self.get_motor_state(received_data)


    def homing_mode(self):

        bin_num = frame_homing_mode << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        # hex_str = hex(bin_num)[2:]
        hex_str = format(bin_num, "08x")
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
        # hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_str = format(bin_num, "08x")

        hex_speed_param = struct.unpack('!I', struct.pack('!f', target_speed))[0]
        hex_speed_param = reverse_hex(format(hex_speed_param, "08x"))
        #print(hex_angle_param)

        hex_max_current_param = struct.unpack('!I', struct.pack('!f', max_current))[0]
        hex_max_current_param = reverse_hex(format(hex_max_current_param, "08x"))
        #print(hex_speed_param)

        target_speed_hex = frame_head + hex_str + "08" + index_speed_mode + "0000" + hex_speed_param + frame_tail
        max_current_hex = frame_head + hex_str + "08" + index_max_current + "0000" + hex_max_current_param + frame_tail

        ser.write(bytes.fromhex(max_current_hex))
        # print(max_current_hex)
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        self.get_motor_state(received_data)

        ser.write(bytes.fromhex(target_speed_hex))
        # print(target_speed_hex)
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        self.get_motor_state(received_data)


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

        # print("Motor ID: ", motor_can_id)

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

        # print("Angle:", angle, "rad")
        # print("Angle Velocity:", angle_vel, "rad/s")
        # print("Torque:", torque, "Nm")
        # print("Temperature:", temp, "℃")

        self.angle = angle
        self.torque_before = self.torque
        self.torque = torque
        self.torque_gap = abs(self.torque - self.torque_before)
        self.real_torque = self.torque - self.torque_const * self.angle


    def set_run_mode(self, mode):

        bin_num = frame_write_param << 24 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        # hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_str = format(bin_num, "08x")

        for key in run_mode:
            if(key == mode):
                mode_data = run_mode[key]
                break
        
        hex_can = frame_head + hex_str + "0805700000" + "0" + str(mode_data) + "000000" + frame_tail
        # print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

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
        # hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_str = format(bin_num, "08x")
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
        # hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_str = format(bin_num, "08x")

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


    # 繰り返し高速化のためget_motor_state()の最低限だけを残した
    def get_motor_angle(self, received_data):

        current_angle = int(received_data[14:18], 16)
        angle = linear_mapping(current_angle, -4*math.pi, 4*math.pi)
        self.angle = angle

        current_torque = int(received_data[22:26], 16)
        torque = linear_mapping(current_torque, -12.0, 12.0)
        self.torque_before = self.torque
        self.torque = torque
        self.torque_gap = abs(self.torque - self.torque_before)
        self.real_torque = self.torque - self.torque_const * self.angle


    def update_state(self):

        bin_num = frame_homing_mode << 24 | self.motor << 16 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        # print(hex(bin_num))
        # hex_str = hex(bin_num)[2:]
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "00" + frame_tail
        # print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        self.get_motor_angle(received_data)

    
    # CANIDを変更する関数
    def reset_canid(self, new_id):

        bin_num = frame_set_canid << 24 | new_id << 16 | self.master << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        # print(hex(bin_num))
        # hex_str = hex(bin_num)[2:]
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "08" + "369a313130333109" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        # data = ser.read_until(expected=b'\r\n')
        # data = int.from_bytes(data, "little")
        # received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        self.motor = new_id

        # self.get_motor_angle(received_data)


        bin_num = frame_power_on << 24 | 0b11111111 << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        # print(hex(bin_num))
        # hex_str = hex(bin_num)[2:]
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "08" + "369a313130333109" + frame_tail
        print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        # data = ser.read_until(expected=b'\r\n')
        # data = int.from_bytes(data, "little")
        # received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        # self.get_motor_angle(received_data)


    # 連続でデータを送って動きを制御するときに使う関数
    def motion_control(self, angle, velocity, kp, kd, torque = 0.0):

        torque_param = int(linear_mapping(torque, min_data=-12.0, max_data=12.0))
        target_angle = int(linear_mapping(angle, min_data=-4*math.pi, max_data=4*math.pi))
        target_velocity = int(linear_mapping(velocity, min_data=-30.0, max_data=30.0))
        kp_param = int(linear_mapping(kp, min_data=0.0, max_data=500.0))
        kd_param = int(linear_mapping(kd, min_data=0.0, max_data=5.0))

        bin_num = frame_motion_control << 24 | torque_param << 8 | self.motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        # print(hex(bin_num))
        # hex_str = hex(bin_num)[2:]
        hex_str = format(bin_num, "08x")
        # print(hex_str)

        hex_angle = format(target_angle, "04x")
        hex_velocity = format(target_velocity, "04x")
        hex_kp = format(kp_param, "04x")
        hex_kd = format(kd_param, "04x")

        hex_can = frame_head + hex_str + "08" + hex_angle + hex_velocity + hex_kp + hex_kd + frame_tail
        # print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        data = int.from_bytes(data, "little")
        received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

        self.get_motor_state(received_data)



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
    # return (data - min_data + 1.0) / (max_data - min_data + 1.0) * (max_num - min_num) + min_num
    return (data - min_data) / (max_data - min_data) * (max_num - min_num) + min_num


def hex_to_float(hex_str):
    # 16進数文字列をバイト列に変換
    byte_array = bytes.fromhex(hex_str)    
    # バイト列を浮動小数点数に変換
    float_num = struct.unpack('!f', byte_array)[0]
    return float_num


# 指定された秒数motor_arrayに格納されたモーターの状態をcsvに記録する
def upload_to_csv(motor_array, seconds, file_path):

    start_time = time.time()

    try:
        df = pd.read_csv(file_path)
    except (FileNotFoundError, pd.errors.EmptyDataError):
        # ファイルが存在しない場合、空のデータフレームを作成
        df = pd.DataFrame(columns=['time','1', '2', '3', '4'])  # ヘッダーを指定

    collected_data = []

    while time.time() - start_time < seconds:

        new_data = {}
        new_data['time'] = time.time() - start_time
        num = 1

        for i in motor_array:
            i.update_state()
            new_data[str(num)] = i.angle
            num += 1

        collected_data.append(new_data)

    # データを指定した列に追加
    df = pd.concat([df, pd.DataFrame(collected_data)], ignore_index=True)

    # 更新したデータフレームをCSVに書き込む
    df.to_csv(file_path, index=False)


# 与えられたCSVファイルの動きを再現する
def replicate(motor_array, file_path):

    df = pd.read_csv(file_path)
    data_list = df.values.tolist()
    
    for motor in motor_array:
        motor.set_run_mode("location")
        motor.enable_motor()

    start_time = time.time()

    for motor_list in data_list:
        if time.time() - start_time > motor_list[0]:
            continue
        else:
            num = 1
            for motor in motor_array:
                motor.position_control(motor_list[num], 8.0)
                num += 1


# 与えられたCSVファイルの動きを再現する。こっちの方が精度いい。
def replicate_2(motor_array, file_path):

    df = pd.read_csv(file_path)
    data_list = df.values.tolist()
    
    for motor in motor_array:
        motor.set_run_mode("motion_control")
        motor.enable_motor()

    start_time = time.time()

    # for motor_list in data_list:
    #     if time.time() - start_time > motor_list[0]:
    #         continue
    #     else:
    #         num = 1
    #         for motor in motor_array:
    #             motor.motion_control(motor_list[num], 0.01, 5.0, 0.5, 1.0)
    #             # print(motor.torque - 1.2 * math.sin(motor.angle))  125
    #             print(motor.torque - 1.2 *math.sin(motor.angle))
    #             num += 1

    gap = 0
    duration = 2
    restart = False

    for motor_list in data_list:
        torque = False
        if time.time() - gap - start_time > motor_list[0]:
            continue
        else:
            num = 1
            if restart:
                restart_start = time.time()
                for motor in motor_array:
                    # motor.set_run_mode("speed")
                    while abs(motor.angle - motor_list[num]) > 0.4:
                        # motor.position_control(motor_list[num], 1.3)
                        motor.motion_control(motor_list[num], 0.5, 0.6, 0.05, 1.0)
                        # if motor.angle - motor_list[num] < 0:
                        #     motor.speed_control(3.0, 1.0)
                        # else:
                        #     motor.speed_control(3.0, 1.0)
                        print(motor.motor, "waiting...")
                    # motor.set_run_mode("motion_control")
                    # motor.motion_control(motor_list[num], 0.01, 3.0, 0.1)
                    num += 1
                # for motor in motor_array:
                #     motor.set_run_mode("motion_control")
                restart_end = time.time()
                gap += restart_end - restart_start
                restart = False
                continue

            for motor in motor_array:
                motor.update_state()
                # print(motor.motor, motor.torque_gap)
                print(motor.real_torque)
                if abs(motor.real_torque) > 2.0:
                    # print(motor.motor, motor.torque_gap)
                    # motor.motion_control(motor_list[num], 0.01, 0.2, 0.01)
                    torque = True
                    break
                else:
                    motor.motion_control(motor_list[num], 0.01, 5.0, 0.5) # 4.5~6.0がちょうどいい？
                    print("--")
                num += 1

                # motor.motion_control(motor_list[num], 0.01, 5.0, 0.5)
                # num += 1
            
            if torque:
                start = time.time()
                while time.time() - start < duration:
                    for motor in motor_array:
                        # motor.set_run_mode("speed")
                        # # motor.position_control(0.0, 1.0)
                        # # motor.motion_control(0.0, 0.2, 0.3, 0.01)
                        # while abs(motor.angle) > 0.2:
                        #     if motor.angle > 0:
                        #         motor.speed_control(-0.3, 1.5)
                        #     else:
                        #         motor.speed_control(0.3, 1.5)
                        motor.set_run_mode("motion_control")
                        motor.motion_control(0.0, 0.1, 0.5, 0.1, 0.3)
                gap += duration
                restart = True
                print("reached 0")


def detect_canid(master=253):

<<<<<<< HEAD
# motor_1 = Cybergear(253, 125, 0.6, 0.45)
# motor_2 = Cybergear(253, 127, 2.0, 0.4)
# motor_3 = Cybergear(253, 126, 2.0, 0.45)
# motor_4 = Cybergear(253, 121, 1.3, 0.9)

# motor_1 = Cybergear(253, 125, torque_const=1.2)
# motor_2 = Cybergear(253, 127)
# motor_3 = Cybergear(253, 126)
# motor_4 = Cybergear(253, 121, torque_const=1.5)

# Motor = [motor_4]
# path = 'output_.csv'
# path_swing_1hz = "swing_1hz.csv"
# path_swing_1_5hz = "swing_1_5hz.csv"
# path_swing_1_8hz = "swing_1_8hz.csv"
# path_swing_2hz = "swing_2hz.csv"
# path_swing_1hz_2 = "swing_1hz_2.csv"

=======
    id_min = 110
    id_max = 127

    ser.write(bytes.fromhex('41 54 2b 41 54 0d 0a'))
    ser.flush()

    data = ser.read_until(expected=b'\r\n')
    print(data)
    data = int.from_bytes(data, "little")
    print(data)
    received_data = reverse_hex("0"+hex(data)[2:])
    print(">>" + received_data)

    for motor in range(id_max, id_min-1, -1):

        bin_num = master << 8 | motor #2進数のまま結合
        bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
        #print(hex(bin_num))
        hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
        hex_can = frame_head + "000" + hex_str + "0100" + frame_tail
        # print(hex_can)

        ser.write(bytes.fromhex(hex_can))
        ser.flush()

        data = ser.read_until(expected=b'\r\n')
        if not data:
            continue
        else:
            print("CAN ID", motor, "detected")
        # data = int.from_bytes(data, "little")
        # received_data = reverse_hex("0"+hex(data)[2:])
        # print(">>" + received_data)

# detect_canid()

motor = Cybergear(253, 120)
motor.power_on()
motor.set_run_mode("current")
motor.enable_motor()
start = time.time()
for i in range(1000):
    motor.current_control(0.3, 0.0)
stop = time.time()
print(stop-start)
motor.stop_motor()

# motor_1 = Cybergear(253, 122, 0.6, 0.45)
# motor_2 = Cybergear(253, 127, 2.0, 0.4)
# motor_3 = Cybergear(253, 126, 2.0, 0.45)
# motor_4 = Cybergear(253, 121, 1.3, 0.9)

# Motor = [motor_1, motor_2, motor_3, motor_4]
# path = 'output.csv'
# path_swing_1hz = "swing_1hz.csv"
# path_swing_1_5hz = "swing_1_5hz.csv"
# path_swing_1_8hz = "swing_1_8hz.csv"
# path_swing_2hz = "swing_2hz.csv"
# path_swing_1hz_2 = "swing_1hz_2.csv"

>>>>>>> daff9e060ba9a53c4ed6c6c001c28da69d5a6457
# motor_1.power_on()
# motor_2.power_on()
# motor_3.power_on()
# motor_4.power_on()
<<<<<<< HEAD

# motor_1.set_run_mode("current")
# motor_2.set_run_mode("current")
# motor_3.set_run_mode("current")
# motor_4.set_run_mode("current")

# motor_1.enable_motor()
# motor_2.enable_motor()
# motor_3.enable_motor()
# motor_4.enable_motor()

# motor_1.homing_mode()
# motor_2.homing_mode()
# motor_3.homing_mode()
# motor_4.homing_mode()

# motor_1.current_control(0.0, 0.0)
# motor_2.current_control(0.0, 0.0)
# motor_3.current_control(0.0, 0.0)
# motor_4.current_control(0.0, 0.0)
=======

# motor_1.set_run_mode("current")
# motor_2.set_run_mode("current")
# motor_3.set_run_mode("current")
# motor_4.set_run_mode("current")

# motor_1.enable_motor()
# motor_2.enable_motor()
# motor_3.enable_motor()
# motor_4.enable_motor()

# motor_1.homing_mode()
# motor_2.homing_mode()
# motor_3.homing_mode()
# motor_4.homing_mode()

# motor_1.current_control(0.0, 0.0)
# motor_2.current_control(0.0, 0.0)
# motor_3.current_control(0.0, 0.0)
# motor_4.current_control(0.0, 0.0)



>>>>>>> daff9e060ba9a53c4ed6c6c001c28da69d5a6457

# time.sleep(10)

# motor_2.homing_mode()
# motor_2.position_control(0.0, 1.5)

# time.sleep(10)

# motor_2.set_run_mode("current")
# motor_2.current_control(0.0, 0.0)


#motor_1.motion_control(3.0, 3.0, 3.0, 10.0, 1.0)


# motor_1.current_control(0.3)
# motor_2.current_control(0.3)


# time.sleep(1)

#motor_1.read_param(index_dict["mechVel"])

# motor_1.current_control(-0.3)
# motor_2.current_control(-0.3)

# time.sleep(5)


# upload_to_csv(Motor, 10, path)

# replicate_2(Motor, path)

# motor_1.stop_motor()
# motor_2.stop_motor()
# motor_3.stop_motor()
# motor_4.stop_motor()







# motor = Cybergear(253, 124)
# motor.power_on()
# motor.enable_motor()

# motor.reset_canid(124)

# motor.stop_motor()






# motor = Cybergear(253, 127)

# motor_array = [motor]
# path = 'output.csv'

# motor.power_on()
# motor.set_run_mode("location")
# motor.enable_motor()

# motor.homing_mode()
# motor.position_control(0.0, 1.0)

# time.sleep(1)

# motor.set_run_mode("current")
# motor.enable_motor()

# motor.current_control(0.0, 0.0)

# upload_to_csv(motor_array, 10, path)

# replicate(motor_array, path)

# motor.stop_motor()






# motor = Cybergear(253, 125)

# motor.power_on()
# motor.set_run_mode("motion_control")
# motor.enable_motor()

# motor.homing_mode()

# motor.motion_control(2.0, 0.01, 3.0, 0.5)

# time.sleep(10)

# motor.stop_motor()


# time.sleep(5)


# motor = Cybergear(253, 121)

# motor.power_on()
# motor.set_run_mode("motion_control")
# motor.enable_motor()

# motor.homing_mode()

# motor.motion_control(1.5, 0.0001, 0.5, 0.1, 0.0)

# time.sleep(10)

# motor.stop_motor()



# motor = Cybergear(253, 125, 2.0, 0.4)

# motor_array = [motor]

# motor.power_on()
# motor.set_run_mode("current")
# motor.enable_motor()

# motor.homing_mode()
# motor.current_control(0.0, 0.0)

# # motor.reset_canid(120)

# # motor.update_state()

# # upload_to_csv(motor_array, 10, path)
# replicate_2(motor_array, path)

# motor.stop_motor()