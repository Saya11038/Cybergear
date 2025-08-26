#define _USE_MATH_DEFINES // M_PI を有効にするために必要
#include <windows.h>
#include <stdio.h>
#include <stdlib.h> // for strtol
#include <stdint.h>
#include <math.h>
#include <time.h>

const unsigned char crlf[] = {0x0D, 0x0A};  //frame_tail
const unsigned char at[] = {0x41, 0x54};  //frame_head

HANDLE open_serial_port(const char* portName, DWORD baudRate) {
    HANDLE hSerial;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };

    // 1. シリアルポートを開く
    hSerial = CreateFile(portName,
                         GENERIC_READ | GENERIC_WRITE,
                         0,
                         NULL,
                         OPEN_EXISTING,
                         FILE_ATTRIBUTE_NORMAL,
                         NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        // fprintf(stderr, "Error opening serial port %s\n", portName); // エラーメッセージは呼び出し元で
        return INVALID_HANDLE_VALUE;
    }

    // 2. シリアルポートのパラメータを設定する
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        // fprintf(stderr, "Error getting current serial port settings\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }
    // 例: 921600bps、8データビット、パリティなし、1ストップビット
    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        // fprintf(stderr, "Error setting serial port settings\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    // 3. タイムアウトを設定する (ReadIntervalTimeoutを無限、ReadTotalTimeoutConstantとReadTotalTimeoutMultiplierを設定)
    // この例では、ReadTotalTimeoutConstant = 500ms (1000ms = 1秒) としています
    // read_until(expected=b'\r\n') に対応するため、ある程度長めに設定します
    timeouts.ReadIntervalTimeout = MAXDWORD;  // ReadFileの呼び出し間で、新しい文字が到着する最大時間
    timeouts.ReadTotalTimeoutConstant = 500;  // ReadFileの呼び出しごとの総読み取り時間 (ms)
    timeouts.ReadTotalTimeoutMultiplier = 0;  // ReadFileの呼び出しごとの文字ごとの総読み取り時間 (ms)
    timeouts.WriteTotalTimeoutConstant = 500;  // WriteFileの呼び出しごとの総書き込み時間 (ms)
    timeouts.WriteTotalTimeoutMultiplier = 0;  // WriteFileの呼び出しごとの文字ごとの総書き込み時間 (ms)

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        // fprintf(stderr, "Error setting serial port timeouts\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    return hSerial;
}

BOOL write_serial_data(HANDLE hSerial, const unsigned char* dataToWrite, DWORD dataLength) {
    DWORD bytesWritten;
    if (!WriteFile(hSerial, dataToWrite, dataLength, &bytesWritten, NULL)) {
        // fprintf(stderr, "Error writing to serial port\n");
        return FALSE;
    }
    // フラッシュする (Pythonの ser.flush() に相当)
    // WriteFileの後、明示的なFlushFileBuffersは通常必要ありませんが、
    // 書き込みバッファを強制的に空にしたい場合は呼び出します

    // if (!FlushFileBuffers(hSerial)) { 
    //     fprintf(stderr, "Error flushing write buffer\n");
    //     return FALSE;
    // }
    return TRUE;
}

BOOL read_serial_data_until(HANDLE hSerial, unsigned char* readBuffer, DWORD bufferSize, DWORD* bytesRead, const unsigned char* expectedSuffix, DWORD suffixLength) {
    DWORD currentBytesRead = 0;
    DWORD bytesJustRead;
    BOOL result = FALSE; // 読み込み成功フラグ

    *bytesRead = 0; // 読み込んだバイト数を初期化

    while (currentBytesRead < bufferSize - 1) { // バッファオーバーフロー防止
        if (!ReadFile(hSerial, &readBuffer[currentBytesRead], 1, &bytesJustRead, NULL)) {
            // fprintf(stderr, "Error reading from serial port\n");
            return FALSE;
        }

        if (bytesJustRead == 0) {
            // タイムアウトなどにより読み込みが停止した場合
            // fprintf(stderr, "Read timeout or no data received.\n");
            break; // ループを抜ける
        }

        currentBytesRead += bytesJustRead;

        // 終了シーケンスのチェック
        if (currentBytesRead >= suffixLength) {
            BOOL suffixMatch = TRUE;
            for (DWORD i = 0; i < suffixLength; ++i) {
                if (readBuffer[currentBytesRead - suffixLength + i] != expectedSuffix[i]) {
                    suffixMatch = FALSE;
                    break;
                }
            }
            if (suffixMatch) {
                result = TRUE; // 終了シーケンスが見つかった
                break; // ループを抜ける
            }
        }
    }
    *bytesRead = currentBytesRead; // 実際に読み込んだバイト数をセット
    return result;
}


float uint16_to_float(uint16_t data, float min_float, float max_float) {
    // 2バイト符号なし整数の最大値
    const float INTEGER_MAX = 65535.0f;

    // 整数値を0.0〜1.0の範囲に正規化
    float normalized_data = (float)data / INTEGER_MAX;

    // 正規化された値を目的のfloat範囲にスケーリング
    float float_data = normalized_data * (max_float - min_float) + min_float;

    return float_data;
}

float uint32_to_float(uint32_t data, float min_float, float max_float) {
    // uint32_t の最大値 (float型として扱う)
    const float INTEGER_MIN_F = 0.0f;
    const float INTEGER_MAX_F = (float)UINT32_MAX; // 4294967295.0f

    // 整数値を0.0〜1.0の範囲に正規化
    float normalized_value = ((float)data - INTEGER_MIN_F) / (INTEGER_MAX_F - INTEGER_MIN_F);

    // 正規化された値を目的のfloat範囲にスケーリング
    float float_value = normalized_value * (max_float - min_float) + min_float;

    return float_value;
}

typedef union {
    float float_value;
    uint32_t uint32_value;
} Float_Uint32;


// Cybergearモーターのデータ構造
typedef struct {
    HANDLE hSerial;       // シリアルポートのハンドル (CAN通信モジュール用)
    uint8_t master_id;   // ホストのCAN ID
    uint8_t motor_id;    // モーターのCAN ID
    // 制御に必要なその他の状態や設定値
    float current_angle; // 現在の位置 (角度など)
    float current_velocity; // 現在の速度
    float current_torque;   // 現在のトルク
    float current_temp;  //現在の温度
    // その他の設定やフラグ
    BOOL is_powered_on;     // 電源ON/OFF状態
} Cybergear;

BOOL init_and_poweron(Cybergear* motor, HANDLE hSerial, uint8_t can_id) {
    // 構造体の初期化・モーターの電源オン・原点合わせを行う(電源を入れた時の角度を0にする)
    // NULLポインタチェック
    if (motor == NULL) {
        fprintf(stderr, "Error: NULL pointer passed to initializeMyData.\n");
        return FALSE;
    }

    motor->hSerial = hSerial;
    motor->master_id = 253;
    motor->motor_id = can_id;
    motor->current_angle = 0.0;
    motor->current_velocity = 0.0;
    motor->current_torque = 0.0;
    motor->current_temp = 0.0;
    motor->is_powered_on = FALSE;

}

BOOL power_on(Cybergear* motor) {
    if (motor == NULL) {
        fprintf(stderr, "Error: NULL pointer passed to initializeMyData.\n");
        return FALSE;
    }
    unsigned char dataToSend[] = {0x41, 0x54, 0x2B, 0x41, 0x54, 0x0D, 0x0A};
    DWORD dataToSendLength = sizeof(dataToSend);

    if (!write_serial_data(motor->hSerial, dataToSend, dataToSendLength)) {
        fprintf(stderr, "Error writing to serial port\n");
        CloseHandle(motor->hSerial);
        return FALSE;
    }
    printf("Command sent.\n");
    printf("Reading data...\n");
    DWORD bytesRead;
    unsigned char readBuffer[256]; // 受信バッファ

    if (read_serial_data_until(motor->hSerial, readBuffer, sizeof(readBuffer), &bytesRead, crlf, sizeof(crlf))) {
        printf("Received %lu bytes.\n", bytesRead);

        // 受信データの表示と処理 (Pythonのprint(data)に相当)
        printf("Received data (raw): ");
        for (DWORD i = 0; i < bytesRead; ++i) {
            printf("%02X ", readBuffer[i]);
        }
        printf("\n");

        // Pythonの print(data) に相当
        // 受信したバイト列をそのまま表示
        printf("Received data (bytes): ");
        for (DWORD i = 0; i < bytesRead; ++i) {
            printf("%c", readBuffer[i]);
        }
        printf("\n");

    } else {
        fprintf(stderr, "Failed to read data from serial port or timeout.\n");
        return FALSE;
    }
    motor->is_powered_on = TRUE;
    return TRUE;
}

BOOL get_motor_state(Cybergear* motor, const unsigned char* data, DWORD data_length) {
    if (data_length != 17) {
        return FALSE;
    }
    uint32_t data_num = 0;
    for (int i=0; i<4; i++) {
        data_num <<= 8;
        data_num |= data[i+2];
    }
    // printf("%08X\n", data_num);
    uint32_t status = data_num >> 3;

    uint8_t frame_type = status >> 24;
    if (frame_type != 2) {
        return FALSE;
    }

    uint8_t master_canid = status & 0xFF;
    uint8_t motor_canid = status >> 8 & 0xFF;
    uint8_t under_voltage = status >> 16 & 1;
    uint8_t over_current = status >> 17 & 1;
    uint8_t over_temp = status >> 18 & 1;
    uint8_t magnetic_encoding_failure = status >> 19 & 1;
    uint8_t HALL_encoding_failure = status >> 20 & 1;
    uint8_t state = status >> 22 & 0b11;

    // printf("Master ID: %d\n", master_canid);
    // printf("Motor ID: %d\n", motor_canid);

    if (under_voltage == 1) {
        printf("Under Voltage Fault\n");
    }
    if (over_current == 1) {
        printf("Over Current\n");
    }
    if (over_temp == 1) {
        printf("Over Temperature\n");
    }
    if (magnetic_encoding_failure == 1) {
        printf("Magnetic Encoding Failure\n");
    }
    if (HALL_encoding_failure == 1) {
        printf("HALL Encoding Failure\n");
    }
    // if (state == 0) {
    //     printf("Reset Mode.\n");
    // } else if (state == 1) {
    //     printf("Cali Mode.\n");
    // } else if (state == 2) {
    //     printf("Motor Mode.\n");
    // }

    uint16_t int_angle = data[7] << 8 | data[8];
    uint16_t int_angle_vel = data[9] << 8 | data[10];
    uint16_t int_torque = data[11] << 8 | data[12];
    uint16_t int_temp = data[13] << 8 | data[14];
 
    float angle = uint16_to_float(int_angle, -4.0*M_PI, 4.0*M_PI);
    float angle_vel = uint16_to_float(int_angle_vel, -30.0, 30.0);
    float torque = uint16_to_float(int_torque, -12.0, 12.0);
    float temp = int_temp / 10.0;

    // printf("Angle: %f [rad]\n", angle);
    // printf("Angle Velocity: %f [rad/s]\n", angle_vel);
    // printf("Torque: %f [Nm]\n", torque);
    // printf("Temperature: %f [C]\n", temp);

    motor->current_angle = angle;
    motor->current_velocity = angle_vel;
    motor->current_torque = torque;
    motor->current_temp = temp;

    return TRUE;
}

BOOL stop_motor(Cybergear* motor) {
    uint8_t frame_stop = 4;
    unsigned char dataToSend[17];

    uint64_t frame = frame_stop << 24 | motor->master_id << 8 | motor->motor_id;
    frame = frame << 3 | 0b100;

    dataToSend[0] = at[0];
    dataToSend[1] = at[1];

    for (int i=0; i<4; i++) {
        dataToSend[i+2] = (unsigned char)((frame >> 8*(3-i)) & 0xFF);
    }

    dataToSend[6] = 0x08;

    for (int i=0; i<8; i++) {
        dataToSend[7+i] = 0x00;
    }

    dataToSend[15] = crlf[0];
    dataToSend[16] = crlf[1];

    DWORD dataToSendLength = sizeof(dataToSend);

    // 確認用
    printf("Send data: ");
    for (int i=0; i<17; i++) {
        printf("%02X ", dataToSend[i]);
    }
    printf("\n");

    if (!write_serial_data(motor->hSerial, dataToSend, dataToSendLength)) {
        fprintf(stderr, "Error writing to serial port\n");
        CloseHandle(motor->hSerial);
        return FALSE;
    }

    DWORD bytesRead;
    unsigned char readBuffer[256]; // 受信バッファ

    if (read_serial_data_until(motor->hSerial, readBuffer, sizeof(readBuffer), &bytesRead, crlf, sizeof(crlf))) {
        // 受信データの表示と処理 (Pythonのprint(data)に相当)
        printf("Received data (raw): ");
        for (DWORD i = 0; i < bytesRead; ++i) {
            printf("%02X ", readBuffer[i]);
        }
        printf("\n");
        get_motor_state(motor, readBuffer, bytesRead);

    } else {
        fprintf(stderr, "Failed to read data from serial port or timeout.\n");
        return FALSE;
    }
    motor->is_powered_on = FALSE;
    return TRUE;
}

BOOL set_run_mode(Cybergear* motor, uint8_t mode) {
    // mode = 1 → 位置制御
    // mode = 2 → 速度制御
    // mode = 3 → 電流制御

    uint8_t frame_write = 18;
    unsigned char dataToSend[17];

    uint64_t frame = frame_write << 24 | motor->master_id << 8 | motor->motor_id;
    frame = frame << 3 | 0b100;

    dataToSend[0] = at[0];
    dataToSend[1] = at[1];

    for (int i=0; i<4; i++) {
        dataToSend[i+2] = (unsigned char)((frame >> 8*(3-i)) & 0xFF);
    }

    dataToSend[6] = 0x08;
    dataToSend[7] = 0x05;
    dataToSend[8] = 0x70;
    dataToSend[9] = 0x00;
    dataToSend[10] = 0x00;
    dataToSend[11] = mode;
    dataToSend[12] = 0x00;
    dataToSend[13] = 0x00;
    dataToSend[14] = 0x00;
    dataToSend[15] = crlf[0];
    dataToSend[16] = crlf[1];

    DWORD dataToSendLength = sizeof(dataToSend);

    // 確認用
    printf("Send data: ");
    for (int i=0; i<17; i++) {
        printf("%02X ", dataToSend[i]);
    }
    printf("\n");

    if (!write_serial_data(motor->hSerial, dataToSend, dataToSendLength)) {
        fprintf(stderr, "Error writing to serial port\n");
        CloseHandle(motor->hSerial);
        return FALSE;
    }

    DWORD bytesRead;
    unsigned char readBuffer[256]; // 受信バッファ

    if (read_serial_data_until(motor->hSerial, readBuffer, sizeof(readBuffer), &bytesRead, crlf, sizeof(crlf))) {
        // 受信データの表示と処理 (Pythonのprint(data)に相当)
        printf("Received data (raw): ");
        for (DWORD i = 0; i < bytesRead; ++i) {
            printf("%02X ", readBuffer[i]);
        }
        printf("\n");
        get_motor_state(motor, readBuffer, bytesRead);

    } else {
        fprintf(stderr, "Failed to read data from serial port or timeout.\n");
        return FALSE;
    }
    return TRUE;
}

BOOL enable_motor(Cybergear* motor) {
    uint8_t frame_enable = 3;
    unsigned char dataToSend[17];

    uint64_t frame = frame_enable << 24 | motor->master_id << 8 | motor->motor_id;
    frame = frame << 3 | 0b100;

    dataToSend[0] = at[0];
    dataToSend[1] = at[1];

    for (int i=0; i<4; i++) {
        dataToSend[i+2] = (unsigned char)((frame >> 8*(3-i)) & 0xFF);
    }

    dataToSend[6] = 0x08;

    for (int i=0; i<8; i++) {
        dataToSend[7+i] = 0x00;
    }

    dataToSend[15] = crlf[0];
    dataToSend[16] = crlf[1];

    DWORD dataToSendLength = sizeof(dataToSend);

    // 確認用
    printf("Send data: ");
    for (int i=0; i<17; i++) {
        printf("%02X ", dataToSend[i]);
    }
    printf("\n");

    if (!write_serial_data(motor->hSerial, dataToSend, dataToSendLength)) {
        fprintf(stderr, "Error writing to serial port\n");
        CloseHandle(motor->hSerial);
        return FALSE;
    }

    DWORD bytesRead;
    unsigned char readBuffer[256]; // 受信バッファ

    if (read_serial_data_until(motor->hSerial, readBuffer, sizeof(readBuffer), &bytesRead, crlf, sizeof(crlf))) {
        // 受信データの表示と処理 (Pythonのprint(data)に相当)
        printf("Received data (raw): ");
        for (DWORD i = 0; i < bytesRead; ++i) {
            printf("%02X ", readBuffer[i]);
        }
        printf("\n");
        get_motor_state(motor, readBuffer, bytesRead);

    } else {
        fprintf(stderr, "Failed to read data from serial port or timeout.\n");
        return FALSE;
    }
    return TRUE;
}

BOOL current_control(Cybergear* motor, float iq, float id) {
    uint8_t frame_write = 18;
    unsigned char dataToSend[17];

    uint64_t frame = frame_write << 24 | motor->master_id << 8 | motor->motor_id;
    frame = frame << 3 | 0b100;

    dataToSend[0] = at[0];
    dataToSend[1] = at[1];

    for (int i=0; i<4; i++) {
        dataToSend[i+2] = (unsigned char)((frame >> 8*(3-i)) & 0xFF);
    }

    dataToSend[6] = 0x08;
    dataToSend[7] = 0x06;
    dataToSend[8] = 0x70;
    dataToSend[9] = 0x00;
    dataToSend[10] = 0x00;
    
    // 浮動小数点型へ変換
    Float_Uint32 iq_converter;
    iq_converter.float_value = iq;
    uint32_t iq_ref = iq_converter.uint32_value;
    // printf("%08X\n", iq_ref); 

    for (int i=0; i<4; i++) {
        dataToSend[11+i] = (iq_ref >> 8*i) & 0xFF;
    }
    dataToSend[15] = crlf[0];
    dataToSend[16] = crlf[1];

    DWORD dataToSendLength = sizeof(dataToSend);

    // // 確認用
    // printf("Send data: ");
    // for (int i=0; i<17; i++) {
    //     printf("%02X ", dataToSend[i]);
    // }
    // printf("\n");

    if (!write_serial_data(motor->hSerial, dataToSend, dataToSendLength)) {
        fprintf(stderr, "Error writing to serial port\n");
        CloseHandle(motor->hSerial);
        return FALSE;
    }

    DWORD bytesRead;
    unsigned char readBuffer[256]; // 受信バッファ

    if (read_serial_data_until(motor->hSerial, readBuffer, sizeof(readBuffer), &bytesRead, crlf, sizeof(crlf))) {
        // 受信データの表示と処理 (Pythonのprint(data)に相当)
        // printf("Received data (raw): ");
        // for (DWORD i = 0; i < bytesRead; ++i) {
        //     printf("%02X ", readBuffer[i]);
        // }
        // printf("\n");
        get_motor_state(motor, readBuffer, bytesRead);

    } else {
        fprintf(stderr, "Failed to read data from serial port or timeout.\n");
        return FALSE;
    }

    // unsigned char dataToSend_2[17];

    // uint64_t frame_2 = frame_write << 24 | motor->master_id << 8 | motor->motor_id;
    // frame_2 = frame_2 << 3 | 0b100;

    // dataToSend_2[0] = at[0];
    // dataToSend_2[1] = at[1];

    // for (int i=0; i<4; i++) {
    //     dataToSend_2[i+2] = (unsigned char)((frame_2 >> 8*(3-i)) & 0xFF);
    // }

    // dataToSend_2[6] = 0x08;
    // dataToSend_2[7] = 0x07;
    // dataToSend_2[8] = 0x70;
    // dataToSend_2[9] = 0x00;
    // dataToSend_2[10] = 0x00;
    
    // // 浮動小数点型へ変換
    // Float_Uint32 id_converter;
    // id_converter.float_value = id;
    // uint32_t id_ref = id_converter.uint32_value;
    // // printf("%08X\n", iq_ref); 

    // for (int i=0; i<4; i++) {
    //     dataToSend_2[11+i] = (id_ref >> 8*i) & 0xFF;
    // }
    // dataToSend_2[15] = crlf[0];
    // dataToSend_2[16] = crlf[1];

    // DWORD dataToSendLength_2 = sizeof(dataToSend_2);

    // // 確認用
    // printf("Send data: ");
    // for (int i=0; i<17; i++) {
    //     printf("%02X ", dataToSend_2[i]);
    // }
    // printf("\n");

    // if (!write_serial_data(motor->hSerial, dataToSend_2, dataToSendLength_2)) {
    //     fprintf(stderr, "Error writing to serial port\n");
    //     CloseHandle(motor->hSerial);
    //     return FALSE;
    // }

    // DWORD bytesRead_2;
    // unsigned char readBuffer_2[256]; // 受信バッファ

    // if (read_serial_data_until(motor->hSerial, readBuffer_2, sizeof(readBuffer_2), &bytesRead_2, crlf, sizeof(crlf))) {
    //     // 受信データの表示と処理 (Pythonのprint(data)に相当)
    //     printf("Received data (raw): ");
    //     for (DWORD i = 0; i < bytesRead_2; ++i) {
    //         printf("%02X ", readBuffer_2[i]);
    //     }
    //     printf("\n");
    //     get_motor_state(motor, readBuffer_2, bytesRead_2);

    // } else {
    //     fprintf(stderr, "Failed to read data from serial port or timeout.\n");
    //     return FALSE;
    // }
    return TRUE;
}

BOOL read_current(Cybergear* motor) {
    uint8_t frame_enable = 17;
    unsigned char dataToSend[17];

    uint64_t frame = frame_enable << 24 | motor->master_id << 8 | motor->motor_id;
    frame = frame << 3 | 0b100;

    dataToSend[0] = at[0];
    dataToSend[1] = at[1];

    for (int i=0; i<4; i++) {
        dataToSend[i+2] = (unsigned char)((frame >> 8*(3-i)) & 0xFF);
    }

    dataToSend[6] = 0x08;
    dataToSend[7] = 0x1A;
    dataToSend[8] = 0x70;

    for (int i=0; i<6; i++) {
        dataToSend[9+i] = 0x00;
    }

    dataToSend[15] = crlf[0];
    dataToSend[16] = crlf[1];

    DWORD dataToSendLength = sizeof(dataToSend);

    // // 確認用
    // printf("Send data: ");
    // for (int i=0; i<17; i++) {
    //     printf("%02X ", dataToSend[i]);
    // }
    // printf("\n");

    if (!write_serial_data(motor->hSerial, dataToSend, dataToSendLength)) {
        fprintf(stderr, "Error writing to serial port\n");
        CloseHandle(motor->hSerial);
        return FALSE;
    }

    DWORD bytesRead;
    unsigned char readBuffer[256]; // 受信バッファ
    uint32_t iqf = 0;

    if (read_serial_data_until(motor->hSerial, readBuffer, sizeof(readBuffer), &bytesRead, crlf, sizeof(crlf))) {
        // 受信データの表示と処理 (Pythonのprint(data)に相当)
        // printf("Received data (raw): ");
        // for (DWORD i = 0; i < bytesRead; ++i) {
        //     printf("%02X ", readBuffer[i]);
        // }
        // printf("\n");
    } else {
        fprintf(stderr, "Failed to read data from serial port or timeout.\n");
        return FALSE;
    }

    for (int j=0; j<4; j++) {
        iqf |= readBuffer[11+j]<<(8*(3-j));
    }
    // printf("%X\n", iqf);

    float iqf_val = uint32_to_float(iqf, -23.0, 23.0);
    // printf("%f\n", iqf_val);

    return TRUE;
}


int main() {

    HANDLE hSerial;
    const char* portName = "COM3"; // 使用するCOMポート名に置き換える
    const DWORD baudRate = 921600;
    DWORD bytesRead;
    unsigned char readBuffer[256]; // 受信バッファ
    // const unsigned char crlf[] = {0x0D, 0x0A};

    // 1. シリアルポートを開く
    hSerial = open_serial_port(portName, baudRate);

    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error opening serial port %s\n", portName);
        return 1;
    }
    printf("Serial port %s opened successfully.\n", portName);

    Cybergear motor;
    init_and_poweron(&motor, hSerial, 120);
    power_on(&motor);
    set_run_mode(&motor, 3);
    enable_motor(&motor);
    time_t start_tick, end_tick;
    start_tick = time(NULL); // 現在の時刻をミリ秒単位で取得

    for (int i=0; i<2000; i++) {
        current_control(&motor, 0.7, 0.0);
        read_current(&motor);
    }
    end_tick = time(NULL); // 終了時刻を取得

    // 差を計算（ミリ秒） 41549007ebc408067000009a99993e0d0a
    double elapsed_ms = difftime(end_tick , start_tick);

    printf("Finished\n");
    printf("Elapsed: %.2f ms\n", elapsed_ms);
    stop_motor(&motor);

    // // 2. データを書き込む (Pythonの bytes.fromhex('41 54 2b 41 54 0d 0a') に相当)
    // // C言語では直接バイト配列を定義します
    // unsigned char dataToSend[] = {0x41, 0x54, 0x2B, 0x41, 0x54, 0x0D, 0x0A};
    // // unsigned char dataToSend[] = {0x41, 0x54, 0x00, 0x07, 0xeb, 0xc4, 0x01, 0x00, 0x0D, 0x0A};
    // DWORD dataToSendLength = sizeof(dataToSend);

    // if (!write_serial_data(hSerial, dataToSend, dataToSendLength)) {
    //     fprintf(stderr, "Error writing to serial port\n");
    //     CloseHandle(hSerial);
    //     return 1;
    // }
    // printf("Command sent.\n");

    // // 3. データを読み込む (Pythonの ser.read_until(expected=b'\r\n') に相当)
    // // `ReadFile` をループで呼び出し、`\r\n` が見つかるまで読み込みます。
    // // または、`SetCommTimeouts` で適切なタイムアウトを設定し、一度に読み込むことも検討します。
    // // ここでは、1文字ずつ読み込み、\r\nをチェックする簡易的な例を示します。
    // // より効率的な方法は、SetCommTimeoutsでReadTotalTimeoutConstantを十分に大きく設定し、
    // // ReadFileで一度に期待される最大バイト数を読み込むことです。

    // printf("Reading data...\n");

    // if (read_serial_data_until(hSerial, readBuffer, sizeof(readBuffer), &bytesRead, crlf, sizeof(crlf))) {
    //     printf("Received %lu bytes.\n", bytesRead);

    //     // 受信データの表示と処理 (Pythonのprint(data)に相当)
    //     printf("Received data (raw): ");
    //     for (DWORD i = 0; i < bytesRead; ++i) {
    //         printf("%02X ", readBuffer[i]);
    //     }
    //     printf("\n");

    //     // Pythonの print(data) に相当
    //     // 受信したバイト列をそのまま表示
    //     printf("Received data (bytes): ");
    //     for (DWORD i = 0; i < bytesRead; ++i) {
    //         printf("%c", readBuffer[i]);
    //     }
    //     printf("\n");

    // } else {
    //     fprintf(stderr, "Failed to read data from serial port or timeout.\n");
    // }

    // 4. シリアルポートを閉じる
    CloseHandle(hSerial);

    return 0;
}

    //  Pythonからの変換まわりメモ

    //     // Pythonの data = int.from_bytes(data, "little") に相当
    //     // 受信したバイト列をリトルエンディアンとして整数に変換
    //     // ただし、Pythonの `int.from_bytes` は可変長のバイト列を扱いますが、
    //     // C言語で同等の処理を行うには、バイト列のサイズとエンディアンを考慮した
    //     // 型変換が必要です。ここでは簡易的に最初の数バイトを変換する例を示します。
    //     // readBufferのデータが常に特定の長さの数値であると仮定した場合。
    //     if (totalBytesRead >= 4) { // 例: 4バイトの整数を想定
    //         long long intValue = 0;
    //         // リトルエンディアンで4バイトの整数を変換する例
    //         intValue = readBuffer[0] |
    //                    (readBuffer[1] << 8) |
    //                    (readBuffer[2] << 16) |
    //                    (readBuffer[3] << 24);
    //         printf("Received data (as int, little-endian first 4 bytes): %lld\n", intValue);
    //     } else {
    //         printf("Not enough bytes to convert to int (little-endian).\n");
    //     }


    //     // Pythonの received_data = reverse_hex("0"+hex(data)[2:]) に相当
    //     // `reverse_hex` のC言語実装は別途必要
    //     // ここでは、受信したバイト列を直接16進数で逆順に表示する例を示します。
    //     // Pythonの `hex(data)[2:]` の部分 (0x を除く) と、その後の `reverse_hex` の
    //     // 挙動を正確に再現するには、文字列操作が必要です。
    //     reverse_hex_c(readBuffer, totalBytesRead);


// reverse_hex関数のC言語版は別途実装が必要です
// ここでは簡易的に元のバイト列をそのまま表示する例を記載しています
// 必要に応じて、16進数文字列を反転させるロジックを実装してください
// void reverse_hex_c(const unsigned char* data, DWORD length) {
//     printf(">>");
//     for (DWORD i = 0; i < length; ++i) {
//         printf("%02X", data[i]);
//     }
//     printf("\n");
// }