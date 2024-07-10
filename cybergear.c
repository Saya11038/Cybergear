#include <stdio.h>
#include <string.h>
#include <windows.h>

HANDLE init_serial(const char *portname, int baudrate) {
    HANDLE hSerial;
    hSerial = CreateFile(portname, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) {
        if (GetLastError() == ERROR_FILE_NOT_FOUND) {
            printf("Serial port does not exist.\n");
            return INVALID_HANDLE_VALUE;
        }
        printf("Error in opening serial port.\n");
        return INVALID_HANDLE_VALUE;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("Error getting state.\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        printf("Error setting state.\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        printf("Error setting timeouts.\n");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    return hSerial;
}

void hex_to_bytes(const char *hex_str, unsigned char *byte_array, size_t byte_array_size) {
    size_t len = strlen(hex_str);
    for (size_t i = 0; i < len; i += 2) {
        sscanf(hex_str + i, "%2hhx", &byte_array[i / 2]);
    }
}

int main() {
    const char *portname = "COM3";  // 使用するシリアルポート名
    int baudrate = 921600;      // ボーレート

    HANDLE hSerial = init_serial(portname, baudrate);
    if (hSerial == INVALID_HANDLE_VALUE) {
        return -1;
    }

    const char *forward = "41 54 90 07 eb fc 08 05 70 00 00 07 01 95 54 0d 0a";
    const char *stop = "41 54 90 07 eb fc 08 05 70 00 00 00 00 00 00 0d 0a";

    unsigned char forward_bytes[17];
    unsigned char stop_bytes[17];

    hex_to_bytes(forward, forward_bytes, sizeof(forward_bytes));
    hex_to_bytes(stop, stop_bytes, sizeof(stop_bytes));

    

    DWORD bytes_written;
    WriteFile(hSerial, forward_bytes, sizeof(forward_bytes), &bytes_written, NULL);
    Sleep(1000);
    WriteFile(hSerial, stop_bytes, sizeof(stop_bytes), &bytes_written, NULL);

    CloseHandle(hSerial);

    return 0;
}
