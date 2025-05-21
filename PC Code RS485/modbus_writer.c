#include <windows.h>
#include <stdint.h>
#include <stdio.h>

uint16_t modbus_crc(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int send_motor_velocities_raw(const char *comPort, float omega_left, float omega_right) {
    uint16_t left = (uint16_t)(omega_left * 100);
    uint16_t right = (uint16_t)(omega_right * 100);

    uint8_t frame[13];
    frame[0] = 0x01; // Slave ID
    frame[1] = 0x10; // Function code
    frame[2] = 0x00; frame[3] = 0x00; // Start address 0
    frame[4] = 0x00; frame[5] = 0x02; // 2 registers
    frame[6] = 0x04; // byte count
    frame[7] = left >> 8;
    frame[8] = left & 0xFF;
    frame[9] = right >> 8;
    frame[10] = right & 0xFF;

    uint16_t crc = modbus_crc(frame, 11);
    frame[11] = crc & 0xFF;
    frame[12] = crc >> 8;

    HANDLE hSerial = CreateFileA(comPort, GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Error opening port %s\n", comPort);
        return -1;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts(hSerial, &timeouts);

    DWORD bytesWritten;
    WriteFile(hSerial, frame, 13, &bytesWritten, NULL);
    CloseHandle(hSerial);

    printf("Sent %lu bytes: ω₁=%.2f rad/s, ω₂=%.2f rad/s\n", bytesWritten, omega_left, omega_right);
    return 0;
}

int main() {
    return send_motor_velocities_raw("COM5", 1.57, 2.09); // update COM5 if needed
}
