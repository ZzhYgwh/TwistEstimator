#!/usr/bin/env python3
import serial
import time

# 常见波特率列表
baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

port = "/dev/ttyUSB0"

for br in baudrates:
    try:
        print(f"\n=== Testing baudrate: {br} ===")
        ser = serial.Serial(port, br, timeout=1)
        time.sleep(0.2)
        ser.reset_input_buffer()

        # 如果需要，可以发送一条测试命令（部分设备需要唤醒）
        # ser.write(b'AT\r\n')

        start = time.time()
        while time.time() - start < 3:
            data = ser.read(100)
            if data:
                print(f"[{br}] RX:", data)
        ser.close()
    except Exception as e:
        print(f"[{br}] Error:", e)
