import serial

# 打开串口
ser = serial.Serial(
    port='/dev/ttyUSB0',   # 串口设备名
    baudrate=9600,       # 波特率 230400 
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1              # 超时 1 秒（可选）
)

print(f"已打开串口: {ser.name}")

try:
    while True:
        if ser.in_waiting > 0:  # 检查是否有数据等待读取
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            print("收到数据:", data)
except KeyboardInterrupt:
    print("\n用户中断")
finally:
    ser.close()
    print("串口已关闭")

