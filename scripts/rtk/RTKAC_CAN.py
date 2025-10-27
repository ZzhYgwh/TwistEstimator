import serial
import datetime
import struct
from collections import deque
import time

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
#2400 4800 9600 19200 38400 57600 115200 230400 460800 921600 2000000

# CAN ID
TARGET_CAN_ID_051 = 0x051  # GPS时间
TARGET_CAN_ID_052 = 0x052  # 加速度
TARGET_CAN_ID_053 = 0x053  # 角速度
TARGET_CAN_ID_054 = 0x054  # 角度
TARGET_CAN_ID_055 = 0x055  # GPS经度
TARGET_CAN_ID_056 = 0x056  # GPS纬度
TARGET_CAN_ID_057 = 0x057  # GPS高度
TARGET_CAN_ID_058 = 0x058  # GPS航向角
TARGET_CAN_ID_059 = 0x059  # GPS地速
TARGET_CAN_ID_05A = 0x05A  # GPS卫星数
TARGET_CAN_ID_05B = 0x05B  # 惯导经度
TARGET_CAN_ID_05C = 0x05C  # 惯导纬度
TARGET_CAN_ID_05D = 0x05D  # 惯导高度
TARGET_CAN_ID_05E = 0x05E  # 惯导航向角
TARGET_CAN_ID_05F = 0x05F  # 惯导地速
TARGET_CAN_ID_CSYS_XY = 0x060  # 差分位移XY
TARGET_CAN_ID_CSYS_ZR = 0x061  # 差分位移ZR
TARGET_CAN_ID_CSYS_YAW = 0x062  # 差分航向角
TARGET_CAN_ID_STATUS = 0x063  # GPS状态

# 状态表 # 修改
STATUS_MODE = {
    0: "未定位",
    1: "单点定位",
    2: "码差分",
    4: "固定解",
    5: "浮点解"
}

G = 9.8
LSB_TO_DEG_PER_SEC = 2000 / 32768
LSB_TO_DEG_ANGLE = 180 / 32768

current_data = {
    'GPS时间': None,
    '加速度X': None, '加速度Y': None, '加速度Z': None,
    '角速度X': None, '角速度Y': None, '角速度Z': None,
    '角度X': None, '角度Y': None, '角度Z': None,
    'Lon': None, 'Lat': None, 'Alt': None,
    'GPSYaw': None, 'GPSFV': None, 'SVNUM': None,
    'IMU_Lon': None, 'IMU_Lat': None, 'IMU_Yaw': None, 'IMU_FV': None, 'IMU_FH': None,
    'CSYSX': None, 'CSYSY': None, 'CSYSZ': None, 'CSYSR': None,
    'CSYSYaw': None,
    'STATUS': None, 'STATUS定位': None, 'STATUS定向': None  # 修改
}

def send_at_command(ser):
    ser.write(b"AT+AT\r\n")
    print("已发送上电初始化命令: AT+AT")
    time.sleep(0.2)
    start_time = time.time()
    while time.time() - start_time < 2:
        if ser.in_waiting:
            resp = ser.readline().decode(errors='ignore').strip()
            if resp:
                print("设备回应:", resp)
            if "OK" in resp.upper():
                print("OK")
                print("设备初始化完成")
                break

def parse_0x051(data_bytes):
    if len(data_bytes) < 8:
        return
    yy, mm, dd, hh, mi, ss = data_bytes[0:6]
    ms_raw = (data_bytes[6] << 8) | data_bytes[7]
    ms = int(ms_raw * 1000 / 65536)
    timestamp = datetime.datetime(2000 + yy, mm, dd, hh, mi, ss, ms * 1000)
    current_data['GPS时间'] = f"{timestamp:%Y-%m-%d %H:%M:%S.%f}"[:-3]

def parse_0x052(data_bytes):
    if len(data_bytes) < 6:
        return
    ax_raw, ay_raw, az_raw = struct.unpack_from("<hhh", data_bytes, 0)
    current_data['加速度X'] = ax_raw / 32768 * 16
    current_data['加速度Y'] = ay_raw / 32768 * 16
    current_data['加速度Z'] = az_raw / 32768 * 16

def parse_0x053(data_bytes):
    if len(data_bytes) < 6:
        return
    gx_raw, gy_raw, gz_raw = struct.unpack_from("<hhh", data_bytes, 0)
    current_data['角速度X'] = gx_raw * LSB_TO_DEG_PER_SEC
    current_data['角速度Y'] = gy_raw * LSB_TO_DEG_PER_SEC
    current_data['角速度Z'] = gz_raw * LSB_TO_DEG_PER_SEC

def parse_0x054(data_bytes):
    if len(data_bytes) < 6:
        return
    roll_raw, pitch_raw, yaw_raw = struct.unpack_from("<hhh", data_bytes, 0)
    current_data['角度X'] = roll_raw * LSB_TO_DEG_ANGLE
    current_data['角度Y'] = pitch_raw * LSB_TO_DEG_ANGLE
    current_data['角度Z'] = yaw_raw * LSB_TO_DEG_ANGLE

def parse_0x055(data_bytes):
    if len(data_bytes) < 4:
        return
    lon_raw = struct.unpack_from("<i", data_bytes, 0)[0]
    current_data['Lon'] = lon_raw / 1e7

def parse_0x056(data_bytes):
    if len(data_bytes) < 4:
        return
    lat_raw = struct.unpack_from("<i", data_bytes, 0)[0]
    current_data['Lat'] = lat_raw / 1e7

def parse_0x057(data_bytes):
    if len(data_bytes) < 4:
        return
    alt_raw = struct.unpack_from("<f", data_bytes, 0)[0]
    current_data['Alt'] = alt_raw

def parse_0x058(data_bytes):
    if len(data_bytes) < 2:
        return
    yaw_raw = struct.unpack_from("<H", data_bytes, 0)[0]
    current_data['GPSYaw'] = yaw_raw * 0.1

def parse_0x059(data_bytes):
    if len(data_bytes) < 4:
        return
    gpsfv_raw = struct.unpack_from("<f", data_bytes, 0)[0]
    current_data['GPSFV'] = gpsfv_raw

def parse_0x05A(data_bytes):
    if len(data_bytes) < 2:
        return
    svnum = struct.unpack_from("<H", data_bytes, 0)[0]
    current_data['SVNUM'] = svnum

def parse_0x05B(data_bytes):
    if len(data_bytes) < 4:
        return
    lon_raw = struct.unpack_from("<i", data_bytes, 0)[0]
    current_data['IMU_Lon'] = lon_raw / 1e7

def parse_0x05C(data_bytes):
    if len(data_bytes) < 4:
        return
    lat_raw = struct.unpack_from("<i", data_bytes, 0)[0]
    current_data['IMU_Lat'] = lat_raw / 1e7

def parse_0x05D(data_bytes):
    if len(data_bytes) < 4:
        return
    fh_raw = struct.unpack_from("<f", data_bytes, 0)[0]
    current_data['IMU_FH'] = fh_raw

def parse_0x05E(data_bytes):
    if len(data_bytes) < 2:
        return
    yaw_raw = struct.unpack_from("<H", data_bytes, 0)[0]
    current_data['IMU_Yaw'] = yaw_raw * 0.1

def parse_0x05F(data_bytes):
    if len(data_bytes) < 4:
        return
    fv_raw = struct.unpack_from("<f", data_bytes, 0)[0]
    current_data['IMU_FV'] = fv_raw

# 修改：LSB=1/1000 m
def parse_0x060(data_bytes):
    if len(data_bytes) < 8:
        return
    csysx_raw, csysy_raw = struct.unpack_from("<ii", data_bytes, 0)
    current_data['CSYSX'] = csysx_raw / 1000.0
    current_data['CSYSY'] = csysy_raw / 1000.0

def parse_0x061(data_bytes):
    if len(data_bytes) < 8:
        return
    csysz_raw, csysr_raw = struct.unpack_from("<ii", data_bytes, 0)
    current_data['CSYSZ'] = csysz_raw / 1000.0
    current_data['CSYSR'] = csysr_raw / 1000.0

def parse_0x062(data_bytes):
    if len(data_bytes) < 2:
        return
    raw_yaw = struct.unpack_from("<h", data_bytes, 0)[0]
    current_data['CSYSYaw'] = raw_yaw * 0.1

# 修改：解析高8位定位，低8位定向
def parse_0x063(data_bytes):
    if len(data_bytes) < 2:
        return
    status_val = struct.unpack_from("<H", data_bytes, 0)[0]
    pos_flag = (status_val >> 8) & 0xFF
    ori_flag = status_val & 0xFF
    current_data['STATUS'] = status_val
    current_data['STATUS定位'] = STATUS_MODE.get(pos_flag, f"未知({pos_flag})")
    current_data['STATUS定向'] = STATUS_MODE.get(ori_flag, f"未知({ori_flag})")

def print_current_data():
    def fmt_acc(val): return f"{val:.2f}g" if val is not None else "---g"
    def fmt_gyro(val): return f"{val:.2f}°/s" if val is not None else "---°/s"
    def fmt_angle(val): return f"{val:.2f}°" if val is not None else "---°"
    def fmt_latlon(val): return f"{val:.7f}°" if val is not None else "---°"
    def fmt_alt(val): return f"{val:.2f}m" if val is not None else "---m"
    def fmt_gpsyaw(val): return f"{val:.2f}°" if val is not None else "---°"
    def fmt_gpsfv(val): return f"{val:.3f}m/s" if val is not None else "---m/s"
    def fmt_svnum(val): return f"{val}" if val is not None else "---"
    def fmt_fh(val): return f"{val:.2f}m" if val is not None else "---m"
    def fmt_csys(val): return f"{val:.3f}m" if val is not None else "---m"
    def fmt_csysyaw(val): return f"{val:.1f}°" if val is not None else "---°"

    print("---------------------------wtzn yyx 2025----------------------------------" )
    print(f"时间: {current_data['GPS时间'] or '---'}")
    print(f"加速度X={fmt_acc(current_data['加速度X'])}, 加速度Y={fmt_acc(current_data['加速度Y'])}, 加速度Z={fmt_acc(current_data['加速度Z'])}")
    print(f"角速度X={fmt_gyro(current_data['角速度X'])}, 角速度Y={fmt_gyro(current_data['角速度Y'])}, 角速度Z={fmt_gyro(current_data['角速度Z'])}")
    print(f"角度X={fmt_angle(current_data['角度X'])}, 角度Y={fmt_angle(current_data['角度Y'])}, 角度Z={fmt_angle(current_data['角度Z'])}")
    print(f"GPS经度={fmt_latlon(current_data['Lon'])}, GPS纬度={fmt_latlon(current_data['Lat'])}, GPS高度={fmt_alt(current_data['Alt'])}, GPS航向角={fmt_gpsyaw(current_data['GPSYaw'])}")
    print(f"GPS地速={fmt_gpsfv(current_data['GPSFV'])}, 卫星数={fmt_svnum(current_data['SVNUM'])}")
    print(f"惯导经度={fmt_latlon(current_data['IMU_Lon'])}, 惯导纬度={fmt_latlon(current_data['IMU_Lat'])}, 惯导高度={fmt_fh(current_data['IMU_FH'])}, 惯导航向角={fmt_gpsyaw(current_data['IMU_Yaw'])}, 惯导地速={fmt_gpsfv(current_data['IMU_FV'])}")
    print(f"差分位移XY: X={fmt_csys(current_data['CSYSX'])}, Y={fmt_csys(current_data['CSYSY'])}")
    print(f"差分位移ZR: Z={fmt_csys(current_data['CSYSZ'])}, R={fmt_csys(current_data['CSYSR'])}")
    print(f"差分航向角: {fmt_csysyaw(current_data['CSYSYaw'])}")
    print(f"GPS状态: 定位={current_data.get('STATUS定位', '---')}  定向={current_data.get('STATUS定向', '---')}")
    print("---------------------------wtzn yyx 2025----------------------------------" )

def main():
    print(f"监控串口 {SERIAL_PORT} 波特率 {BAUD_RATE}")
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
        send_at_command(ser)
        buffer = deque(maxlen=8192)
        print(f"catch data ... ")
        while True:
            if ser.in_waiting:
                print("0")
                buffer.extend(ser.read(ser.in_waiting))
                i = 0
                while i + 7 < len(buffer):
                    print("A")
                    print(hex(buffer[i]))
                    if buffer[i] == 0x41 and buffer[i + 1] == 0x54:
                        print("B")
                        can_id = ((buffer[i + 2] << 8) | buffer[i + 3]) >> 5
                        dlc = buffer[i + 6]
                        if i + 7 + dlc > len(buffer):
                            break
                        data_bytes = bytes(buffer[j] for j in range(i + 7, i + 7 + dlc))

                        if can_id == TARGET_CAN_ID_051: parse_0x051(data_bytes)
                        elif can_id == TARGET_CAN_ID_052: parse_0x052(data_bytes)
                        elif can_id == TARGET_CAN_ID_053: parse_0x053(data_bytes)
                        elif can_id == TARGET_CAN_ID_054: parse_0x054(data_bytes)
                        elif can_id == TARGET_CAN_ID_055: parse_0x055(data_bytes)
                        elif can_id == TARGET_CAN_ID_056: parse_0x056(data_bytes)
                        elif can_id == TARGET_CAN_ID_057: parse_0x057(data_bytes)
                        elif can_id == TARGET_CAN_ID_058: parse_0x058(data_bytes)
                        elif can_id == TARGET_CAN_ID_059: parse_0x059(data_bytes)
                        elif can_id == TARGET_CAN_ID_05A: parse_0x05A(data_bytes)
                        elif can_id == TARGET_CAN_ID_05B: parse_0x05B(data_bytes)
                        elif can_id == TARGET_CAN_ID_05C: parse_0x05C(data_bytes)
                        elif can_id == TARGET_CAN_ID_05E: parse_0x05E(data_bytes)
                        elif can_id == TARGET_CAN_ID_05F: parse_0x05F(data_bytes)
                        elif can_id == TARGET_CAN_ID_05D: parse_0x05D(data_bytes)
                        elif can_id == TARGET_CAN_ID_CSYS_XY: parse_0x060(data_bytes)
                        elif can_id == TARGET_CAN_ID_CSYS_ZR: parse_0x061(data_bytes)
                        elif can_id == TARGET_CAN_ID_CSYS_YAW: parse_0x062(data_bytes)
                        elif can_id == TARGET_CAN_ID_STATUS: parse_0x063(data_bytes)

                        i += 7 + dlc
                        print_current_data()
                    else:
                        i += 1
                for _ in range(i):
                    buffer.popleft()
            # else:
            #     print("serial has interupted")
            #     print(ser.in_waiting)
                

if __name__ == "__main__":
    main()
