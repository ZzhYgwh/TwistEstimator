# coding:UTF-8
"""
WT-RTKAC 全功能传感器读取系统
版本：v1.1

"""

import time
import serial
import threading
import struct
from crcmod import crcmod

# 配置参数
CONFIG = {
    'port': '/dev/ttyUSB0',  # 串口号
    'baudrate': 230400,  # 波特率
    'device_addr': 0x50,  # 设备地址
    'timeout': 0.5,  # 超时时间(秒)

    # 寄存器配置
    'angle_reg': 0x3D,  # 角度起始寄存器
    'angle_count': 3,  # 角度寄存器数量
    'temp_reg': 0x40,  # 温度寄存器
    'gps_reg': 0x41,  # GPS起始寄存器
    'gps_count': 4,  # GPS寄存器数量
    'altitude_reg': 0x45,  # 高度起始寄存器
    'altitude_count': 2,  # 高度寄存器数量(2个寄存器=4字节)
    'yaw_reg': 0x47,  # GPS航向角寄存器地址
    'yaw_count': 1,  # 读取1个寄存器（16位）
    'speed_reg': 0x48,  # 速度起始寄存器地址
    'speed_count': 2,  # 读取2个寄存器（GPSVL+GPSVH）
    'svnum_reg': 0x4A,  # 卫星数寄存器地址
    'svnum_count': 1,  # 读取1个寄存器（16位）

    # 惯导寄存器配置
    'ins_gps_reg': 0x4B,  # 惯导GPS起始寄存器 (Lon2L~Lat2H)
    'ins_gps_count': 4,  # 4个寄存器（8字节）
    'ins_yaw_reg': 0x51,  # 惯导航向角寄存器 (GPSYAW2)
    'ins_yaw_count': 1,  # 读取1个寄存器（16位）
    'ins_speed_reg': 0x52,  # 惯导地速寄存器 (GPSFVL~GPSFVH)
    'ins_speed_count': 2,  # 读取2个寄存器（4字节）

    # 状态寄存器
    'netcsq_reg': 0x28,  # 4G信号质量寄存器
    'netcsq_count': 1,  # 1个寄存器（16位）
    'netrxcnt_reg': 0x2A,  # 4G数据量寄存器
    'netrxcnt_count': 1,  # 1个寄存器（16位）
    'pdop_reg': 0xDB,  # PDOP寄存器
    'pdop_count': 2,  # 2个寄存器
    'difage_reg': 0xDE,  # 差分龄期寄存器
    'difage_count': 1,  # 1个寄存器（16位）
    'batvolt_reg': 0xDF,  # 电池电压寄存器
    'batvolt_count': 1,  # 1个寄存器（16位）

    # 差分位移寄存器 (CSYSXL~CSYSRH)
    'csysx_reg': 0x54,  # X轴差分位移起始寄存器
    'csysy_reg': 0x56,  # Y轴差分位移起始寄存器
    'csysz_reg': 0x58,  # Z轴差分位移起始寄存器
    'csysr_reg': 0x5A,  # 差分距离起始寄存器
    'csys_count': 2,     # 每个参数占2个寄存器(4字节)

    'status_reg': 0x1B,  # 天线状态寄存器
    'status_count': 1,   # 1个寄存器（16位）
}

# CRC16计算器
crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF)


class FullFeatureReader:
    def __init__(self):
        self.serial_port = None
        self.running = False
        self.data = {
            'angles': {'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.0},
            'temperature': 0.0,
            'gps': {
                'longitude': 0.0,
                'latitude': 0.0,
                'altitude': 0.0,
                'yaw': 0.0,  # GPS航向角
                'speed': 0.0,  # 地速（单位：km/h）
                'satellites': 0,
                'pdop': 0.0,  # 位置定位精度
                'difage': 0  # 差分龄期
            },
            # 惯导数据结构
            'ins': {
                'longitude': 0.0,
                'latitude': 0.0,
                'altitude': 0.0,  # 使用GPS高度值
                'yaw': 0.0,  # 惯导航向角
                'speed': 0.0  # 惯导地速
            },
            # 状态数据
            'status': {
                'voltage': 0.0,  # 电压值 (单位: V)
                'netcsq': 0,  # 4G信号质量 (0-31)
                'netrxcnt': 0  # 4G数据量 (单位: KB)
            },
            # 差分位移数据
            'displacement': {
                'x': 0.0,  # X轴差分位移 (m)
                'y': 0.0,  # Y轴差分位移 (m)
                'z': 0.0,  # Z轴差分位移 (m)
                'distance': 0.0  # 差分距离 (m)
            },
            # 天线状态数据
            'status': {
                'main_antenna': "未知",  # 主天线状态
                'sub_antenna': "未知",  # 从天线状态
            }
        }
        self.lock = threading.Lock()

    def _build_request(self, reg_addr, reg_count):
        """构建Modbus请求帧"""
        addr_high, addr_low = divmod(reg_addr, 0x100)
        len_high, len_low = divmod(reg_count, 0x100)

        frame = bytes([
            CONFIG['device_addr'],
            0x03,
            addr_high, addr_low,
            len_high, len_low
        ])

        crc = crc16(frame)
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def _read_registers(self, reg_addr, reg_count):
        """读取寄存器数据"""
        try:
            request = self._build_request(reg_addr, reg_count)
            self.serial_port.reset_input_buffer()
            self.serial_port.write(request)

            expected_len = 5 + 2 * reg_count
            response = self.serial_port.read(expected_len)

            if len(response) != expected_len:
                print(f"响应长度错误: 需要{expected_len} 收到{len(response)}")
                return None

            # CRC校验
            crc_received = response[-2] | (response[-1] << 8)
            crc_calculated = crc16(response[:-2])
            if crc_received != crc_calculated:
                print(f"CRC校验失败: 接收{crc_received:04X} 计算{crc_calculated:04X}")
                return None

            return response[3:-2]  # 返回数据部分

        except Exception as e:
            print(f"读取错误: {str(e)}")
            return None

    def _parse_float(self, data):
        """解析32位浮点数"""
        if len(data) < 4:
            return 0.0
        # 将4字节转换为float (大端模式)
        return struct.unpack('>f', bytes(data))[0]

    def _parse_angle(self, high, low):
        """解析角度值"""
        raw = (high << 8) | low
        if raw > 32767: raw -= 65536
        return raw / 32768.0 * 180.0

    def _parse_temp(self, high, low):
        """解析温度值"""
        raw = (high << 8) | low
        if raw > 32767: raw -= 65536
        return raw / 100.0

    def _parse_gps_coordinate(self, data):
        """解析GPS坐标数据（经度或纬度）"""
        if len(data) < 4:
            return 0.0

        # 字节交换（低字节在前）
        swapped = bytes([data[1], data[0], data[3], data[2]])

        # 转换为32位整数
        raw_value = int.from_bytes(swapped, byteorder='little', signed=True)

        # 转换为度（除以1e7）
        degrees = raw_value / 1e7

        return degrees

    def _parse_gps(self, data):
        """解析GPS数据（符合NMEA0183标准）"""
        if len(data) < 8:
            return False

        try:
            # 解析经度（前4字节）
            longitude = self._parse_gps_coordinate(data[0:4])

            # 解析纬度（后4字节）
            latitude = self._parse_gps_coordinate(data[4:8])

            with self.lock:
                self.data['gps']['longitude'] = longitude
                self.data['gps']['latitude'] = latitude
            return True
        except Exception as e:
            print(f"解析GPS坐标错误: {e}")
            return False

    def _parse_altitude(self, data):
        """解析GPS高度"""
        if len(data) < 4:
            return 0.0  # 返回默认值
        float_bytes = struct.pack('4B', data[1], data[0], data[3], data[2])
        try:
            # 小端序解析32位浮点数
            raw_value = struct.unpack('<f', float_bytes)[0]
            return raw_value  # 根据实际情况决定是否需要除以10.0
        except struct.error:
            return 0.0

    def _parse_yaw(self, high, low):
        """解析GPS航向角"""
        raw_value = (high << 8) | low
        yaw = raw_value / 10.0
        return round(yaw, 1)

    def _parse_speed(self, data):
        """
        解析地速（单位转换：节 → km/h）
        """
        if len(data) < 4:
            return 0.0  # 返回默认值

        # 小端序解析32位浮点数
        float_bytes = struct.pack('4B', data[1], data[0], data[3], data[2])
        try:
            # 解析为浮点数（单位：节）
            speed_knots = struct.unpack('<f', float_bytes)[0]
            # 转换为km/h（1节=1.852 km/h）
            speed_kmh = speed_knots * 1.852

            # 有效性检查（0-2000 km/h）
            if speed_kmh < 0 or speed_kmh > 2000:
                return 0.0
            return speed_kmh
        except struct.error:
            return 0.0


    def _parse_svnum(self, data):
        """解析GPS卫星数"""
        if len(data) < 2:
            return False

        svnum = (data[0] << 8) | data[1]
        if svnum > 60: svnum = 0  # 有效性检查

        with self.lock:
            self.data['gps']['satellites'] = svnum
        return True

    def latlon2deg(self, data):
        """经纬度转化为度分格式"""
        iDeg = int(data / 100)
        dDeg = (data - float(iDeg) * 100) / 60.0 + iDeg
        return dDeg

    def _parse_ins_gps(self, data):
        """解析惯导GPS数据（处理零值情况）"""
        if len(data) < 8:
            print(f"数据长度不足8字节: {len(data)}")
            return False

        try:
            # 检查是否为无效数据（全零或特定模式）
            if all(b == 0 for b in data) or data == [0xFF] * 8:
                print("警告：接收到无效数据（全零或全FF）")
                return False

            # 解析经度（前4字节）
            longitude = self._parse_gps_coordinate(data[0:4])

            # 解析纬度（后4字节）
            latitude = self._parse_gps_coordinate(data[4:8])

            with self.lock:
                self.data['ins']['longitude'] = longitude
                self.data['ins']['latitude'] = latitude
            return True
        except Exception as e:
            print(f"解析惯导GPS数据错误: {e}")
            return False

    def _parse_ins_yaw(self, high, low):
        """解析惯导航向角"""
        raw_value = (high << 8) | low
        yaw = raw_value / 10.0  # 假设单位是0.1度
        return round(yaw, 1)

    def _parse_pdop(self, data):
        """解析位置定位精度(PDOP)"""
        if len(data) < 4:
            return 99.99  # 默认无效值

        try:
            # 小端字节序解析(注意寄存器顺序)
            float_bytes = bytes([data[1], data[0], data[3], data[2]])
            pdop_value = struct.unpack('<f', float_bytes)[0]

            # 有效性检查 (典型PDOP范围0.5-50.0)
            if 0.5 <= pdop_value <= 50.0:
                return pdop_value
            return 99.99
        except Exception as e:
            return 99.99

    def _parse_difage(self, data):
        """解析差分龄期(DIFAGE)"""
        if len(data) < 2:
            return 9999  # 默认无效值

        return (data[0] << 8) | data[1]  # 直接返回整数值

    def _parse_batvolt(self, data):
        """解析电池电压(BATVOLT)"""
        if len(data) < 2:
            return 0.0

        raw_value = (data[0] << 8) | data[1]
        return raw_value / 10.0  # 转换为伏特

    def _parse_netcsq(self, data):
        """解析4G信号质量(NETCSQ)"""
        if len(data) < 2:
            return 0

        return (data[0] << 8) | data[1]  # 信号质量值 (0-31)

    def _parse_netrxcnt(self, data):
        """解析4G数据量(NETRXCNT)"""
        if len(data) < 2:
            return 0

        return (data[0] << 8) | data[1]  # 数据量 (单位: KB)

    def _parse_antenna_status(self, status_value):
        """解析天线状态值"""
        status_map = {
            0: "初始化",
            1: "单点定位",
            2: "码差分",
            4: "固定解",
            5: "浮点解"
        }
        main_status = (status_value >> 8) & 0xFF
        sub_status = status_value & 0xFF
        return (status_map.get(main_status, f"未知({main_status})"),
                status_map.get(sub_status, f"未知({sub_status})"))

    def _parse_displacement(self, data):
        """解析差分位移数据(32位整数，实际值需要除以1000)"""
        if len(data) < 4:
            return float('nan')

        try:
            # 小端字节序解析32位有符号整数
            int_bytes = bytes([data[1], data[0], data[3], data[2]])
            raw_value = int.from_bytes(int_bytes, byteorder='little', signed=True)

            # 转换为实际值（除以1000）
            value = raw_value / 1000.0

            # 有效性检查
            if abs(value) > 10000:  # 假设位移不会超过10000米
                return float('nan')
            return value
        except Exception as e:
            print(f"解析差分位移错误: {e}")
            return float('nan')

    def _update_displacement(self):
        """更新差分位移数据 - 使用新的解析方法"""
        # X轴位移 (寄存器54L-55H)
        data = self._read_registers(CONFIG['csysx_reg'], CONFIG['csys_count'])
        if data and len(data) >= 4:
            with self.lock:
                self.data['displacement']['x'] = self._parse_displacement(data)
        else:
            with self.lock:
                self.data['displacement']['x'] = float('nan')

        # Y轴位移 (寄存器56L-57H)
        data = self._read_registers(CONFIG['csysy_reg'], CONFIG['csys_count'])
        if data and len(data) >= 4:
            with self.lock:
                self.data['displacement']['y'] = self._parse_displacement(data)
        else:
            with self.lock:
                self.data['displacement']['y'] = float('nan')

        # Z轴位移 (寄存器58L-59H)
        data = self._read_registers(CONFIG['csysz_reg'], CONFIG['csys_count'])
        if data and len(data) >= 4:
            with self.lock:
                self.data['displacement']['z'] = self._parse_displacement(data)
        else:
            with self.lock:
                self.data['displacement']['z'] = float('nan')

        # 差分距离 (寄存器5AL-5BH)
        data = self._read_registers(CONFIG['csysr_reg'], CONFIG['csys_count'])
        if data and len(data) >= 4:
            with self.lock:
                self.data['displacement']['distance'] = self._parse_displacement(data)
        else:
            with self.lock:
                self.data['displacement']['distance'] = float('nan')

    def _update_angles(self):
        """更新角度数据"""
        data = self._read_registers(CONFIG['angle_reg'], CONFIG['angle_count'])
        if data and len(data) >= 6:
            with self.lock:
                self.data['angles']['Roll'] = self._parse_angle(data[0], data[1])
                self.data['angles']['Pitch'] = self._parse_angle(data[2], data[3])
                self.data['angles']['Yaw'] = self._parse_angle(data[4], data[5])

    def _update_temp(self):
        """更新温度数据"""
        data = self._read_registers(CONFIG['temp_reg'], 1)
        if data and len(data) >= 2:
            with self.lock:
                self.data['temperature'] = self._parse_temp(data[0], data[1])

    def _update_gps(self):
        """更新GPS数据"""
        data = self._read_registers(CONFIG['gps_reg'], CONFIG['gps_count'])
        if data:
            self._parse_gps(data)

    def _update_altitude(self):
        """更新高度数据"""
        data = self._read_registers(CONFIG['altitude_reg'], CONFIG['altitude_count'])
        if data and len(data) >= 4:  # 4字节的浮点数
            with self.lock:
                altitude = self._parse_altitude(data)  # 传入整个数据
                self.data['gps']['altitude'] = altitude
                self.data['ins']['altitude'] = altitude

    def _update_yaw(self):
        """更新GPS航向角数据"""
        data = self._read_registers(CONFIG['yaw_reg'], CONFIG['yaw_count'])
        if data and len(data) >= 2:
            with self.lock:
                self.data['gps']['yaw'] = self._parse_yaw(data[0], data[1])

    def _update_speed(self):
        """更新GPS速度数据"""
        data = self._read_registers(CONFIG['speed_reg'], CONFIG['speed_count'])
        if data and len(data) >= 4:
            with self.lock:
                speed = self._parse_speed(data)
                self.data['gps']['speed'] = speed

    def _update_ins_speed(self):
        """更新惯导地速数据"""
        data = self._read_registers(CONFIG['ins_speed_reg'], CONFIG['ins_speed_count'])
        if data and len(data) >= 4:
            with self.lock:
                self.data['ins']['speed'] = self._parse_speed(data)

    def _update_ins_yaw(self):
        """更新惯导航向角数据"""
        data = self._read_registers(CONFIG['ins_yaw_reg'], CONFIG['ins_yaw_count'])
        if data and len(data) >= 2:
            with self.lock:
                self.data['ins']['yaw'] = self._parse_ins_yaw(data[0], data[1])

    def _update_svnum(self):
        """更新GPS卫星数数据"""
        data = self._read_registers(CONFIG['svnum_reg'], CONFIG['svnum_count'])
        if data:
            self._parse_svnum(data)

    def _update_ins_gps(self):
        """更新惯导经纬度数据"""
        data = self._read_registers(CONFIG['ins_gps_reg'], CONFIG['ins_gps_count'])
        if data:
            self._parse_ins_gps(data)

    def _update_ins_yaw(self):
        """更新惯导航向角数据"""
        data = self._read_registers(CONFIG['ins_yaw_reg'], CONFIG['ins_yaw_count'])
        if data and len(data) >= 2:
            with self.lock:
                self.data['ins']['yaw'] = self._parse_yaw(data[0], data[1])

    def _update_netcsq(self):
        """更新4G信号质量"""
        data = self._read_registers(CONFIG['netcsq_reg'], CONFIG['netcsq_count'])
        if data and len(data) >= 2:
            with self.lock:
                self.data['status']['netcsq'] = self._parse_netcsq(data)

    def _update_netrxcnt(self):
        """更新4G数据量"""
        data = self._read_registers(CONFIG['netrxcnt_reg'], CONFIG['netrxcnt_count'])
        if data and len(data) >= 2:
            with self.lock:
                self.data['status']['netrxcnt'] = self._parse_netrxcnt(data)

    def _update_pdop(self):
        """更新定位精度数据"""
        data = self._read_registers(CONFIG['pdop_reg'], CONFIG['pdop_count'])
        if data and len(data) >= 4:
            with self.lock:
                self.data['gps']['pdop'] = self._parse_pdop(data)

    def _update_difage(self):
        """更新差分龄期(DIFAGE)"""
        data = self._read_registers(CONFIG['difage_reg'], CONFIG['difage_count'])
        if data and len(data) >= 2:
            with self.lock:
                self.data['gps']['difage'] = self._parse_difage(data)

    def _update_batvolt(self):
        """更新电池电压"""
        data = self._read_registers(CONFIG['batvolt_reg'], CONFIG['batvolt_count'])
        if data and len(data) >= 2:
            with self.lock:
                self.data['status']['voltage'] = self._parse_batvolt(data)

    def _update_antenna_status(self):
        """更新天线状态数据"""
        data = self._read_registers(CONFIG['status_reg'], CONFIG['status_count'])
        if data and len(data) >= 2:
            status_value = (data[0] << 8) | data[1]
            with self.lock:
                main, sub = self._parse_antenna_status(status_value)
                self.data['status']['main_antenna'] = main
                self.data['status']['sub_antenna'] = sub

    def _monitor_loop(self):
        while self.running:
            try:
                # 更新所有传感器数据
                self._update_angles()
                self._update_temp()
                self._update_gps()
                self._update_altitude()
                self._update_yaw()
                self._update_speed()
                self._update_svnum()
                self._update_ins_gps()
                self._update_ins_yaw()
                self._update_ins_speed()

                # 更新状态数据
                self._update_netcsq()
                self._update_netrxcnt()
                self._update_pdop()
                self._update_difage()
                self._update_batvolt()

                # 更新差分位移数据
                self._update_displacement()

                # 更新天线状态数据
                self._update_antenna_status()

                with self.lock:
                    # 根据卫星数决定速度显示
                    gps_speed_display = f"{self.data['gps']['speed']:.2f}"
                    ins_speed_display = f"{self.data['ins']['speed']:.2f}"

                    if self.data['gps']['satellites'] < 4:
                        gps_speed_display = "--"
                        ins_speed_display = "--"

                    # 格式化网络状态
                    net_status = f"4G：信号质量={self.data['status']['netcsq']} 数据量={self.data['status']['netrxcnt']}KB"

                    # 格式化电池状态
                    voltage = self.data['status']['voltage']
                    bat_status = f"电池: {voltage:.2f}V"

                    # 格式化GPS精度
                    gps_accuracy = f"精度: PDOP={self.data['gps']['pdop']:.2f} 差分龄期={self.data['gps']['difage']}s"

                    # 格式化差分位移
                    disp = self.data['displacement']
                    disp_status = (f"差分: X={disp['x']:.3f}m Y={disp['y']:.3f}m "
                                 f"Z={disp['z']:.3f}m 距离={disp['distance']:.3f}m")

                    print(f"\r[状态] "
                          f"角度: X={self.data['angles']['Roll']:.2f}° "
                          f"Y={self.data['angles']['Pitch']:.2f}° "
                          f"Z={self.data['angles']['Yaw']:.2f}° | "
                          f"温度: {self.data['temperature']:.1f}℃ | "
                          f"GPS: 经度={self.data['gps']['longitude']:.6f} "
                          f"纬度={self.data['gps']['latitude']:.6f} "
                          f"高度={self.data['gps']['altitude']:.1f}m "
                          f"航向角={self.data['gps']['yaw']:.1f}° "
                          f"地速={gps_speed_display}km/h "
                          f"卫星数={self.data['gps']['satellites']:2d} | "
                          f"惯导: 经度={self.data['ins']['longitude']:.6f} "
                          f"纬度={self.data['ins']['latitude']:.6f} "
                          f"高度={self.data['ins']['altitude']:.1f}m "
                          f"航向角={self.data['ins']['yaw']:.1f}° ",
                          f"地速={ins_speed_display}km/h | "
                          f"天线: 定位状态={self.data['status']['main_antenna']} "
                          f"定向状态={self.data['status']['sub_antenna']} | "
                          f"{net_status} | "
                          f"{bat_status} | "
                          f"{gps_accuracy} | "
                          f"{disp_status}",
                          end='')

                time.sleep(0.2)  # 提高更新频率到5Hz

            except Exception as e:
                print(f"监控错误: {str(e)}")
                self.stop()

    def start(self):
        """启动系统"""
        try:
            print("---------------------------wtzn yyx 2025-----------------------------")
            print("WT-RTKAC-485 v1.1")
            print(f"串口: {CONFIG['port']} @ {CONFIG['baudrate']} bps")
            print(f"设备地址: 0x{CONFIG['device_addr']:02X}")
            print("---------------------------wtzn yyx 2025-----------------------------")


            # 初始化串口
            self.serial_port = serial.Serial(
                port=CONFIG['port'],
                baudrate=CONFIG['baudrate'],
                timeout=CONFIG['timeout'],
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )

            if not self.serial_port.is_open:
                raise ConnectionError("串口打开失败")

            print("开始数据采集... (Ctrl+F2停止)")
            self.running = True
            threading.Thread(target=self._monitor_loop, daemon=True).start()

            while self.running:
                time.sleep(1)

        except KeyboardInterrupt:
            print("\n用户终止操作")
        except Exception as e:
            print(f"启动失败: {str(e)}")
        finally:
            self.stop()

    def stop(self):
        """安全停止系统"""
        if self.running:
            self.running = False
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            print("\n系统已安全关闭")


if __name__ == '__main__':
    reader = FullFeatureReader()
    reader.start()