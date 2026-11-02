#!/usr/bin/env python3
"""/cmd_vel -> 小车底盘串口（/dev/car，115200）。

协议沿用 ~/workspace/navigation/src/cmd_vel_serial/scripts/cmd_vel_to_serial.py：
Modbus RTU 写多个寄存器（功能码 0x10），float32 字序交换（CDAB），CRC16 低字节在前
    线速度  寄存器 0x0154
    角速度  寄存器 0x015E

与原脚本的区别：以 10 Hz 固定频率发送最新指令；超过 CMD_TIMEOUT 未收到指令发零速；
退出时发零速。空跑模式（~dry_run 或环境变量 NAV_DRY_RUN=1）不打开串口，只记录。
状态以 JSON 发布在 /nav/serial_status。
"""
import json
import os
import struct
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

PORT, BAUD = "/dev/car", 115200
REG_LINEAR, REG_ANGULAR = 0x0154, 0x015E
CMD_TIMEOUT = 0.5


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(reg, value):
    f = struct.pack(">f", value)
    data = [0x01, 0x10, reg >> 8, reg & 0xFF, 0x00, 0x02, 0x04, f[2], f[3], f[0], f[1]]
    crc = crc16(data)
    return bytes(data + [crc & 0xFF, crc >> 8])


class CarSerial:
    def __init__(self):
        self.dry = rospy.get_param("~dry_run", False) or os.environ.get("NAV_DRY_RUN") == "1"
        self.lock = threading.Lock()
        self.v = self.w = 0.0
        self.cmd_t = 0.0
        self.ser = None
        self.error = ""
        self.pub = rospy.Publisher("/nav/serial_status", String, queue_size=1, latch=True)
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=1)

    def on_cmd(self, msg):
        with self.lock:
            self.v, self.w, self.cmd_t = msg.linear.x, msg.angular.z, time.time()

    def ensure_open(self):
        if self.dry or self.ser is not None:
            return True
        try:
            import serial
            self.ser = serial.Serial(PORT, BAUD, timeout=0.05)
            self.error = ""
            rospy.loginfo("opened %s", PORT)
            return True
        except Exception as e:      # 权限不足 / 设备未连接
            self.error = f"无法打开 {PORT}: {e}"
            return False

    def send(self, v, w):
        if self.dry:
            return
        try:
            self.ser.reset_input_buffer()   # 丢弃底盘的应答帧
            self.ser.write(frame(REG_LINEAR, v))
            self.ser.write(frame(REG_ANGULAR, w))
        except Exception as e:
            self.error = f"串口写入失败: {e}"
            try:
                self.ser.close()
            finally:
                self.ser = None

    def run(self):
        rate = rospy.Rate(10)
        last_pub = 0.0
        while not rospy.is_shutdown():
            with self.lock:
                fresh = time.time() - self.cmd_t < CMD_TIMEOUT
                v, w = (self.v, self.w) if fresh else (0.0, 0.0)
            if self.ensure_open():
                self.send(v, w)
            if time.time() - last_pub > 0.5:
                last_pub = time.time()
                self.pub.publish(String(json.dumps(
                    {"dry_run": self.dry, "connected": self.dry or self.ser is not None,
                     "error": self.error, "v": round(v, 3), "w": round(w, 3)}, ensure_ascii=False)))
            rate.sleep()

    def shutdown(self):
        for _ in range(3):
            if self.ser is not None:
                self.send(0.0, 0.0)
            time.sleep(0.05)


def main():
    rospy.init_node("car_serial")
    car = CarSerial()
    rospy.on_shutdown(car.shutdown)
    car.run()


if __name__ == "__main__":
    main()
