#!/usr/bin/env python3
"""Minimal SIYI SDK client for the ZR10 gimbal camera (UDP control protocol)."""
import socket
import struct
import sys
import time

CAMERA_IP = "192.168.144.25"
CAMERA_PORT = 37260
STX = b"\x55\x66"


def crc16(data: bytes, crc: int = 0) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(cmd_id: int, data: bytes = b"", seq: int = 0, need_ack: bool = True) -> bytes:
    ctrl = 0x01 if need_ack else 0x00
    body = STX + bytes([ctrl]) + struct.pack("<H", len(data)) + struct.pack("<H", seq) + bytes([cmd_id]) + data
    crc = crc16(body)
    return body + struct.pack("<H", crc)


def parse_packet(raw: bytes):
    if len(raw) < 10 or raw[0:2] != STX:
        return None
    data_len = struct.unpack("<H", raw[3:5])[0]
    seq = struct.unpack("<H", raw[5:7])[0]
    cmd_id = raw[7]
    data = raw[8:8 + data_len]
    return {"cmd_id": cmd_id, "seq": seq, "data": data}


class ZR10:
    def __init__(self, ip=CAMERA_IP, port=CAMERA_PORT, timeout=1.5):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.seq = 0

    def send(self, cmd_id: int, data: bytes = b"") -> bytes | None:
        pkt = build_packet(cmd_id, data, seq=self.seq)
        self.seq = (self.seq + 1) & 0xFFFF
        self.sock.sendto(pkt, self.addr)
        try:
            raw, _ = self.sock.recvfrom(256)
            return raw
        except socket.timeout:
            return None

    def firmware_version(self):
        raw = self.send(0x01)
        if not raw:
            return None
        p = parse_packet(raw)
        if p is None:
            return None
        d = p["data"]
        # code firmware / gimbal firmware / zoom firmware (each 4 bytes, version major.minor.patch + reserved)
        return d.hex()

    def hardware_id(self):
        raw = self.send(0x02)
        if not raw:
            return None
        p = parse_packet(raw)
        if p is None:
            return None
        return p["data"].decode(errors="replace")

    def attitude(self):
        raw = self.send(0x0D)
        if not raw:
            return None
        p = parse_packet(raw)
        if p is None:
            return None
        d = p["data"]
        if len(d) < 12:
            return None
        yaw, pitch, roll, yaw_v, pitch_v, roll_v = struct.unpack("<6h", d[:12])
        return {
            "yaw": yaw / 10.0,
            "pitch": pitch / 10.0,
            "roll": roll / 10.0,
            "yaw_vel": yaw_v / 10.0,
            "pitch_vel": pitch_v / 10.0,
            "roll_vel": roll_v / 10.0,
        }

    def center(self):
        return self.send(0x08, bytes([0x01]))

    def rotate(self, yaw_speed: int, pitch_speed: int):
        # speed range -100..100
        yaw_speed = max(-100, min(100, yaw_speed))
        pitch_speed = max(-100, min(100, pitch_speed))
        return self.send(0x07, struct.pack("<bb", yaw_speed, pitch_speed))

    def take_photo(self):
        return self.send(0x0C, bytes([0x00]))

    def record_toggle(self):
        return self.send(0x0C, bytes([0x02]))

    def zoom(self, direction: int):
        # 1 = zoom in, 0 = stop, -1 = zoom out (signed byte)
        return self.send(0x05, struct.pack("<b", direction))


if __name__ == "__main__":
    cam = ZR10()
    print("Firmware (raw hex):", cam.firmware_version())
    print("Hardware ID:", cam.hardware_id())
    try:
        while True:
            att = cam.attitude()
            if att:
                print(f"pitch={att['pitch']:7.2f}  yaw={att['yaw']:7.2f}  roll={att['roll']:7.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
