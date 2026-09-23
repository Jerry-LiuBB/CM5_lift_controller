#!/usr/bin/env python3
"""读取 DS/RS 驱动器 F 参数(通过 CANopen SDO)"""

import socket
import struct
import sys

CAN_IFACE = "can1"
NODE_ID   = 1

# F0.y.zzz → 0x2000 + 0x100*y + z
PARAMS = [
    ("F0.2.000", 0x2200, 0),
    ("F0.3.000", 0x2300, 0),
    ("F0.3.001", 0x2301, 0),
    ("F0.4.000", 0x2400, 0),
    ("F0.4.001", 0x2401, 0),
]

def make_sdo_read(node_id, index, subindex):
    """构造 SDO 读请求帧"""
    can_id = 0x600 + node_id
    data = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
    return can_id, data

def parse_sdo_response(frame_id, data):
    """解析 SDO 响应帧,返回 (成功, 值)"""
    if len(data) < 8:
        return False, 0
    cmd = data[0]
    # SDO abort
    if cmd == 0x80:
        abort_code = struct.unpack_from("<I", data, 4)[0]
        return False, abort_code
    # expedited read (cmd >> 4 == 4)
    if (cmd >> 4) == 4:
        value = struct.unpack_from("<I", data, 4)[0]
        return True, value
    # segmented read (cmd >> 4 == 2)
    if (cmd >> 4) == 2:
        value = struct.unpack_from("<I", data, 4)[0]
        return True, value
    return False, 0

def main():
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((CAN_IFACE,))
    sock.settimeout(1.0)

    print(f"Reading {len(PARAMS)} parameters from node {NODE_ID} via {CAN_IFACE}\n")

    for name, index, sub in PARAMS:
        can_id, data = make_sdo_read(NODE_ID, index, sub)
        sock.send(struct.pack("=IB3x8s", can_id, 8, data))

        try:
            raw = sock.recv(16)
            resp_id, dlc, payload = struct.unpack("=IB3x8s", raw)
            ok, value = parse_sdo_response(resp_id, payload)

            if ok:
                signed_val = struct.unpack("<i", struct.pack("<I", value))[0]
                print(f"  {name} (0x{index:04X}) = {value} (0x{value:08X}, signed={signed_val})")
            else:
                print(f"  {name} (0x{index:04X}) = SDO ABORT (code=0x{value:08X})")
        except socket.timeout:
            print(f"  {name} (0x{index:04X}) = TIMEOUT (no response)")
        except Exception as e:
            print(f"  {name} (0x{index:04X}) = ERROR: {e}")

    sock.close()

if __name__ == "__main__":
    main()
