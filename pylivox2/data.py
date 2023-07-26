import struct

import numpy as np
from loguru import logger

import pylivox2.livox_crc


def decode_data(buffer: bytes):
    version, length, time_interval, dot_num, udp_cnt, frame_cnt, data_type, time_type, pack_info, crc32, timestamp = struct.unpack(
        "<BHHHHBBBB11xLQ", buffer[:36])
    crc32_span = buffer[28:]
    if not pylivox2.livox_crc.crc32.verify(crc32_span, crc32):
        logger.warning("Wrong crc32")
    data = buffer[36:]
    if data_type == 0:
        # imu data
        return struct.unpack("<6f", data)
    elif data_type == 1:
        # pc data 1
        # coord is int32, stands for mm
        coords = np.zeros((96, 3), dtype=np.int32)
        reflectivity = np.zeros((96,), dtype=np.int8)
        tag_info = np.zeros((96,), dtype=np.int8)
        for i in range(96):
            coords[i, 0], coords[i, 1], coords[i, 2], reflectivity[i], tag_info[i] = struct.unpack(
                "<lllbb", data[i * 14:i * 14 + 14])
        return coords, reflectivity, tag_info
    elif data_type == 2:
        # pc data 2
        # coord is int16, stands for cm aka 10mm
        coords = np.zeros((96, 3), dtype=np.int16)
        reflectivity = np.zeros((96,), dtype=np.int8)
        tag_info = np.zeros((96,), dtype=np.int8)
        for i in range(96):
            coords[i, 0], coords[i, 1], coords[i, 2], reflectivity[i], tag_info[i] = struct.unpack(
                "<hhhbb", data[i * 8:i * 8 + 8])
        return coords, reflectivity, tag_info
