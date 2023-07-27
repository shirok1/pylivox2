import struct
from enum import Enum

from loguru import logger

from pylivox2 import livox_crc, kv


class CmdId(Enum):
    DEVICE_TYPE_QUERY = 0x0000

    LIDAR_INFORMATION_PARAMETER_CONFIGURATION = 0x0100
    LIDAR_INFORMATION_PARAMETER_INQUIRE = 0x0101
    LIDAR_INFORMATION_PUSH = 0x0102

    CONTROL_REBOOT = 0x0200

    LOG_PUSH = 0x0300
    LOG_CONFIGURATION = 0x0301

    GENERAL_UPDATE_REQUEST = 0x0400
    GENERAL_UPDATE_TRANSMIT = 0x0401
    GENERAL_UPDATE_TRANSMIT_END = 0x0402
    GENERAL_UPDATE_STATUS = 0x0403


req_structs = {
    CmdId.DEVICE_TYPE_QUERY: "<",
    CmdId.LIDAR_INFORMATION_PARAMETER_CONFIGURATION: (kv.config_req_pack, kv.config_req_unpack),
    CmdId.LIDAR_INFORMATION_PARAMETER_INQUIRE: (kv.query_req_pack, kv.query_req_unpack),
    CmdId.LIDAR_INFORMATION_PUSH: (None, kv.push_unpack),
    CmdId.CONTROL_REBOOT: "<H",
}
ack_structs = {
    CmdId.DEVICE_TYPE_QUERY: "<BB16s4sH",
    CmdId.LIDAR_INFORMATION_PARAMETER_CONFIGURATION: "<BH",
    CmdId.LIDAR_INFORMATION_PARAMETER_INQUIRE: (None, kv.query_ack_unpack),
    CmdId.CONTROL_REBOOT: "<B",
}


class CmdType(Enum):
    REQ = 0
    ACK = 1


class SenderType(Enum):
    HOST = 0
    LIDAR = 1


def make_header(version: int, length: int, seq_num: int, cmd_id: int,
                cmd_type: CmdType, sender_type: SenderType, crc32: int):
    buffer = struct.pack("<BBHLHBBxxxxxx",
                         0xAA, version, length, seq_num, cmd_id,
                         cmd_type.value, sender_type.value)
    crc16 = livox_crc.crc16.checksum(buffer)
    return buffer + struct.pack("<HL", crc16, crc32)


def make_pack_of_bytes(version: int, seq_num: int, cmd_id: int,
                       cmd_type: CmdType, sender_type: SenderType, data):
    return make_header(version, len(data) + 24, seq_num, cmd_id,
                       cmd_type, sender_type,
                       livox_crc.crc32.checksum(data)) + data


@logger.catch()
def make_pack_of_struct_args(version: int, seq_num: int, cmd_id: CmdId,
                             cmd_type: CmdType, sender_type: SenderType, *args):
    structs = req_structs if cmd_type == CmdType.REQ else ack_structs
    if cmd_id not in structs:
        raise RuntimeError(f"struct descriptor for {cmd_id} not found in structs of {cmd_type}")

    match structs[cmd_id]:
        case str(pack_str):
            data = struct.pack(pack_str, *args)
        case (packer, _) if callable(packer):
            data = packer(*args)
        case _:
            raise RuntimeError(f"struct descriptor for {cmd_id} has unknown pattern: {structs[cmd_id]}")

    return make_pack_of_bytes(version, seq_num, cmd_id.value,
                              cmd_type, sender_type,
                              data)


def unpack_pack_bytes(buffer: bytes):
    _, version, length, seq_num, cmd_id_value, cmd_type_value, sender_type_value = struct.unpack("<BBHLHBBxxxxxx",
                                                                                                 buffer[:18])
    # header_tuple = struct.unpack("<BBHLHBBxxxxxx", buffer[:18])
    crc16, crc32 = struct.unpack("<HL", buffer[18:24])
    data = buffer[24:]
    if length != len(data) + 24:
        logger.warning(f"wrong length: {length} != {len(data) + 24}")
    if not livox_crc.crc16.verify(buffer[:18], crc16):
        logger.warning("Wrong crc16")
    if not livox_crc.crc32.verify(data, crc32):
        logger.warning("Wrong crc32")

    return version, seq_num, CmdId(cmd_id_value), CmdType(cmd_type_value), SenderType(sender_type_value), data


@logger.catch()
def unpack_pack(buffer: bytes):
    version, seq_num, cmd_id, cmd_type, sender_type, data = unpack_pack_bytes(buffer)

    structs = req_structs if cmd_type == CmdType.REQ else ack_structs
    if cmd_id not in structs:
        raise RuntimeError(f"struct descriptor for {cmd_id} not found in structs of {cmd_type}")
    match structs[cmd_id]:
        case str(pack_str):
            data_unpacked = struct.unpack(pack_str, data)
        case (_, unpacker) if callable(unpacker):
            data_unpacked = unpacker(data)
        case _:
            raise RuntimeError(f"struct descriptor for {cmd_id} has unknown pattern: {structs[cmd_id]}")
    return version, seq_num, cmd_id, cmd_type, sender_type, data_unpacked
