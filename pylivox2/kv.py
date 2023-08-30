import struct
from enum import Enum

from loguru import logger


class Key(Enum):
    PCL_DATA_TYPE = 0x0000
    PATTERN_MODE = 0x0001
    POINT_SEND_EN = 0x0003  # HAP Only
    LIDAR_IPCFG = 0x0004
    STATE_INFO_HOST_IPCFG = 0x0005  # Mid-360 Only
    POINTCLOUD_HOST_IPCFG = 0x0006
    IMU_HOST_IPCFG = 0x0007
    INSTALL_ATTITUDE = 0x0012
    BLIND_SPOT_SET = 0x0013  # HAP Only
    FOV_CFG0 = 0x0015  # Mid-360 Only
    FOV_CFG1 = 0x0016  # Mid-360 Only
    FOV_CFG_EN = 0x0017  # Mid-360 Only
    DETECT_MODE = 0x0018  # Mid-360 Only
    FUNC_IO_CFG = 0x0019  # Mid-360 Only
    WORK_TGT_MODE = 0x001A
    GLASS_HEAT_SUPPOR = 0x001B  # HAP Only
    IMU_DATA_EN = 0x001C
    FUSA_EN = 0x001D  # HAP Only
    FORCE_HEAT_EN = 0x001E  # HAP Only
    SN = 0x8000
    PRODUCT_INFO = 0x8001
    VERSION_APP = 0x8002
    VERSION_LOADER = 0x8003
    VERSION_HARDWARE = 0x8004
    MAC = 0x8005
    CUR_WORK_STATE = 0x8006
    CORE_TEMP = 0x8007  # Mid-360 Only
    POWERUP_CNT = 0x8008  # Mid-360 Only
    LOCAL_TIME_NOW = 0x8009  # Mid-360 Only
    LAST_SYNC_TIME = 0x800A  # Mid-360 Only
    TIME_OFFSET = 0x800B  # Mid-360 Only
    TIME_SYNC_TYPE = 0x800C  # Mid-360 Only
    STATUS_CODE = 0x800D  # HAP Only
    LIDAR_DIAG_STATUS = 0x800E
    LIDAR_FLASH_STATUS = 0x800F  # HAP Only
    FW_TYPE = 0x8010
    HMS_CODE = 0x8011  # Mid-360 Only
    CUR_GLASS_HEAT_STATE = 0x8012  # HAP Only


def lidar_diag_status_unpack(buffer: bytes):
    (bf,) = struct.unpack("<H", buffer)
    system_status = bf & 7
    scan_status = bf >> 3 & 7
    ranging_status = bf >> 6 & 7
    communication_status = bf >> 9 & 7
    return system_status, scan_status, ranging_status, communication_status


value_structs = {
    Key.PCL_DATA_TYPE: "<B",
    Key.PATTERN_MODE: "<B",
    Key.POINT_SEND_EN: "<?",
    Key.LIDAR_IPCFG: "<4s4s4s",
    Key.STATE_INFO_HOST_IPCFG: "<4sHH",
    Key.POINTCLOUD_HOST_IPCFG: "<4sHH",
    Key.IMU_HOST_IPCFG: "<4sHH",
    Key.INSTALL_ATTITUDE: "<3f3i",
    Key.BLIND_SPOT_SET: "<L",
    Key.FOV_CFG0: "<5l",
    Key.FOV_CFG1: "<5l",
    Key.FOV_CFG_EN: "<B",
    Key.DETECT_MODE: "<B",
    Key.FUNC_IO_CFG: "<4B",
    Key.WORK_TGT_MODE: "<B",  # TODO: enum
    Key.GLASS_HEAT_SUPPOR: "<?",
    Key.IMU_DATA_EN: "<?",
    Key.FUSA_EN: "<?",
    Key.FORCE_HEAT_EN: "<?",
    Key.SN: "<16s",
    Key.PRODUCT_INFO: "<64s",
    Key.VERSION_APP: "<4B",
    Key.VERSION_LOADER: "<4B",
    Key.VERSION_HARDWARE: "<4B",
    Key.MAC: "<6s",
    Key.CUR_WORK_STATE: "<B",  # TODO: enum
    Key.CORE_TEMP: "<l",
    Key.POWERUP_CNT: "<L",
    Key.LOCAL_TIME_NOW: "<Q",
    Key.LAST_SYNC_TIME: "<Q",
    Key.TIME_OFFSET: "<q",
    Key.TIME_SYNC_TYPE: "<B",
    # Key.STATUS_CODE: "<32s",
    Key.LIDAR_DIAG_STATUS: (None, lidar_diag_status_unpack),
    Key.LIDAR_FLASH_STATUS: "<?",
    Key.FW_TYPE: "<B",
    Key.HMS_CODE: "<8L",
    Key.CUR_GLASS_HEAT_STATE: "<?",
}


def config_req_pack(key_value_list: list[tuple[Key, tuple]]):
    key_num = len(key_value_list)
    buffer = struct.pack("<Hxx", key_num)
    # buffer = bytearray(buffer)
    for key, value in key_value_list:
        if key not in value_structs:
            raise RuntimeError(f"key {key} not in value_structs")
        # struct.pack_into("<HH", buffer, len(buffer), key.value, struct.calcsize(value_structs[key]))
        # struct.pack_into(value_structs[key], buffer, len(buffer), *value)
        buffer += struct.pack("<HH", key.value, struct.calcsize(value_structs[key]))
        buffer += struct.pack(value_structs[key], *value)
    return buffer


def config_req_unpack(buffer: bytes):
    key_num, _ = struct.unpack("<HH", buffer[:4])
    list_span = buffer[4:]
    return kv_list_unpack(key_num, list_span)


def query_req_pack(key_list: list[Key]):
    key_num = len(key_list)
    buffer = struct.pack("<Hxx", key_num)
    for key in key_list:
        # struct.pack_into("<H", buffer, len(buffer), key.value)
        buffer += struct.pack("<H", key.value)
    return buffer


def query_req_unpack(buffer: bytes):
    key_num, _ = struct.unpack("<HH", buffer[:4])
    list_span = buffer[4:]
    return key_list_unpack(key_num, list_span)


def query_ack_unpack(buffer: bytes):
    ret_code, key_num = struct.unpack("<BH", buffer[:3])
    list_span = buffer[3:]
    return ret_code, kv_list_unpack(key_num, list_span)


def push_unpack(buffer: bytes):
    (key_num,) = struct.unpack("<H2x", buffer[:4])
    list_span = buffer[4:]
    return kv_list_unpack(key_num, list_span)


def key_list_unpack(key_num, list_span):
    result = []
    for i in range(key_num):
        key_v = struct.unpack("<H", list_span[:2])[0]
        key = Key(key_v)
        list_span = list_span[2:]
        result.append(key)
    return result


def kv_list_unpack(key_num, list_span):
    result = []
    for i in range(key_num):
        key_v, length = struct.unpack("<HH", list_span[:4])
        key = Key(key_v)
        value = list_span[4:4 + length]
        list_span = list_span[4 + length:]
        if key not in value_structs:
            logger.warning(f"unknown key: {key}, suggest to update value_structs")
            # result += (key, value)
        else:
            pattern = value_structs[key]
            match pattern:
                case str(pack_str):
                    # TODO: check for calcsize
                    # result += (key, struct.unpack(pack_str, value))
                    value = struct.unpack(pack_str, value)
                case (_, unpacker) if callable(unpacker):
                    value = unpacker(value)
                case _:
                    logger.warning(f"unknown pattern: {pattern}")
                    # result += (key, value)
        result.append((key, value))
    return result


if __name__ == '__main__':
    print(config_req_pack([(Key.PCL_DATA_TYPE, (1,)), (Key.PATTERN_MODE, (2,))]).hex())
