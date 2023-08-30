import socket
from enum import Enum
from typing import Callable

from loguru import logger

from pylivox2.kv import Key
from pylivox2.packing import unpack_pack, CmdId, CmdType, make_pack_of_struct_args, SenderType


class WorkStatus(Enum):
    SAMPLING = 0x01
    IDLE = 0x02
    SLEEP = 0x03
    ERROR = 0x04
    SELFCHECK = 0x05
    MOTORSTARUP = 0x06
    MOTORSTOP = 0x07
    UPGRADE = 0x08
    READY = 0x09


class Device:
    def __init__(self, dev_type, serial_number, lidar_ip, cmd_port, host_cmd_port=56000):
        self.dev_type = dev_type
        self.serial_number = serial_number
        self.lidar_ip = lidar_ip
        self.cmd_port = cmd_port
        self.host_cmd_port = host_cmd_port

    def __repr__(self):
        return f"Device{{type: {self.dev_type}, sn: {self.serial_number}, cmd: {self.lidar_ip}:{self.cmd_port}}}"

    @classmethod
    def from_unpacked(cls, unpacked):
        ret_code, dev_type, serial_number_raw, lidar_ip_raw, cmd_port = unpacked[5]
        serial_number_str = serial_number_raw.decode()
        lidar_ip = socket.inet_ntoa(lidar_ip_raw)
        return cls(dev_type, serial_number_str, lidar_ip, cmd_port)

    def generic_req(self, cmd_id: CmdId, *args):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(("", self.host_cmd_port))
            msg = make_pack_of_struct_args(0, 1,
                                           cmd_id, CmdType.REQ, SenderType.HOST,
                                           *args)
            s.sendto(msg, (self.lidar_ip, self.cmd_port))
            while True:
                buffer, (addr, port) = s.recvfrom(1424)
                logger.debug("Received {} bytes from {}:{}", len(buffer), addr, port)
                unpacked = unpack_pack(buffer)
                logger.debug("Unpacked: {}", unpacked)
                if unpacked[2] == cmd_id and unpacked[3] == CmdType.ACK:
                    return unpacked

    def get_ip_that_route_to(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(("", self.host_cmd_port))
            s.connect((self.lidar_ip, self.cmd_port))
            return s.getsockname()[0]

    def get_parameters(self, keys: list[Key]) -> dict[Key, tuple]:
        errno, values = self.generic_req(CmdId.LIDAR_INFORMATION_PARAMETER_INQUIRE, keys)[5]
        if errno != 0:
            logger.warning("Got errno: {}", errno)
        # logger.info("Got values: {}", dict(values))
        for k, v in values:
            logger.debug("{}: {}", k, v)
        return dict(values)

    def get_host_ip_config(self):
        value_dict = self.get_parameters([Key.POINTCLOUD_HOST_IPCFG, Key.IMU_HOST_IPCFG])
        if Key.POINTCLOUD_HOST_IPCFG not in value_dict or Key.IMU_HOST_IPCFG not in value_dict:
            raise ValueError("Host IP config not found")
        host_ip_raw, host_port, lidar_port = value_dict[Key.POINTCLOUD_HOST_IPCFG]
        # host_ip_raw_imu, host_port_imu, lidar_port_imu = value_dict[Key.IMU_HOST_IPCFG]
        # assert host_ip_raw == host_ip_raw_imu
        host_ip = socket.inet_ntoa(host_ip_raw)
        return host_ip, host_port

    def get_work_mode(self):
        value_dict = self.get_parameters([Key.WORK_TGT_MODE])
        if Key.WORK_TGT_MODE not in value_dict:
            raise ValueError("Work mode not found")
        work_mode = WorkStatus(value_dict[Key.WORK_TGT_MODE][0])
        return work_mode

    def watch_heartbeat_until(self, pred: Callable[[tuple], bool]):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(("", self.host_cmd_port))
            logger.info("Listening to heartbeat...")
            s.connect((self.lidar_ip, self.cmd_port))
            while True:
                buffer, (addr, port) = s.recvfrom(1424)
                logger.debug("Received {} bytes from {}:{}", len(buffer), addr, port)
                hb_kv = dict(unpack_pack(buffer)[5])
                diag_status = hb_kv[Key.LIDAR_DIAG_STATUS]
                work_state = WorkStatus(hb_kv[Key.CUR_WORK_STATE][0])
                flash_status = hb_kv[Key.LIDAR_FLASH_STATUS][0]
                logger.debug("diag_status: {}, work_state: {}, flash_status: {}", diag_status, work_state, flash_status)
                status = (diag_status, work_state, flash_status)
                yield status
                if pred(*status):
                    break

    def set_parameters(self, key_value_list: list[tuple[Key, tuple]]):
        logger.debug("Setting parameters: {}", key_value_list)
        errno, error_key = self.generic_req(CmdId.LIDAR_INFORMATION_PARAMETER_CONFIGURATION, key_value_list)[5]
        if errno != 0:
            logger.warning("Got errno: {}", errno)
            logger.warning("First error configuration: {}", key_value_list[error_key])

    def set_host_to(self, host_ip: str, host_port: int = 57000):
        self.set_parameters([(Key.POINTCLOUD_HOST_IPCFG, (socket.inet_aton(host_ip), host_port, 57000))])

    def set_host_to_this(self, host_port: int = 57000):
        auto_host_ip = self.get_ip_that_route_to()
        self.set_host_to(auto_host_ip, host_port)
        logger.info("Set host to {}", auto_host_ip)

    def set_work_mode(self, work: bool):
        self.set_parameters([(Key.WORK_TGT_MODE, ((WorkStatus.SAMPLING if work else WorkStatus.IDLE).value,))])

    def set_point_send(self, enable: bool):
        self.set_parameters([(Key.POINT_SEND_EN, (enable,))])

    @classmethod
    def discover_one(cls):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(("", 56000))
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
            s.sendto(make_pack_of_struct_args(0, 0, CmdId.DEVICE_TYPE_QUERY, CmdType.REQ, SenderType.HOST),
                     ("255.255.255.255", 56000))
            logger.info("Waiting for discover response...")
            while True:
                buffer, (addr, port) = s.recvfrom(1424)
                logger.debug("Received {} bytes from {}:{}", len(buffer), addr, port)
                unpacked = unpack_pack(buffer)
                logger.debug("Unpacked: {}", unpacked)
                if unpacked[2] == CmdId.DEVICE_TYPE_QUERY and unpacked[3] == CmdType.ACK:
                    device = cls.from_unpacked(unpacked)
                    logger.info("Found device: {}", device)
                    return device
