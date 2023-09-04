from datetime import datetime, timedelta

import rclpy
from loguru import logger
from rclpy.node import Node

from pylivox2.device import Device, WorkStatus
from pylivox2.kv import Key

from .loguru_sink import NodeLoguruSink


class Livox2Daemon(Node):
    STATIC_KEYS = [
        Key.PCL_DATA_TYPE,
        Key.PATTERN_MODE,
        Key.LIDAR_IPCFG,
        Key.POINTCLOUD_HOST_IPCFG,
        Key.STATE_INFO_HOST_IPCFG,
        Key.IMU_HOST_IPCFG,
        Key.INSTALL_ATTITUDE,
        Key.FOV_CFG0,
        Key.FOV_CFG1,
        Key.FOV_CFG_EN,
        Key.DETECT_MODE,
        Key.FUNC_IO_CFG,
        Key.IMU_DATA_EN,
        Key.SN,
        Key.PRODUCT_INFO,
        Key.VERSION_APP,
        Key.VERSION_LOADER,
        Key.VERSION_HARDWARE,
        Key.MAC,
        Key.POWERUP_CNT,
        Key.FW_TYPE,
    ]

    def __init__(self):
        super().__init__("livox2_daemon")
        NodeLoguruSink(self).set_as_only_logger()
        self.device = Device(9, "sfsf", "192.168.1.3", 56100)
        logger.info(f"dd: {self.device}")
        # ret = self.device.get_parameters([Key.VERSION_APP, Key.VERSION_LOADER, Key.VERSION_HARDWARE, Key.LIDAR_IPCFG])
        # logger.info("App Version: {}.{}.{}.{}", *ret[Key.VERSION_APP])
        # logger.info("Loader Version: {}.{}.{}.{}", *ret[Key.VERSION_LOADER])
        # logger.info("Hardware Version: {}.{}.{}.{}", *ret[Key.VERSION_HARDWARE])
        # logger.info("Lidar IP: {}", inet_ntoa(ret[Key.LIDAR_IPCFG][0]))
        # logger.info("Lidar Netmask: {}", inet_ntoa(ret[Key.LIDAR_IPCFG][1]))
        # logger.info("Lidar Gateway: {}", inet_ntoa(ret[Key.LIDAR_IPCFG][2]))
        for kv in self.device.watch_heartbeat_until(lambda _: True):
            static_kv = {key: kv[key] for key in kv.keys() & self.STATIC_KEYS}
            logger.info(f"static_kv: {static_kv}")
            if Key.SN in kv.keys():
                logger.info(f"SN: {kv[Key.SN][0].decode()}")
            if Key.PRODUCT_INFO in kv.keys():
                logger.info(f"{kv[Key.PRODUCT_INFO][0].decode()}")

    def go(self):
        self.device.set_host_to_this()
        for kv in self.device.watch_heartbeat_until(lambda _: not rclpy.ok()):
            dynamic_kv = {key: kv[key] for key in (
                set(kv.keys()) - set(self.STATIC_KEYS))}
            logger.info(f"dynamic_kv: {dynamic_kv}")
            logger.info("WORK_TGT_MODE: {}",
                        WorkStatus(kv[Key.WORK_TGT_MODE][0]))
            logger.info("CUR_WORK_STATE: {}",
                        WorkStatus(kv[Key.CUR_WORK_STATE][0]))
            if Key.LIDAR_DIAG_STATUS in kv.keys():
                codes = dynamic_kv[Key.LIDAR_DIAG_STATUS]
                module_names = ["System", "Scan", "Ranging", "Communication"]
                for index in range(0, 4):
                    match codes[index]:
                        case 1:
                            logger.warning(
                                f"DIAG_STATUS: {module_names[index]} Module")
                        case 2:
                            logger.error(
                                f"DIAG_STATUS: {module_names[index]} Module")
                        case 3:
                            logger.critical(
                                f"DIAG_STATUS: {module_names[index]} Module")
            if Key.HMS_CODE in kv.keys():
                codes = kv[Key.HMS_CODE]
                for start_index in range(0, 24, 3):
                    level, _, abnormal_id = codes[start_index: start_index + 3]
                    match level:
                        case 0x01:
                            logger.info(f"HMS: 0x{abnormal_id:04x}")
                        case 0x02:
                            logger.warning(f"HMS: 0x{abnormal_id:04x}")
                        case 0x03:
                            logger.error(f"HMS: 0x{abnormal_id:04x}")
                        case 0x04:
                            logger.critical(f"HMS: 0x{abnormal_id:04x}")
            if Key.CORE_TEMP in kv.keys():
                logger.info(f"CORE_TEMP: {kv[Key.CORE_TEMP][0]/100}℃")
            if Key.LOCAL_TIME_NOW in kv.keys():
                local_time_now = \
                    datetime.fromtimestamp(
                        kv[Key.LOCAL_TIME_NOW][0] / 1000_000_000)
                last_sync_time = \
                    timedelta(microseconds=kv[Key.LAST_SYNC_TIME][0])
                # last_sync_time = datetime.fromtimestamp(kv[Key.LAST_SYNC_TIME][0]/1000_000)
                logger.info(
                    f"LOCAL_TIME_NOW: {local_time_now} ({local_time_now - datetime.fromtimestamp(0)})")
                logger.info(
                    f"TIME_SYNC_TYPE: {kv[Key.TIME_SYNC_TYPE][0]}, LAST_SYNC_TIME: {last_sync_time}, TIME_OFFSET: {kv[Key.TIME_OFFSET][0]/1000_000}ms")


@logger.catch()
def main(args=None):
    rclpy.init(args=args)

    node = Livox2Daemon()

    node.go()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
