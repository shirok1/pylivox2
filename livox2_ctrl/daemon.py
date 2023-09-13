from datetime import datetime, timedelta

import rclpy
from loguru import logger
from rclpy.node import Node
from std_msgs.msg import Float32, String

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

    HMS_ID_DESC = {
        0x0102: "Environment temperature is slightly high. Please check the environment temperature and losing heat measures.",
        0x0104: "The window is dirty, which will influence the reliability of the point cloud. Please clean the window.",
        0x0111: "Abnormal temperature of internal components of the device. Please check the environment temperature and losing heat measures.",
        0x0112: "Abnormal temperature of internal components of the device. Please check the environment temperature and losing heat measures.",
        0x0113: "IMU stopped working. Please try to restart the device to restore.",
        0x0114: "Environment temperature is high. Please check the environment temperature and losing heat measures.",
        0x0115: "Environment temperature beyond the limit, the device has stopped working. Please check the environment temperature and losing heat measures.",
        0x0116: "Abnormal external voltage. Please check the external voltage.",
        0x0117: "Abnormal lidar parameters. Please try to restart the device to restore.",
        0x0201: "Scan module is heating. Please wait for the scan module heating.",
        # 0x0210-0x0219: "Scan module is abnormal, the system is trying to recover. Please wait, if it lasts too long, please try restarting the device to restore.",
        # 0x0210-0x0219: "Scan module is abnormal. Please try to restart the device to restore.",
        0x0401: "Communication link was linked down, now it is recovered. please check the communication link.",
        0x0402: "PTP time synchronization stop or time gap is too big. Please check the PTP time source.",
        0x0403: "The version of PTP is 1588-v2.1, device don't support this version. Please replace 1588-v2.1 version with 1588.2.0 version.",
        0x0404: "PPS time synchronization abnormal. Please check the PPS and GPS signal.",
        0x0405: "There was an exception in time synchronization. Please check the exception reason.",
        0x0406: "Time synchronization accuracy is low. Please check the time source.",
        0x0407: "PPS time synchronization fails because of loss of GPS signal. Please check the GPS signal.",
        0x0408: "PPS time synchronization fails because of loss of PPS signal. Please check the PPS signal.",
        0x0409: "GPS signal is abnormal. Please check the GPS time source.",
        0x040A: "The PTP and GPTP signals exist at the same time. Please check the network topology, use PTP or GPTP alone to synchronize.",
    }

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

        self.action_queue = []
        self.action_sub = self.create_subscription(
            String, "lidar_action", lambda msg: self.action_queue.append(msg), 10)
        self.core_temp_pub = self.create_publisher(
            Float32, 'lidar_core_temp', 10)

    def go(self):
        self.device.set_host_to_this()
        while True:
            for kv in self.device.watch_heartbeat_until(lambda _: not rclpy.ok()):
                dynamic_kv = {key: kv[key] for key in (
                    set(kv.keys()) - set(self.STATIC_KEYS))}
                logger.info("=====HEARTBEAT RECEIVED=====")
                logger.debug(f"dynamic_kv: {dynamic_kv}")
                logger.info("WORK_TGT_MODE: {}",
                            WorkStatus(kv[Key.WORK_TGT_MODE][0]))
                logger.info("CUR_WORK_STATE: {}",
                            WorkStatus(kv[Key.CUR_WORK_STATE][0]))
                if Key.LIDAR_DIAG_STATUS in kv.keys():
                    codes = dynamic_kv[Key.LIDAR_DIAG_STATUS]
                    module_names = ["System", "Scan",
                                    "Ranging", "Communication"]
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
                        if level == 0:
                            continue
                        msg = f"HMS: 0x{abnormal_id:04x}"
                        if abnormal_id in self.HMS_ID_DESC:
                            msg += " " + self.HMS_ID_DESC[abnormal_id]
                        match level:
                            case 0x01:
                                logger.info(msg)
                            case 0x02:
                                logger.warning(msg)
                            case 0x03:
                                logger.error(msg)
                            case 0x04:
                                logger.critical(msg)
                if Key.CORE_TEMP in kv.keys():
                    core_temp = kv[Key.CORE_TEMP][0]/100
                    logger.info(f"CORE_TEMP: {core_temp}℃")
                    msg = Float32()
                    msg.data = core_temp
                    self.core_temp_pub.publish(msg)
                if Key.LOCAL_TIME_NOW in kv.keys():
                    local_time_now = \
                        datetime.fromtimestamp(
                            kv[Key.LOCAL_TIME_NOW][0] / 1000_000_000)
                    last_sync_time = kv[Key.LAST_SYNC_TIME][0]
                    logger.info(
                        f"LOCAL_TIME_NOW: {local_time_now} ({local_time_now - datetime.fromtimestamp(0)})")
                    logger.info(
                        f"TIME_SYNC_TYPE: {kv[Key.TIME_SYNC_TYPE][0]}, LAST_SYNC_TIME: {last_sync_time}, TIME_OFFSET: {kv[Key.TIME_OFFSET][0]/1000_000}ms")
                logger.info("============================")

                rclpy.spin_once(self, timeout_sec=0)
                if len(self.action_queue) != 0:
                    break
            for action in self.action_queue:
                logger.debug(f"Received action: {action}")
                match action.data:
                    case "enable":
                        logger.warning("ENABLING SAMPLING")
                        self.device.set_work_mode(True)
                    case "disable":
                        logger.warning("DISABLING SAMPLING")
                        self.device.set_work_mode(False)
                    case _:
                        logger.warning(f"UNKNOWN ACTION: {action.data}")
            self.action_queue = []


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
