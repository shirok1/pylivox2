import argparse
from socket import inet_ntoa

from loguru import logger

from pylivox2.device import Device, WorkStatus
from pylivox2.kv import Key

parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("-d", "--discover", action="store_true", help="discover devices")
group.add_argument("--ip", nargs='?', const="192.168.1.100", help="IP address of the device")
parser.add_argument("action", choices=["version", "get_mode", "enable", "disable", "watch"],
                    help="action to perform")
args = parser.parse_args()

logger.info("Args: {}", args)

if args.discover:
    device = Device.discover_one()
else:
    device = Device(10, "", "192.168.1.100", 56000)
match args.action:
    case "version":
        ret = device.get_parameters([Key.VERSION_APP, Key.VERSION_LOADER, Key.VERSION_HARDWARE, Key.LIDAR_IPCFG])
        logger.info("App Version: {}.{}.{}.{}", *ret[Key.VERSION_APP])
        logger.info("Loader Version: {}.{}.{}.{}", *ret[Key.VERSION_LOADER])
        logger.info("Hardware Version: {}.{}.{}.{}", *ret[Key.VERSION_HARDWARE])
        logger.info("Lidar IP: {}", inet_ntoa(ret[Key.LIDAR_IPCFG][0]))
        logger.info("Lidar Netmask: {}", inet_ntoa(ret[Key.LIDAR_IPCFG][1]))
        logger.info("Lidar Gateway: {}", inet_ntoa(ret[Key.LIDAR_IPCFG][2]))
    case "get_mode":
        work_state = device.get_work_mode()
        logger.info("{}", work_state)
    case "enable":
        work_state = device.get_work_mode()
        logger.info("{}", work_state)
        device.set_host_to_this()
        device.set_work_mode(True)
        for _, work_state, _ in device.watch_heartbeat_until(
                lambda _, state, __: state == WorkStatus.SAMPLING):
            logger.info("work_state: {}", work_state)
    case "disable":
        work_state = device.get_work_mode()
        logger.info("{}", work_state)
        device.set_work_mode(False)
        for _, work_state, _ in device.watch_heartbeat_until(
                lambda _, state, __: state == WorkStatus.IDLE):
            logger.info("work_state: {}", work_state)
    case "watch":
        for diag_status, work_state, flash_status in device.watch_heartbeat_until(
                lambda _, __, ___: False):
            system_status, scan_status, ranging_status, communication_status = diag_status
            logger.info("system_status: {}, scan_status: {}, ranging_status: {}, communication_status: {}",
                        system_status, scan_status, ranging_status, communication_status)
            logger.info("work_state: {}, flash_status: {}", work_state, flash_status)
