import argparse

from loguru import logger

from pylivox2.device import Device
from pylivox2.kv import Key

parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("-d", "--discover", action="store_true", help="discover devices")
group.add_argument("--ip", nargs='?', const="192.168.1.100", help="IP address of the device")
parser.add_argument("action", choices=["version", "get_mode", "enable", "disable"], help="action to perform")
args = parser.parse_args()

logger.info("Args: {}", args)

if args.discover:
    device = Device.discover_one()
else:
    device = Device(10, "", "192.168.1.100", 56000)
match args.action:
    case "version":
        device.get_parameters([Key.VERSION_APP, Key.VERSION_LOADER, Key.VERSION_HARDWARE, Key.LIDAR_IPCFG])
    case "get_mode":
        device.get_parameters([Key.WORK_TGT_MODE, Key.POINT_SEND_EN])
    case "enable":
        device.get_parameters([Key.WORK_TGT_MODE, Key.POINT_SEND_EN])
        device.set_host_to_this()
        device.set_work_mode(True)
        device.get_parameters([Key.WORK_TGT_MODE, Key.POINT_SEND_EN])
    case "disable":
        device.get_parameters([Key.WORK_TGT_MODE, Key.POINT_SEND_EN])
        device.set_work_mode(False)
        device.get_parameters([Key.WORK_TGT_MODE, Key.POINT_SEND_EN])
