import logging

from loguru import logger
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node


class NodeLoguruSink(logging.Handler):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node

    def emit(self, record: logging.LogRecord) -> None:
        # self.node.get_logger().log(record.getMessage(), LoggingSeverity(record.levelno))
        match record.levelno:
            case logging.FATAL:
                self.node.get_logger().fatal(record.getMessage())
            case logging.ERROR:
                self.node.get_logger().error(record.getMessage())
            case logging.WARNING:
                self.node.get_logger().warning(record.getMessage())
            case logging.INFO:
                self.node.get_logger().info(record.getMessage())
            case logging.DEBUG:
                self.node.get_logger().debug(record.getMessage())
            case logging.NOTSET:
                self.node.get_logger().log(record.getMessage(), LoggingSeverity.UNSET)

    def set_as_only_logger(self):
        logger.remove()
        logger.add(self,
                   format="<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> <level>{message}</level>")
