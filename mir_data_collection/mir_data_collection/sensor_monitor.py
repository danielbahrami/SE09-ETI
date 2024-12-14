import os
import shutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from . import IMUHandler
from . import JointStateHandler
from . import LaserScanHandler
from . import OdometryHandler


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')

        # Create instances of each data handler
        self.joint_state_handler = JointStateHandler.JointStateHandler(database_upload_interval_seconds=10,
                                                                       bag_file_split_duration=10)
        self.odom_handler = OdometryHandler.OdometryHandler(database_upload_interval_seconds=10,
                                                            bag_file_split_duration=10)
        self.laser_scan_handler = LaserScanHandler.LaserScanHandler(database_upload_interval_seconds=10,
                                                                    bag_file_split_duration=10)
        self.imu_handler = IMUHandler.IMUHandler(database_upload_interval_seconds=10, bag_file_split_duration=10)

        # Create subscriptions for each sensor topic
        self.create_subscription(JointStateHandler.JointState, '/joint_states', self.joint_state_handler.callback, 10)
        self.create_subscription(OdometryHandler.Odometry, '/odom', self.odom_handler.callback, 10)
        self.create_subscription(LaserScanHandler.LaserScan, '/scan', self.laser_scan_handler.callback, QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # RELIABLE - TCP like, BEST_EFFORT - UDP like
        ))
        self.create_subscription(IMUHandler.Imu, '/imu_data/data', self.imu_handler.callback, 10)


def clear_data_dir():
    if not os.path.exists("data/"):
        os.makedirs("data/")

    for entry in os.listdir("data/"):
        entry_path = os.path.join("data/", entry)
        # Check if it's a file or directory and delete it
        if os.path.isfile(entry_path):
            os.remove(entry_path)
        elif os.path.isdir(entry_path):
            shutil.rmtree(entry_path)


def main(args=None):
    clear_data_dir()

    rclpy.init(args=args)
    sensor_monitor = SensorMonitor()
    rclpy.spin(sensor_monitor)

    sensor_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
