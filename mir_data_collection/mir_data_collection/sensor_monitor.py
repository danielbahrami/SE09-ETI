import rclpy
import os
import shutil
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from . import sensor_data_handler

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # Create instances of each data handler
        self.joint_state_handler = sensor_data_handler.JointStateHandler(database_upload_interval_seconds=60)
        self.odom_handler = sensor_data_handler.OdometryHandler(database_upload_interval_seconds=60)
        self.laser_scan_handler = sensor_data_handler.LaserScanHandler(database_upload_interval_seconds=60)

        # Create subscriptions for each sensor topic
        self.create_subscription(sensor_data_handler.JointState, '/joint_states', self.joint_state_handler.callback, 10)
        self.create_subscription(sensor_data_handler.Odometry, '/odom', self.odom_handler.callback, 10)
        self.create_subscription(sensor_data_handler.LaserScan, '/scan', self.laser_scan_handler.callback, QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT # RELIABLE - TCP like, BEST_EFFORT - UDP like
        ))

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
