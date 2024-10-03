import rclpy
import os
import shutil
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.clock import Clock
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

DATA_DIR = "data/"

class SensorDataHandler:
    MSG_TYPE_MAP = {
        JointState: 'sensor_msgs/msg/JointState',
        Odometry: 'nav_msgs/msg/Odometry',
        LaserScan: 'sensor_msgs/msg/LaserScan'
    }
    """Base class for handling sensor data."""
    def __init__(self, topic, msg_type, batch_size):
        self.topic = topic
        self.msg_type = msg_type
        self.buffer = []
        self.batch_size = batch_size
        self.clock = Clock()
        self.writer = SequentialWriter()
        self.open_new_bag()
        self.topic_info = TopicMetadata(
            name=self.topic,
            type=self.MSG_TYPE_MAP[msg_type],
            serialization_format='cdr'
        )
        self.writer.create_topic(self.topic_info)
        
    def open_new_bag(self):
        bag_file_name = f"{DATA_DIR}{self.topic.replace('/', '_')}"
        storage_options = StorageOptions(uri=bag_file_name, storage_id='sqlite3')
        converter_options = ConverterOptions()
        self.writer.open(storage_options, converter_options)

    def callback(self, msg):
        """Callback to handle incoming messages."""
        self.buffer.append(msg)
        self.try_upload()

    def try_upload(self):
        """Try to upload buffered data to the database."""
        if len(self.buffer) >= self.batch_size:
            success = self.upload_to_database(self.buffer)
            if not success:
                print(f"Upload failed for {self.topic}, saving data to ROS bag.")
                self.save_to_ros_bag(self.buffer)
                self.buffer.clear()  # Clear the buffer after saving

    def upload_to_database(self, data):
        """Simulate upload logic; returns True if successful, False if failed."""
        # Implement your actual upload logic here
        return False  # Simulating a failure for demonstration

    def save_to_ros_bag(self, data):
        """Save buffered data to a ROS bag."""
        for msg in data:
            serialized_msg = serialize_message(msg)
            self.writer.write(self.topic, serialized_msg, self.clock.now().nanoseconds)

class JointStateHandler(SensorDataHandler):
    def __init__(self, batch_size):
        super().__init__('/joint_states', JointState, batch_size)

    def callback(self, msg):
        super().callback(msg)
        # print(f"Joint Names: {msg.name}")
        print(f"Joint Positions: {msg.position}")

class OdometryHandler(SensorDataHandler):
    def __init__(self, batch_size):
        super().__init__('/odom', Odometry, batch_size)

    def callback(self, msg):
        super().callback(msg)
        print(f"Odom Position: {msg.pose.pose.position}")

class LaserScanHandler(SensorDataHandler):
    def __init__(self, batch_size):
        super().__init__('/scan', LaserScan, batch_size)

    def callback(self, msg):
        super().callback(msg)
        print(f"Laser Scan Ranges: {msg.ranges[:5]}")  # Print first 5 ranges

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # Create instances of each data handler
        self.joint_state_handler = JointStateHandler(batch_size=10)
        self.odom_handler = OdometryHandler(batch_size=1000)
        self.laser_scan_handler = LaserScanHandler(batch_size=10)

        # Create subscriptions for each sensor topic
        self.create_subscription(JointState, '/joint_states', self.joint_state_handler.callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_handler.callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_scan_handler.callback, QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT # RELIABLE - TCP like, BEST_EFFORT - UDP like
        ))

def clear_data_dir():
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)

    for entry in os.listdir(DATA_DIR):
        entry_path = os.path.join(DATA_DIR, entry)
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
