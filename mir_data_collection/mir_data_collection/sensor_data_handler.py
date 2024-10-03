from rclpy.serialization import serialize_message
from rclpy.clock import Clock
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry
from time import time

class SensorDataHandler:
    MSG_TYPE_MAP = {
        JointState: 'sensor_msgs/msg/JointState',
        Odometry: 'nav_msgs/msg/Odometry',
        LaserScan: 'sensor_msgs/msg/LaserScan'
    }
    """Base class for handling sensor data."""
    def __init__(self, topic, msg_type, database_upload_interval_seconds):
        self.topic = topic
        self.msg_type = msg_type
        self.data_buffer = []
        self.clock = Clock()
        
        # Ros bag init
        self.writer = SequentialWriter()
        bag_file_name = f"data/{self.topic.replace('/', '_')}"
        storage_options = StorageOptions(uri=bag_file_name, storage_id='sqlite3')
        converter_options = ConverterOptions()
        self.writer.open(storage_options, converter_options)
        self.topic_info = TopicMetadata(
            name=self.topic,
            type=self.MSG_TYPE_MAP[msg_type],
            serialization_format='cdr'
        )
        self.writer.create_topic(self.topic_info)
        
        self.database_upload_interval_seconds = database_upload_interval_seconds
        self.last_upload_time = time()

    def callback(self, msg):
        """Callback to handle incoming messages."""
        self.data_buffer.append(msg)
        self.try_upload()

    def try_upload(self):
        """Try to upload buffered data to the database."""
        if time() >= self.last_upload_time + self.database_upload_interval_seconds:
            self.last_upload_time = self.database_upload_interval_seconds
            success = self.upload_to_database(self.data_buffer)
            if not success:
                print(f"Upload failed for {self.topic}, saving data to ROS bag.")
                self.save_to_ros_bag(self.data_buffer)
                self.data_buffer.clear()  # Clear the buffer after saving

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
    def __init__(self, database_upload_interval_seconds = 60):
        super().__init__('/joint_states', JointState, database_upload_interval_seconds)

    def callback(self, msg):
        super().callback(msg)
        # print(f"Joint Positions: {msg.position}")

class OdometryHandler(SensorDataHandler):
    def __init__(self, database_upload_interval_seconds = 60):
        super().__init__('/odom', Odometry, database_upload_interval_seconds)

    def callback(self, msg):
        super().callback(msg)
        # print(f"Odom Position: {msg.pose.pose.position}")

class LaserScanHandler(SensorDataHandler):
    def __init__(self, database_upload_interval_seconds = 60):
        super().__init__('/scan', LaserScan, database_upload_interval_seconds)

    def callback(self, msg):
        super().callback(msg)
        # print(f"Laser Scan Ranges: {msg.ranges[:5]}")  # Print first 5 ranges