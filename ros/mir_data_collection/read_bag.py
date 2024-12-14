from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Path to your bag file

# bag_file_path = 'data/_joint_states/_joint_states_0.mcap'
# msg_type = JointState

# bag_file_path = 'data/_scan/_scan_0.mcap'
# msg_type = LaserScan

# bag_file_path = 'data/_odom/_odom_0.mcap'
# msg_type = Odometry

bag_file_path = 'data/imu_data_data/imu_data_data_0.mcap'
msg_type = Imu

# Create a reader
reader = SequentialReader()
storage_options = StorageOptions(uri=bag_file_path, storage_id='mcap')
converter_options = ConverterOptions()

# Open the bag file
reader.open(storage_options, converter_options)

# Read and print messages
while reader.has_next():
    (topic, data, timestamp) = reader.read_next()
    print(f"Topic: {topic}, Timestamp: {timestamp}, Data: {deserialize_message(data, msg_type)} \n")
