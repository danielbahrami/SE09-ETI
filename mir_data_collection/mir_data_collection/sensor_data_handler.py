import re
from time import time

from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message, deserialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState, LaserScan

global_time = time()


class SensorDataHandler:
    MSG_TYPE_MAP = {
        JointState: 'sensor_msgs/msg/JointState',
        Odometry: 'nav_msgs/msg/Odometry',
        LaserScan: 'sensor_msgs/msg/LaserScan',
        Imu: 'sensor_msgs/msg/Imu'
    }
    """Base class for handling sensor data."""

    def __init__(self, topic, msg_type, database_upload_interval_seconds, bag_file_split_duration):
        self.topic = topic
        self.msg_type = msg_type
        self.database_upload_interval_seconds = database_upload_interval_seconds
        self.last_upload_time = time()

        # Ros bag init
        self.writer = SequentialWriter()
        self.bag_file_path = f"data/{self.topic.replace('/', '_')[1:]}"

        # switch between sql and mcap easily
        storage_type_is_sql = False

        storage = 'sqlite3' if storage_type_is_sql else 'mcap'
        self.bag_extension = ".db3" if storage_type_is_sql else ".mcap"
        self.storage_options = StorageOptions(uri=self.bag_file_path, storage_id=storage,
                                              max_bagfile_duration=bag_file_split_duration)

        self.converter_options = ConverterOptions()
        self.writer.open(self.storage_options, self.converter_options)
        self.topic_info = TopicMetadata(
            name=self.topic,
            type=self.MSG_TYPE_MAP[msg_type],
            serialization_format='cdr'
        )
        self.writer.create_topic(self.topic_info)

    def callback(self, msg):
        """Callback to handle incoming messages."""

        """
        I'm writing to the mir_data_collection bag directly, might be slower than keeping messages 
        in memory, but I give up on buffer approach. You are welcom to give it a shot.
        """
        # self.data_buffer.append(msg)
        self.save_msg_to_ros_bag(msg)

        """
        We try to upload only if we receive a message, 
        even if specified time in database_upload_interval_seconds has passed.
        Should probably change this later.
        """
        self.try_upload()

    def try_upload(self):
        """Try to upload buffered data to the database."""
        if self.should_try_upload():
            files_to_upload = self.get_files_ready_to_upload()
            if len(files_to_upload) < 1:
                return

            success = self.upload_files_to_database(files_to_upload)

    def should_try_upload(self):
        if time() >= self.last_upload_time + self.database_upload_interval_seconds:
            self.last_upload_time += self.database_upload_interval_seconds
            return True

        return False

    def upload_files_to_database(self, file_paths_arr):
        """Simulate upload logic; returns True if successful, False if failed."""
        for file in file_paths_arr:
            success = self.upload_file_to_database(file)
            if not success:
                return False

            os.remove(file)

        return True

    def upload_file_to_database(self, file):
        """Upload a single file to database; returns True if successful, False if failed."""
        upload_file_to_db(self.topic, file)
        # return False

    def save_buffer_to_ros_bag(self, data):
        for msg in data:
            self.save_msg_to_ros_bag(msg)

    def save_msg_to_ros_bag(self, msg):
        serialized_msg = serialize_message(msg)
        timestamp = int(time() * 1_000_000_000) + msg.header.stamp.nanosec
        self.writer.write(self.topic, serialized_msg, timestamp)

    def deserialize_msg(self, msg):
        """Deserialize message from binary data."""
        return deserialize_message(msg, self.msg_type)

    def get_files_ready_to_upload(self):
        """Get available bag files except last one"""

        def sort_by_bag_index(filename):
            match = re.search(r'_(\d+)' + re.escape(self.bag_extension) + r'$', filename)
            return int(match.group(1)) if match else float('inf')

        files = os.listdir(self.bag_file_path)
        filtered_files = [os.path.join(self.bag_file_path, file) for file in files if file.endswith(self.bag_extension)]
        sorted_files_by_bag_index = sorted(filtered_files, key=sort_by_bag_index)

        if len(sorted_files_by_bag_index) > 0:
            sorted_files_by_bag_index.pop()

        return sorted_files_by_bag_index


import requests
import pwd
import os
from datetime import datetime, timezone
import logging
import json
import base64

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


def upload_file_to_db(topic_name, file):
    """
    Uploads a file to a database via HTTP POST request.

    :param topic_name: The topic name for the file.
    :param file: File path (str) or file content (bytes).
    :return: True if upload succeeds, False otherwise.
    """
    try:
        # Construct the API URL
        url = f"http://{os.getenv('BACKEND_URL')}:8080/robot/:{pwd.getpwuid(os.getuid())[0]}"

        # HTTP headers
        headers = {'Content-Type': 'application/json'}

        # Read file content
        if isinstance(file, str):  # File path provided
            if not os.path.exists(file):
                logging.error(f"File not found: {file}")
                return False

            with open(file, 'rb') as f:
                file_data = f.read()
        elif isinstance(file, (bytes, bytearray)):  # File content provided
            file_data = file
        else:
            logging.error("Invalid file type. Must be a file path (str) or file content (bytes).")
            return False

        # Prepare payload
        payload = {
            'data': base64.b64encode(file_data).decode('utf-8'),
            'Robot_user': pwd.getpwuid(os.getuid())[0],
            'Date': datetime.now(timezone.utc).isoformat(),
            'topic': topic_name
        }

        # Make the POST request
        logging.info(f"Sending POST request to {url} with topic '{topic_name}'")
        response = requests.post(url=url, data=json.dumps(payload), headers=headers)

        # Handle response
        if response.status_code == 200:
            logging.info("File uploaded successfully.")
            return True
        else:
            logging.error(f"Failed to upload file. Status Code: {response.status_code}, Response: {response.text}")
            return False

    except requests.exceptions.RequestException as e:
        logging.error(f"HTTP request failed: {e}")
        return False
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        return False
