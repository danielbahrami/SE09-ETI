from . import sensor_data_handler
from sensor_msgs.msg import JointState

"""
Performed every 10ms. Contains 9 messages for each different link, it's position and velocity. Example data obtained from read_bag.py

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['fl_caster_rotation_joint'], position=[-0.5440394531693746], velocity=[-0.005163600494526478], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['fl_caster_wheel_joint'], position=[-1.1188976719371162], velocity=[0.0004919319121814841], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['fr_caster_rotation_joint'], position=[-0.581832579460662], velocity=[-0.005624149718354879], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['fr_caster_wheel_joint'], position=[-1.139902999851449], velocity=[-0.0016130537600815978], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['bl_caster_rotation_joint'], position=[-0.666050478884765], velocity=[-0.004295369182070469], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['bl_caster_wheel_joint'], position=[-1.1034222787268408], velocity=[0.0009656503472965229], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['br_caster_rotation_joint'], position=[-0.7141542996451822], velocity=[-0.004894490637779653], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['br_caster_wheel_joint'], position=[-1.120855707107295], velocity=[-0.0010316835508631752], effort=[]) 

Topic: /joint_states, Timestamp: 474088000000, Data: sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=474, nanosec=88000000), frame_id=''), name=['right_wheel_joint', 'left_wheel_joint'], position=[-1.0687704019417783, -1.0308859769340728], velocity=[-0.0028012147334669207, -0.0024552901957832335], effort=[])
"""

class JointStateHandler(sensor_data_handler.SensorDataHandler):
    def __init__(self, database_upload_interval_seconds, bag_file_split_duration):
        super().__init__('/joint_states', JointState, database_upload_interval_seconds, bag_file_split_duration)

    def callback(self, msg):
        super().callback(msg)