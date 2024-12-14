from nav_msgs.msg import Odometry

from . import sensor_data_handler

"""
Stored every 1ms. Provides essential details about the robot's position and motion over time based on wheel encoders or other motion sensors. Can experience drift over time. We can probably store not all messages?
 
pose: Represents the position and orientation of the robot.
covariance: A 6x6 matrix representing the uncertainty in the robot's pose (position and orientation).
twist: Describes the robot's linear and angular velocities.
covariance: Similar to the pose, it indicates the uncertainty in the velocity estimates, with small values suggesting low uncertainty.

Topic: /odom, Timestamp: 3566886000000, Data: nav_msgs.msg.Odometry(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=3566, nanosec=886000000), frame_id='odom'), child_frame_id='base_footprint', pose=geometry_msgs.msg.PoseWithCovariance(pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.06583785078628177, y=-0.019726396725516838, z=-4.507029490868897e-06), orientation=geometry_msgs.msg.Quaternion(x=8.426933611074894e-07, y=-8.735483530827466e-07, z=-0.03750153216395431, w=0.999296570135154)), covariance=array([1.e-05, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e-05,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 1.e-03])), twist=geometry_msgs.msg.TwistWithCovariance(twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.00010590935019000677, y=2.4570170366107435e-05, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=3.303811267387432e-05)), covariance=array([1.e-05, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e-05,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 1.e-03])))
"""


class OdometryHandler(sensor_data_handler.SensorDataHandler):
    def __init__(self, database_upload_interval_seconds, bag_file_split_duration):
        super().__init__('/odom', Odometry, database_upload_interval_seconds, bag_file_split_duration)

    def callback(self, msg):
        super().callback(msg)
