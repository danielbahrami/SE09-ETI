from . import sensor_data_handler
from sensor_msgs.msg import Imu

"""
published every 2ms

Orientation: Represents the orientation of the IMU in 3D space, using quaternion representation.
Orientation Covariance: Represents the uncertainty in the orientation measurements.
Angular Velocity: Represents the rate of rotation around the X, Y, and Z axes.
Angular Velocity Covariance: Uncertainty in angular velocity measurements.
Linear Acceleration: Represents the linear acceleration along the X, Y, and Z axes.
Linear Acceleration Covariance: Uncertainty in linear acceleration measurements.

Topic: /imu_data/data, Timestamp: 415686000000, Data: sensor_msgs.msg.Imu(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=415, nanosec=686000000), frame_id='base_footprint'), orientation=geometry_msgs.msg.Quaternion(x=2.6813595906450587e-05, y=1.302859511782345e-05, z=-0.1556178558441262, w=-0.9878173323311167), orientation_covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0.]), angular_velocity=geometry_msgs.msg.Vector3(x=-0.00027582991380818635, y=0.00021087862711775693, z=0.00018145953222759817), angular_velocity_covariance=array([2.89e-08, 0.00e+00, 0.00e+00, 0.00e+00, 2.89e-08, 0.00e+00,
       0.00e+00, 0.00e+00, 2.89e-08]), linear_acceleration=geometry_msgs.msg.Vector3(x=-0.10190823064012598, y=0.07896371959584128, z=9.898074797454012), linear_acceleration_covariance=array([2.89e-08, 0.00e+00, 0.00e+00, 0.00e+00, 2.89e-08, 0.00e+00,
       0.00e+00, 0.00e+00, 2.89e-08]))
       
the ros2 topic echo returns a more human readable format instead

header:
  stamp:
    sec: 191
    nanosec: 530000000
  frame_id: base_footprint
orientation:
  x: 3.751916292166651e-06
  y: 4.6634879057317e-06
  z: -0.13250873705380983
  w: -0.9911818372874774
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.0003892143432612784
  y: -8.91420795280089e-05
  z: 0.0004468924623865408
angular_velocity_covariance:
- 2.8900000000000004e-08
- 0.0
- 0.0
- 0.0
- 2.8900000000000004e-08
- 0.0
- 0.0
- 0.0
- 2.8900000000000004e-08
linear_acceleration:
  x: -0.1024890261996328
  y: 0.11004959461524721
  z: 9.914164453643247
linear_acceleration_covariance:
- 2.8900000000000004e-08
- 0.0
- 0.0
- 0.0
- 2.8900000000000004e-08
- 0.0
- 0.0
- 0.0
- 2.8900000000000004e-08

"""

class IMUHandler(sensor_data_handler.SensorDataHandler):
    def __init__(self, database_upload_interval_seconds, bag_file_split_duration):
        super().__init__('/imu_data/data', Imu, database_upload_interval_seconds, bag_file_split_duration)

    def callback(self, msg):
        super().callback(msg)