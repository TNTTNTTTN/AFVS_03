#!/usr/bin/env python3

PI = 3.141592
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleStatus, VehicleAttitude
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from message_filters import Subscriber, ApproximateTimeSynchronizer

class TFBroadcaster(Node):
	def __init__(self) -> None:
		super().__init__('TFBroadcaster')
        
		qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

	# Time Synchronizer & Broadcaster
		self.sub1 = Subscriber(self, VehicleLocalPosition, "/fmu/out/vehicle_local_position", qos_profile=qos_profile)
		self.sub2 = Subscriber(self, VehicleAttitude, "/fmu/out/vehicle_attitude", qos_profile=qos_profile)
		sync_ = ApproximateTimeSynchronizer([self.sub1, self.sub2], 1, 1, allow_headerless=True)
		sync_.registerCallback(self.vehicle_tf2_broadcaster)

		self.tf_broadcaster1 = TransformBroadcaster(self)
		# self.tf_broadcaster2 = TransformBroadcaster(self)

	def vehicle_tf2_broadcaster(self, msg1: VehicleLocalPosition, msg2: VehicleAttitude):
		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = '/map'
		t.child_frame_id = '/odom'
		t.transform.translation.x = msg1.x
		t.transform.translation.y = -msg1.y
		t.transform.translation.z = -msg1.z
		t.transform.rotation.w = float(msg2.q[0])
		t.transform.rotation.x = float(msg2.q[1])
		t.transform.rotation.y = float(msg2.q[2])
		t.transform.rotation.z = float(msg2.q[3])

		f = TransformStamped()
		f.header.stamp = self.get_clock().now().to_msg()
		f.header.frame_id = '/odom'
		f.child_frame_id = '/base_link'
		f.transform.translation.x = 0.0
		f.transform.translation.y = 0.0
		f.transform.translation.z = 0.0
		f.transform.rotation.w = 1.0
		f.transform.rotation.x = 0.0
		f.transform.rotation.y = 0.0
		f.transform.rotation.z = 0.0
		
		self.tf_broadcaster1.sendTransform(t)
		self.tf_broadcaster1.sendTransform(f)
		self.get_logger().info('Transform sent')

def main(args=None) -> None:
	print('broadcasting topics')
	rclpy.init(args=args)
	tfbroadcaster = TFBroadcaster()
	try:
		rclpy.spin(tfbroadcaster)
	except SystemExit:
		tfbroadcaster.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print(e)
