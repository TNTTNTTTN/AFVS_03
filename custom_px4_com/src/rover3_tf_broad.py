#!/usr/bin/env python3

PI = 3.141592
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleAttitude, VehicleOdometry
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
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

	# # Time Synchronizer & Broadcaster
	# 	self.sub1 = Subscriber(self, VehicleLocalPosition, "/fmu/out/vehicle_local_position", qos_profile=qos_profile)
	# 	self.sub2 = Subscriber(self, VehicleAttitude, "/fmu/out/vehicle_attitude", qos_profile=qos_profile)
	# 	sync_ = ApproximateTimeSynchronizer([self.sub1, self.sub2], 1, 1, allow_headerless=True)
	# 	sync_.registerCallback(self.vehicle_tf2_broadcaster)

	# 	self.tf_broadcaster1 = TransformBroadcaster(self)
	# 	# self.tf_broadcaster2 = TransformBroadcaster(self)

	# Time Synchronizer & Broadcaster
		self.vehicle_odom_publisher = self.create_publisher(Odometry, '/odom', qos_profile)
		self.sub1 = Subscriber(self, VehicleLocalPosition, "/px4_3/fmu/out/vehicle_local_position", qos_profile=qos_profile)
		self.sub2 = Subscriber(self, VehicleOdometry, "/px4_3/fmu/out/vehicle_odometry", qos_profile=qos_profile)
		sync_ = ApproximateTimeSynchronizer([self.sub1, self.sub2], 1, 1, allow_headerless=True)
		sync_.registerCallback(self.vehicle_tf2_broadcaster)
		# self.vehicle_init_goal_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/px4_1/initialpose', self.vehicle_init_goal_callback, qos_profile)
		self.tf_broadcaster1 = TransformBroadcaster(self)
		self.odom = VehicleOdometry()

	def vehicle_tf2_broadcaster(self, msg1: VehicleLocalPosition, msg2: VehicleOdometry):

		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = '/odom'
		t.child_frame_id = '/base_footprint'
		t.transform.translation.x = float(msg1.x)
		t.transform.translation.y = float(-msg1.y)
		t.transform.translation.z = 0.0
		t.transform.rotation.w = float(msg2.q[0])
		t.transform.rotation.x = float(msg2.q[1])
		t.transform.rotation.y = float(msg2.q[2])
		t.transform.rotation.z = float(msg2.q[3])

		f = Odometry()
		f.header.stamp = self.get_clock().now().to_msg()
		f.header.frame_id = '/odom'
		f.child_frame_id = '/base_footprint'
		f.pose.pose.position.x = float(msg1.x)
		f.pose.pose.position.y = float(-msg1.y)
		f.pose.pose.position.z = 0.0
		f.pose.pose.orientation.w = float(msg2.q[0])
		f.pose.pose.orientation.x = float(msg2.q[1])
		f.pose.pose.orientation.y = float(msg2.q[2])
		f.pose.pose.orientation.z = float(msg2.q[3])
		f.twist.twist.linear.x = float(msg1.vx)
		f.twist.twist.angular.z = float(msg2.angular_velocity[2])

		i = TransformStamped()
		i.header.stamp = self.get_clock().now().to_msg()
		i.header.frame_id = '/base_footprint'
		i.child_frame_id = '/base_link'
		i.transform.translation.x = 0.0
		i.transform.translation.y = 0.0
		i.transform.translation.z = float(-msg1.z)
		i.transform.rotation.w = 1.0
		i.transform.rotation.x = 0.0
		i.transform.rotation.y = 0.0
		i.transform.rotation.z = 0.0
		
		# j = TransformStamped()
		# j.header.stamp = self.get_clock().now().to_msg()
		# j.header.frame_id = '/map'
		# j.child_frame_id = '/base_link'
		# j.transform.translation.x = float(msg.position[0])
		# j.transform.translation.y = float(-msg.position[1])
		# j.transform.translation.z = float(-msg.position[2])
		# j.transform.rotation.w = float(msg.q[0])
		# j.transform.rotation.x = float(msg.q[1])
		# j.transform.rotation.y = float(msg.q[2])
		# j.transform.rotation.z = float(msg.q[3])

		# k = TransformStamped()
		# k.header.stamp = self.get_clock().now().to_msg()
		# k.header.frame_id = '/map'
		# k.child_frame_id = '/base_footprint'
		# k.transform.translation.x = float(msg.position[0])
		# k.transform.translation.y = float(-msg.position[1])
		# k.transform.translation.z = 0.0
		# k.transform.rotation.w = float(msg.q[0])
		# k.transform.rotation.x = float(msg.q[1])
		# k.transform.rotation.y = float(msg.q[2])
		# k.transform.rotation.z = float(msg.q[3])

		# self.px4_odom_publisher.publish(msg)

		self.vehicle_odom_publisher.publish(f)
		# self.tf_broadcaster1.sendTransform(j)
		# self.tf_broadcaster1.sendTransform(k)
		self.tf_broadcaster1.sendTransform(t)
		self.tf_broadcaster1.sendTransform(i)
		self.get_logger().info('Transform sent')
	
	def vehicle_init_goal_callback(self, msg: PoseWithCovarianceStamped):
		f = Odometry()
		f.header.stamp = self.get_clock().now().to_msg()
		f.header.frame_id = '/odom'
		f.child_frame_id = '/base_footprint'
		f.pose.pose.position = msg.pose.pose.position
		f.pose.covariance = msg.pose.covariance
		f.pose.pose.orientation = msg.pose.pose.orientation
		self.vehicle_odom_publisher.publish(f)

		p = self.odom
		p.timestamp = int(self.get_clock().now().nanoseconds / 1000)
		p.position[0] = msg.pose.pose.position.x
		p.position[1] = msg.pose.pose.position.y
		p.position[2] = msg.pose.pose.position.z
		p.q[0] = msg.pose.pose.orientation.w
		p.q[1] = msg.pose.pose.orientation.x
		p.q[2] = msg.pose.pose.orientation.y
		p.q[3] = msg.pose.pose.orientation.z
		# self.px4_odom_publisher.publish(p)

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
