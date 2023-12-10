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
class SimpleControl(Node):
	def __init__(self) -> None:
		super().__init__('simple_movement')
        
		qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

	# Create publishers
		self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
		self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
		self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
		self.multi_disarm_publisher = self.create_publisher(Bool, '/multi/out', qos_profile)
	
	# Create subscribers
		self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
		self.vehicle_global_position_subscriber = self.create_subscription(
			VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
		self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

	# Variable Setup
		self.setup = False
		self.position = [0,0,0]
		self.rtl = False
		self.offboard_setpoint_counter = 0
		self.vehicle_local_position = VehicleLocalPosition()
		self.vehicle_status = VehicleStatus()
		self.vehicle_global_position = VehicleGlobalPosition()

        # Create a timer to publish control commands
		self.timer = self.create_timer(0.1, self.timer_callback)
		
	def multi_disarm(self, data):
		msg = Bool()
		msg.data = data
		self.multi_disarm_publisher.publish(msg)

	def vehicle_local_position_callback(self, vehicle_local_position):
		"""Callback function for vehicle_local_position topic subscriber."""
		self.vehicle_local_position = vehicle_local_position
		x = self.vehicle_local_position.x - self.position[0]
		y = self.vehicle_local_position.y - self.position[1]

		# self.get_logger().info(f"Current Velocity [{self.vehicle_local_position.vx :.2f}, {self.vehicle_local_position.vy:.2f}]")
		
		# self.get_logger().info(f"Current position {[x, y]}")
		if (x**2 + y**2 < 4.0) & (self.setup==True):
			if (self.rtl):
				self.disarm()
				raise SystemExit
			self.setup = False

	def vehicle_global_position_callback(self, vehicle_global_position):
		self.vehicle_global_position = vehicle_global_position
		
	def vehicle_status_callback(self, vehicle_status):
		"""Callback function for vehicle_status topic subscriber."""
		self.vehicle_status = vehicle_status

	def arm(self):
		"""Send an arm command to the vehicle."""
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
		self.multi_disarm(False)
		self.get_logger().info('Arm command sent')

	def disarm(self):
		"""Send a disarm command to the vehicle."""
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
		self.multi_disarm(True)
		self.get_logger().info('Disarm command sent')

	def engage_offboard_mode(self):
		"""Switch to offboard mode."""
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
		self.get_logger().info("Switching to offboard mode")

	def land(self):
		"""Switch to land mode."""
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
		self.get_logger().info("Switching to land mode")

	def publish_offboard_control_heartbeat_signal(self):
		"""Publish the offboard control mode."""
		msg = OffboardControlMode()
		msg.position = True
		msg.velocity = False
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
		self.offboard_control_mode_publisher.publish(msg)

	def publish_position_setpoint(self, x: float, y: float, z: float):
		"""Publish the trajectory setpoint."""
		msg = TrajectorySetpoint()
		msg.position = [x, y, z]
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
		self.trajectory_setpoint_publisher.publish(msg)
		# self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

	def publish_vehicle_command(self, command, **params) -> None:
		"""Publish a vehicle command."""
		msg = VehicleCommand()
		msg.command = command
		msg.param1 = params.get("param1", 0.0)
		msg.param2 = params.get("param2", 0.0)
		msg.param3 = params.get("param3", 0.0)
		msg.param4 = params.get("param4", 0.0)
		msg.param5 = params.get("param5", 0.0)
		msg.param6 = params.get("param6", 0.0)
		msg.param7 = params.get("param7", 0.0)
		msg.target_system = 1
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
		self.vehicle_command_publisher.publish(msg)

	def timer_callback(self) -> None:
		"""Callback function for the timer."""
		self.publish_offboard_control_heartbeat_signal()
		# print(f"{self.position[0], self.position[1], self.setup}")
		if (not self.setup):
			print("PX4 Rover Position Control")
			print("------------------------------------------------------")
			print("Input desired position based on NED frame")
			print("------------------------------------------------------")
			self.position[0], self.position[1] = map(float, input().split())
			self.setup = True
			if (self.position == [0, 0, 0]):
				self.get_logger().info(f"Return to Home")
				self.rtl = True
			else :
				self.get_logger().info(f"Target Position : {self.position[0], self.position[1]}")

		else: 
			if self.offboard_setpoint_counter == 10:
				self.arm()
				self.engage_offboard_mode()
				
			if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
				self.publish_position_setpoint(self.position[0], self.position[1], 0.0)

			if self.offboard_setpoint_counter < 11:
				self.offboard_setpoint_counter += 1

def main(args=None) -> None:
	print('Starting offboard control node...')
	rclpy.init(args=args)
	offboard_control = SimpleControl()
	try:
		rclpy.spin(offboard_control)
	except SystemExit:
		offboard_control.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print(e)
