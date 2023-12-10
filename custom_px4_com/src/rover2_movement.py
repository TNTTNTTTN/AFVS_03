#!/usr/bin/env python3

PI = 3.141592
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleStatus
from std_msgs.msg import Bool

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
            OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', qos_profile)
		self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', qos_profile)
		self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/px4_2/fmu/in/vehicle_command', qos_profile)
	
	# Create subscribers (Target)
		self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position', self.target_local_position_callback, qos_profile)
		self.vehicle_global_position_subscriber = self.create_subscription(
			VehicleGlobalPosition, '/px4_1/fmu/out/vehicle_global_position', self.target_global_position_callback, qos_profile)
		
    # Create subscribers (Main)
		self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/px4_2/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
		self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/px4_2/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
		self.multi_disarm_subscriber = self.create_subscription(Bool, '/multi/out', self.multi_disarm_callback, qos_profile)

		self.setup = False
		self.distance = 0.0
		self.rtl = False
		self.offboard_setpoint_counter = 0
		self.vehicle_local_position = VehicleLocalPosition()
		self.target_local_position = VehicleLocalPosition()
		self.vehicle_status = VehicleStatus()
		self.target_global_position = VehicleGlobalPosition()

        # Create a timer to publish control commands
		self.timer = self.create_timer(0.1, self.timer_callback)

	def target_local_position_callback(self, vehicle_local_position):
		"""Callback function for target_local_position topic subscriber."""
		self.target_local_position = vehicle_local_position

	def target_global_position_callback(self, vehicle_global_position):
		self.target_global_position = vehicle_global_position

	def vehicle_local_position_callback(self, vehicle_local_position):
		"""Callback function for vehicle_local_position topic subscriber."""
		self.vehicle_local_position = vehicle_local_position

	def vehicle_status_callback(self, vehicle_status):
		"""Callback function for vehicle_status topic subscriber."""
		self.vehicle_status = vehicle_status

	def multi_disarm_callback(self, disarm_command):
		print(disarm_command.data)
		self.rtl = disarm_command.data

	def arm(self):
		"""Send an arm command to the vehicle."""
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
		self.get_logger().info('Arm command sent')

	def disarm(self):
		"""Send a disarm command to the vehicle."""
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
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

	def set_home(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, param1=0.0, param5=self.target_global_position.lat, 
			       param6=self.target_global_position.lon, param7=self.target_global_position.alt)
		self.get_logger().info("Set Home Complete")

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
		self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

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
		msg.target_system = 3
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
		self.vehicle_command_publisher.publish(msg)

	def timer_callback(self) -> None:
		"""Callback function for the timer."""
		self.publish_offboard_control_heartbeat_signal()
		t_x = self.target_local_position.x
		t_y = self.target_local_position.y
		b_x = self.vehicle_local_position.x
		b_y = self.vehicle_local_position.y

		self.distance = math.sqrt((t_x-b_x)**2 + (t_y-b_y)**2)

		if self.offboard_setpoint_counter == 10:
			self.set_home()
			self.engage_offboard_mode()
			self.arm()

		if ((self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD) &
		(self.distance > 10.0)):
			self.publish_position_setpoint(t_x, t_y, 0.0)
			self.get_logger().info(f"Distance : {self.distance}")

		elif ((self.distance <= 5.0) & (self.rtl)):
			self.disarm()
			raise SystemExit
		
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
