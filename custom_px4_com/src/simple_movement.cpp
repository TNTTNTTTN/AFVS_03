#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <functional>

#include "math.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using matrix::Vector2f;
using matrix::isEqual;

using std::placeholders::_1;

class SimpleControl : public rclcpp::Node{
    public : 
        SimpleControl() : Node("simple_movement"){
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
            offboard_setpoint_counter_ = 0;

            local_position_sub = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, 
            std::bind(&SimpleControl::subscription_position, this, _1));
            vehicle_status_sub = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos,
            std::bind(&SimpleControl::subscription_status, this, _1));

            offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

            auto command = [this]() -> void {
                if (!setup){
                std::cout<<"PX4 Rover Position Control \n";
                std::cout<<"------------------------------------------------------\n";
                std::cout<<"Input desired position based on NED frame\n";
                std::cout<<"------------------------------------------------------\n";
                std::cin>>input1>>input2;
                Target = Vector2f(input1, input2);

                    if (isEqual(Target, Vector2f(0.0f,0.0f))){
                        RCLCPP_INFO(this->get_logger(), "Returning home");
                        end = true;
                    }

                    else{
                        RCLCPP_INFO(this->get_logger(), "Target position : %.3f %.3f", Target(0), Target(1));
                    }
                setup = true;
                }

                else {
                    if (offboard_setpoint_counter_ == 1) {
                        // Change to Offboard mode after 1 setpoints
                        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                        // Arm the vehicle
                        this->arm();
                        is_armed_send = true;
                    }
                    if (!is_armed){
                        RCLCPP_INFO(this->get_logger(), "Waiting for Arming");
                        rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
                    }
                    // offboard_control_mode needs to be paired with trajectory_setpoint
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(Target);

                    // stop the counter after reaching 11
                    if (offboard_setpoint_counter_ < 1) {
                        offboard_setpoint_counter_++;
                    }
                }
            };
            timer_ = this->create_wall_timer(100ms, command);
        }

        void arm(){
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

            RCLCPP_INFO(this->get_logger(), "Arm command send");
        }

        void disarm(){
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	        RCLCPP_INFO(this->get_logger(), "Disarm command send");
        }

        ~SimpleControl(){}

    private :
        Vector2f Target;
        Vector2f Current;
        float input1, input2;
        bool setup=false, end=false, is_armed=false, is_armed_send=false;
        rclcpp::TimerBase::SharedPtr timer_;
        uint64_t offboard_setpoint_counter_;

        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
        std::atomic<uint64_t> timestamp_;

        void subscription_position(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg){
            RCLCPP_INFO(this->get_logger(), "Current XPos : %.2f, YPos : %.2f", msg->x, msg->y);
            Current = Vector2f(msg->x, msg->y);
            Vector2f Diff = Current - Target;
            if (Diff.norm() < 2.0f){
                setup=false;
                if (end){
                    disarm();
                    rclcpp::shutdown();
                }
            }
        }

        void subscription_status(const px4_msgs::msg::VehicleStatus::UniquePtr msg){
            RCLCPP_INFO(this->get_logger(), "Current State : %d, Target : %d",msg->arming_state, px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED);
            if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED){
                is_armed = false;
            }
            else {
                is_armed = true;
            }
        }

        void publish_offboard_control_mode(){
            OffboardControlMode msg{};
            msg.position = true;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_publisher_->publish(msg);
        }

        void publish_trajectory_setpoint(Vector2f &Target){
            TrajectorySetpoint msg{};
            msg.position = {Target(0), Target(1), 0.0f};
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_setpoint_publisher_->publish(msg);
        }

        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0){
            VehicleCommand msg{};
            msg.param1 = param1;
            msg.param2 = param2;
            msg.command = command;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);
        }

};

int main(int argc, char * argv[]){
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
