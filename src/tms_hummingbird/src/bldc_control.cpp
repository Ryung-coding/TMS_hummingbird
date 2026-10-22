#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include "params.hpp"

class BldcControl : public rclcpp::Node
{
public:
  BldcControl() : Node("bldc_control")
  {
    motor_pub_ = create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
    mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/tms/bldc_state", 10);
    output_pub_ = create_publisher<std_msgs::msg::Float32>("/tms/motor_output", 10);
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) { status_ = *msg; status_time_ = now(); status_seen_ = true; });
    sensor_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>("/RFT_FORCE1", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr) { sensor_time_ = now(); sensor_seen_ = true; });
    servo_sub_ = create_subscription<std_msgs::msg::Bool>("/tms/servo_ready", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        servo_ready_ = msg->data; servo_time_ = now(); servo_seen_ = true;
      });
    limit_sub_ = create_subscription<std_msgs::msg::Float32>("/tms/u_max", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        if (std::isfinite(msg->data) && msg->data >= 0.0F && msg->data <= 1.0F) {
          u_max_ = msg->data; limit_time_ = now(); limit_seen_ = true;
          target_ = std::min(target_, u_max_);
        } else { limit_seen_ = false; stop(); }
      });
    motor_sub_ = create_subscription<std_msgs::msg::Float32>("/tms/motor_target", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        if (std::isfinite(msg->data) && msg->data >= 0.0F && msg->data <= u_max_ + 1.0e-6F) {
          target_ = std::min(msg->data, u_max_); target_time_ = now(); target_seen_ = true;
        } else { stop(); }
      });
    mode_sub_ = create_subscription<std_msgs::msg::Bool>("/tms/offboard_request", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) { if (msg->data && ready()) { mode_requested_ = true; mode_start_ = now(); }
        else if (!msg->data) stop(); });
    arm_sub_ = create_subscription<std_msgs::msg::Bool>("/tms/arm_request", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && ready() && mode_requested_ &&
            status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) arm_requested_ = true;
        else if (!msg->data) stop();
      });
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>("/tms/emergency_stop", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) { if (msg->data) stop(); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { tick(); });
  }
  ~BldcControl() override { if (rclcpp::ok()) stop(); }

private:
  bool ready() const
  {
    return status_seen_ && sensor_seen_ && servo_seen_ && servo_ready_ && limit_seen_ && target_seen_ &&
      (now() - status_time_).seconds() < params::message_timeout_sec &&
      (now() - sensor_time_).seconds() < params::SENSOR_TIMEOUT_SEC &&
      (now() - servo_time_).seconds() < params::SERVO_TIMEOUT_SEC &&
      (now() - limit_time_).seconds() < params::MOTOR_TIMEOUT_SEC &&
      (now() - target_time_).seconds() < params::MOTOR_TIMEOUT_SEC &&
      !status_.failsafe && motor_pub_->get_subscription_count() > 0 &&
      count_publishers("/fmu/in/actuator_motors") == 1 &&
      count_publishers("/fmu/in/offboard_control_mode") == 1 &&
      count_publishers("/fmu/in/vehicle_command") == 1;
  }
  void vehicle_command(uint16_t command, float param1, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    msg.param1 = param1; msg.param2 = param2; msg.command = command;
    msg.target_system = params::target_system_id;
    msg.target_component = params::target_component_id;
    msg.source_system = params::source_system_id;
    msg.source_component = params::source_component_id;
    msg.from_external = true;
    command_pub_->publish(msg);
  }
  void publish_motor(float u)
  {
    px4_msgs::msg::ActuatorMotors msg{};
    msg.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    msg.timestamp_sample = msg.timestamp;
    msg.control.fill(std::numeric_limits<float>::quiet_NaN());
    for (int i = 0; i < 4; ++i) msg.control[i] = u;
    motor_pub_->publish(msg);
    std_msgs::msg::Float32 output{}; output.data = u; output_pub_->publish(output);
  }
  void stop()
  {
    const bool owned = mode_requested_ || arm_requested_;
    target_ = 0.0F;
    arm_requested_ = false;
    mode_requested_ = false;
    publish_motor(0.0F);
    if (owned) {
      vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
      disarm_repeat_ = 20;
    }
  }
  void tick()
  {
    const bool safe = ready();
    const bool armed = status_seen_ && status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard = status_seen_ && status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    if (!safe && (mode_requested_ || arm_requested_)) stop();
    if (mode_requested_ && safe)
    {
      px4_msgs::msg::OffboardControlMode mode{};
      mode.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
      mode.direct_actuator = true;
      mode_pub_->publish(mode);
      if (!offboard && (now() - mode_start_).seconds() > params::command_delay_sec &&
          (now() - last_command_).seconds() > params::command_repeat_sec)
      {
        vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        last_command_ = now();
      }
    }
    if (arm_requested_ && safe && offboard && !armed &&
        (now() - last_command_).seconds() > params::command_repeat_sec)
    {
      vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      last_command_ = now();
    }
    if (arm_requested_ && armed && (!offboard || !mode_requested_)) stop();
    publish_motor((safe && armed && offboard && arm_requested_) ? target_ : 0.0F);
    if (disarm_repeat_ > 0) {
      vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
      --disarm_repeat_;
    }
    std_msgs::msg::String state{};
    state.data = std::string(safe ? "READY" : "NOT_READY") +
      (offboard ? " OFFBOARD" : " MANUAL") + (armed ? " ARMED" : " DISARMED");
    state_pub_->publish(state);
  }
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motor_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr limit_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_, arm_sub_, emergency_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  px4_msgs::msg::VehicleStatus status_{};
  rclcpp::Time status_time_{0, 0, RCL_ROS_TIME}, sensor_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time servo_time_{0, 0, RCL_ROS_TIME}, target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time limit_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mode_start_{0, 0, RCL_ROS_TIME}, last_command_{0, 0, RCL_ROS_TIME};
  float target_{0.0F};
  float u_max_{0.0F};
  int disarm_repeat_{0};
  bool status_seen_{false}, sensor_seen_{false}, servo_seen_{false}, servo_ready_{false};
  bool target_seen_{false}, limit_seen_{false};
  bool mode_requested_{false}, arm_requested_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BldcControl>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
