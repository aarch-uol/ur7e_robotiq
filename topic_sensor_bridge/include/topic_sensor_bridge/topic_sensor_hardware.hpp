#ifndef TOPIC_SENSOR_BRIDGE__TOPIC_SENSOR_HARDWARE_HPP_
#define TOPIC_SENSOR_BRIDGE__TOPIC_SENSOR_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace topic_sensor_bridge
{

class TopicSensorHardware : public hardware_interface::SensorInterface
{
public:
  TopicSensorHardware() = default;
  ~TopicSensorHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // A failed read() takes the component to on_error -> unconfigured without on_deactivate,
  // so these must stop the spin thread too, or the next activation would overwrite a
  // running std::thread (std::terminate in the arm's ros2_control_node).
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  bool read_double_parameter(const std::string & name, double & value) const;
  void stop_executor();

  struct WrenchSample
  {
    std::array<double, 6> values{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int64_t steady_time_ns{0};
    bool valid{false};
  };

  // Parameters
  std::string sensor_name_;
  std::string topic_name_;
  std::array<double, 6> wrench_bias_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double max_sample_age_sec_{0.1};

  // State interface values (force x,y,z and torque x,y,z)
  std::array<double, 6> ft_values_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  realtime_tools::RealtimeBuffer<WrenchSample> wrench_buffer_;
  std::atomic<bool> running_{false};

  // ROS node and subscriber
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};

}  // namespace topic_sensor_bridge

#endif  // TOPIC_SENSOR_BRIDGE__TOPIC_SENSOR_HARDWARE_HPP_
