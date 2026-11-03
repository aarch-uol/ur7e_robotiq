#include "topic_sensor_bridge/topic_sensor_hardware.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace topic_sensor_bridge
{

namespace
{
constexpr std::array<const char *, 6> kBiasParameterNames{
  "force_bias_x", "force_bias_y", "force_bias_z",
  "torque_bias_x", "torque_bias_y", "torque_bias_z"};

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

TopicSensorHardware::~TopicSensorHardware()
{
  stop_executor();
}

hardware_interface::CallbackReturn TopicSensorHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  sensor_name_ = info_.hardware_parameters.count("sensor_name") 
    ? info_.hardware_parameters.at("sensor_name") 
    : "tcp_fts_sensor";
  
  topic_name_ = info_.hardware_parameters.count("topic_name") 
    ? info_.hardware_parameters.at("topic_name") 
    : "/ft/force_torque_sensor_broadcaster/wrench";

  for (std::size_t i = 0; i < wrench_bias_.size(); ++i) {
    if (!read_double_parameter(kBiasParameterNames[i], wrench_bias_[i])) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (!read_double_parameter("max_sample_age_sec", max_sample_age_sec_) ||
      max_sample_age_sec_ <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TopicSensorHardware"),
      "max_sample_age_sec must be finite and greater than zero");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("TopicSensorHardware"),
    "Initialized sensor_name='%s', topic='%s', bias="
    "[%.5f, %.5f, %.5f, %.5f, %.5f, %.5f], max_age=%.3fs",
    sensor_name_.c_str(), topic_name_.c_str(),
    wrench_bias_[0], wrench_bias_[1], wrench_bias_[2],
    wrench_bias_[3], wrench_bias_[4], wrench_bias_[5], max_sample_age_sec_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool TopicSensorHardware::read_double_parameter(
  const std::string & name, double & value) const
{
  const auto parameter = info_.hardware_parameters.find(name);
  if (parameter == info_.hardware_parameters.end()) {
    return true;
  }

  try {
    std::size_t parsed_characters = 0;
    const double parsed_value = std::stod(parameter->second, &parsed_characters);
    if (parsed_characters != parameter->second.size() || !std::isfinite(parsed_value)) {
      throw std::invalid_argument("not a finite number");
    }
    value = parsed_value;
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TopicSensorHardware"),
      "Invalid hardware parameter '%s'='%s': %s", name.c_str(),
      parameter->second.c_str(), exception.what());
    return false;
  }
  return true;
}

hardware_interface::CallbackReturn TopicSensorHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_executor();
  wrench_sub_.reset();
  ft_values_.fill(0.0);
  wrench_buffer_.writeFromNonRT(WrenchSample{});

  // Create a node for topic subscription
  node_ = std::make_shared<rclcpp::Node>("topic_sensor_bridge_node");
  
  // Subscribe to wrench topic
  wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_name_, rclcpp::SensorDataQoS(),
    std::bind(&TopicSensorHardware::wrench_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("TopicSensorHardware"),
    "Subscribed to topic '%s'", topic_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicSensorHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Start spinning the node in a separate thread
  stop_executor();
  ft_values_.fill(0.0);
  wrench_buffer_.writeFromNonRT(WrenchSample{});
  running_.store(true, std::memory_order_release);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {
    while (running_.load(std::memory_order_acquire)) {
      executor_->spin_some(std::chrono::milliseconds(1));
    }
  });

  RCLCPP_INFO(rclcpp::get_logger("TopicSensorHardware"), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicSensorHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_executor();
  
  RCLCPP_INFO(rclcpp::get_logger("TopicSensorHardware"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicSensorHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_executor();
  wrench_sub_.reset();
  node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicSensorHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_executor();
  wrench_sub_.reset();
  node_.reset();
  RCLCPP_ERROR(rclcpp::get_logger("TopicSensorHardware"),
    "F/T samples went stale (> %.3f s): bridge stopped. Re-activate it once '%s' is publishing.",
    max_sample_age_sec_, topic_name_.c_str());
  // SUCCESS = go to 'unconfigured', from where set_hardware_component_state can bring it back.
  return hardware_interface::CallbackReturn::SUCCESS;
}

void TopicSensorHardware::stop_executor()
{
  running_.store(false, std::memory_order_release);
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.reset();
}

std::vector<hardware_interface::StateInterface> TopicSensorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export force interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    sensor_name_, "force.x", &ft_values_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    sensor_name_, "force.y", &ft_values_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    sensor_name_, "force.z", &ft_values_[2]));

  // Export torque interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    sensor_name_, "torque.x", &ft_values_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    sensor_name_, "torque.y", &ft_values_[4]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    sensor_name_, "torque.z", &ft_values_[5]));

  RCLCPP_INFO(rclcpp::get_logger("TopicSensorHardware"),
    "Exported 6 state interfaces for sensor '%s'", sensor_name_.c_str());

  return state_interfaces;
}

hardware_interface::return_type TopicSensorHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto * sample = wrench_buffer_.readFromRT();
  const int64_t max_sample_age_ns = static_cast<int64_t>(max_sample_age_sec_ * 1e9);
  if (sample == nullptr || !sample->valid) {
    // The admittance chain is activated after the sensor broadcaster, so fresh data
    // should normally arrive before these interfaces are consumed.
    ft_values_.fill(0.0);
    return hardware_interface::return_type::OK;
  }
  if (steady_now_ns() - sample->steady_time_ns > max_sample_age_ns) {
    // Returning ERROR is safer than exporting zero: after gravity compensation,
    // a zero raw wrench would look like a full payload force in the opposite direction.
    return hardware_interface::return_type::ERROR;
  }

  ft_values_ = sample->values;
  return hardware_interface::return_type::OK;
}

void TopicSensorHardware::wrench_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  const std::array<double, 6> raw_values{
    msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};

  WrenchSample sample;
  for (std::size_t i = 0; i < raw_values.size(); ++i) {
    if (!std::isfinite(raw_values[i])) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Ignoring non-finite F/T sensor sample");
      return;
    }
    sample.values[i] = raw_values[i] - wrench_bias_[i];
  }
  sample.steady_time_ns = steady_now_ns();
  sample.valid = true;
  wrench_buffer_.writeFromNonRT(sample);
}

}  // namespace topic_sensor_bridge

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  topic_sensor_bridge::TopicSensorHardware, hardware_interface::SensorInterface)
