#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <string>

namespace robo_control
{

class Hardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & /*info*/) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("drive_joint", "position", &drive_pos_);
    state_interfaces.emplace_back("drive_joint", "velocity", &drive_vel_);
    state_interfaces.emplace_back("steer_joint", "position", &steer_pos_);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("drive_joint", "velocity", &drive_cmd_);
    command_interfaces.emplace_back("steer_joint", "position", &steer_cmd_);
    return command_interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    // Simple integration: velocity -> position for drive joint
    double dt = period.seconds();
    drive_pos_ += drive_vel_ * dt;
    
    // State values are already exposed via pointers
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // Apply commands to state variables for simulation
    drive_vel_ = drive_cmd_;
    steer_pos_ = steer_cmd_;
    
    return hardware_interface::return_type::OK;
  }

private:
  double drive_pos_ = 0.0;
  double drive_vel_ = 0.0;
  double drive_cmd_ = 0.0;

  double steer_pos_ = 0.0;
  double steer_cmd_ = 0.0;
};

}  // namespace robo_control

// Export plugin
PLUGINLIB_EXPORT_CLASS(robo_control::Hardware, hardware_interface::SystemInterface)
