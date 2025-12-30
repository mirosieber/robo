#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <pigpio.h>
#include <iostream>

#include <vector>
#include <string>

namespace robo_control
{

class Hardware : public hardware_interface::SystemInterface
{

private:
  int steerServoPin{}; // PWM0 pin
  int trottleServoPin{}; // PWM1 pin
  void initGPIO(){
    if (gpioInitialise() < 0){
      std::cerr << "Pigpio initialization failed!" << std::endl;
    }
  }
  void closeGPIO(){
    gpioTerminate();
  }
  void setVel(double velocity){
    // Implement servo/motor control for drive velocity
    // Convert velocity to pulse width in microseconds (1ms to 2ms)
    uint16_t pulseWidth = static_cast<uint16_t>(1500 + (velocity * 500)); // Assuming velocity in range [-1, 1]
    gpioServo(trottleServoPin, pulseWidth);
  }

  void setSteerPos(double angle){
    // Implement servo control for steering position
    // Convert angle to pulse width in microseconds (1ms to 2ms)
    uint16_t pulseWidth = static_cast<uint16_t>(1000 + ((angle * 1000) / 180.0)); // Assuming angle in range [-90, 90]
    gpioServo(steerServoPin, pulseWidth);
  }


public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    try {
      steerServoPin = std::stoi(info.hardware_parameters.at("steer_servo_pin"));
      trottleServoPin = std::stoi(info.hardware_parameters.at("drive_motor_pin"));
    } catch (const std::exception& e) {
      std::cerr << "Failed to read servo pins from URDF: " << e.what() << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }
    initGPIO();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  { 
    closeGPIO();
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

    //Apply commands to hardware here in a real implementation
    setVel(drive_cmd_);
    setSteerPos(steer_cmd_);
    
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
