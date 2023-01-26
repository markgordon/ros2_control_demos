// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_hardware/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  std::string port1,port2;
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  last_query_nano_ = std::chrono::system_clock().now().time_since_epoch();

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  port1 = info_.hardware_parameters["ddms_tty1"];
  port2  = info_.hardware_parameters["ddms_tty2"];
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  last_hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  current_wheel_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  current_wheel_position_[0] = 0;
  current_wheel_position_[1] = 0;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "open port");

  port1 = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_54D2035530-if00";
  ddms_diff::return_type ret = wheel_command[0].open(port1);
  if(ret != ddms_diff::return_type::SUCCESS)
  {
    RCLCPP_FATAL(rclcpp::get_logger("DiffBotSystemHardware"),"Couldn't open port %s, code %i",port1.c_str(),(int)ret);
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  port2 = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9XOMIL6-if00-port0";
  ret = wheel_command[1].open(port2);

  if(ret != ddms_diff::return_type::SUCCESS)
  {
    RCLCPP_FATAL(rclcpp::get_logger("DiffBotSystemHardware"),"Couldn't open port %s, code %i",port2.c_str(),(int)ret);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
    //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "export state");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
    //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "export command");
  
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "activate");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  
  //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
   // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "close port");

  wheel_command[0].close();
  wheel_command[1].close();
  //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}
//***********************************************************Read Joints
hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  for(int i=0;i<2;i++){
    std::vector<double> state;
    //re-use last rpm, since we have to update to get detailed info
    ddms_diff::return_type retval = wheel_command[i].get_wheel_state(i+1,last_hw_commands_[i],state);
    if(retval != ddms_diff::return_type::SUCCESS){
      RCLCPP_ERROR(
      rclcpp::get_logger("DiffBotSystemHardware"), "Read joint fail");
       return hardware_interface::return_type::ERROR;
    }
    //states may be empty if there was a non-fatal read error
    if(state.size() > 1){
      //if this is first measurement the wheels are at starting position
      if(last_angle_[i] == -1)last_angle_[i] = round(state[1]*1000)/1000.0;
      else{
          if(round(state[0]) !=0){
            //create absolute encoder from relative
            state[1] = round(state[1]*1000)/1000.0;
            double delta=state[1] - last_angle_[i];
            //rotating backwards over last interval
            if(state[0] < 0){
              if (last_angle_[i] - state[1] < -M_PI){
                delta = -(2*M_PI - state[1] + last_angle_[i]);
              }
            }else if(state[0] > 0){
              if (last_angle_[i] - state[1] > M_PI){
                delta = round((2*M_PI - last_angle_[i] + state[1])*1000)/1000.0;
              }
            }
           // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "id %d last %f :current %f sp %f",i,last_angle_[i] ,state[1],state[0],delta);

            current_wheel_position_[i]+=delta;
          }
          last_angle_[i] = state[1];

      }
      //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "pos %f",current_wheel_position_[i]);

      hw_velocities_[i] = state[0];
      hw_positions_[i] = current_wheel_position_[i];
      last_state_[i][0] = state[0];
      last_state_[i][1] = state[1];
    }else{
      hw_positions_[i] = current_wheel_position_[i];
    }
  }
  return hardware_interface::return_type::OK;
}
//**************************************************************Write Joints
hardware_interface::return_type ros2_control_demo_hardware::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  last_hw_commands_[0] = hw_commands_[0];
  last_hw_commands_[1] = -hw_commands_[1];
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)
