#include "waveshare_servos.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <limits>  // for quiet_NaN

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace waveshare_servos
{
hardware_interface::CallbackReturn WaveshareServos::on_init(
	const hardware_interface::HardwareInfo & info)
{
	if (
		hardware_interface::SystemInterface::on_init(info) !=
    	hardware_interface::CallbackReturn::SUCCESS)
  	{
    	return hardware_interface::CallbackReturn::ERROR;
  	}
	// check urdf definitions
	pos_offsets_.resize(info_.joints.size(), 0.0);
	int i = 0;
	for (const hardware_interface::ComponentInfo & joint : info_.joints)
	{
		all_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
		// check num, order, and type of state interfaces
		if (joint.state_interfaces.size() != 4)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"joint has the wrong number of state interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the position state interface first");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the velocity state interface second");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[2].name != "torque")
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the torque state interface third");
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.state_interfaces[3].name != "temperature")
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
				"a joint does not have the temperature state interface fourth");
			return hardware_interface::CallbackReturn::ERROR;
		}
		// check presence and types of command interfaces
		if (joint.command_interfaces.size() < 1)
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"), 
				"a joint does not have a command interfaces");
			return hardware_interface::CallbackReturn::ERROR;
		}
		for (long unsigned int ci = 0; ci < joint.command_interfaces.size(); ci++)
		{
			if (joint.command_interfaces[ci].name != hardware_interface::HW_IF_POSITION &&
				joint.command_interfaces[ci].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"),
					"a joint is using a command interface that isn't position or velocity");
				return hardware_interface::CallbackReturn::ERROR;
			}
		}
		// store ids in different vectors by type
		if (joint.parameters.find("type")->second == "pos")
		{
			pos_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			pos_is_.emplace_back(i);
		} 
		else if (joint.parameters.find("type")->second == "vel") 
		{
			vel_ids_.emplace_back(std::stoul(joint.parameters.find("id")->second));
			vel_is_.emplace_back(i);
		}
		else 
		{
			RCLCPP_FATAL(rclcpp::get_logger("waveshare_servos"), 
				"a joint has the wrong type, it should be vel or pos");
			return hardware_interface::CallbackReturn::ERROR;
		}
		// save pose offsets to work around motor movement limitations
		auto offset = joint.parameters.find("offset");
    	if (offset != joint.parameters.end())
    	{
      		pos_offsets_[i] = std::stod(offset->second);  
    	}
		i++;
	}
	// init vectors for state interfaces
	pos_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	vel_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	torq_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	temp_states_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	// create vectors for command interfaces
	pos_cmds_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	vel_cmds_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());

	// ---- ADDED (unwrap state storage) ----
	last_raw_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	rev_count_.resize(all_ids_.size(), 0);
	last_pos_meas_.resize(all_ids_.size(), std::numeric_limits<double>::quiet_NaN());
	// -------------------------------------

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_configure(
  	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// start servo communication
	if (!sm_st.begin(baudrate_, port_.c_str()))
	{
		return hardware_interface::CallbackReturn::ERROR;
	}
	// ping motors
	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		if (sm_st.Ping(all_ids_[i]) == -1)
		{
			RCLCPP_WARN(rclcpp::get_logger("waveshare_servos"), 
				"unable to ping motor id '%d'", all_ids_[i]);
		}
	}
	// pointers to ids for motor control
	p_ids_pnt_ = &pos_ids_[0];
	v_ids_pnt_ = &vel_ids_[0];
	// arrays for servo commands
	p_pos_ar_ = new s16[pos_ids_.size()];
	p_vel_ar_ = new u16[pos_ids_.size()];
	p_acc_ar_ = new  u8[pos_ids_.size()];
	v_vel_ar_ = new s16[vel_ids_.size()];
	v_acc_ar_ = new  u8[vel_ids_.size()];
	// set motor modes: 0 = servo, 1 = closed loop wheel; set max acceleration
	for (u8 i = 0; i < pos_ids_.size(); i++)
	{
		sm_st.Mode(pos_ids_[i], 0); 
		p_acc_ar_[i] = max_acc_;
	}
	for (u8 i = 0; i < vel_ids_.size(); i++)
	{
		sm_st.Mode(vel_ids_[i], 1);
		v_acc_ar_[i] = max_acc_;
	}
	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WaveshareServos::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;
	for (u8 i = 0; i < all_ids_.size(); i++)
	{
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, "torque", &torq_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			info_.joints[i].name, "temperature", &temp_states_[i]));
	}
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WaveshareServos::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;
	for (u8 i = 0; i < all_ids_.size(); i++)
	{
		for (long unsigned int ci = 0; ci < info_.joints[i].command_interfaces.size(); ci++)
		{
			if (info_.joints[i].command_interfaces[ci].name == hardware_interface::HW_IF_POSITION) 
			{
				command_interfaces.emplace_back(hardware_interface::CommandInterface(
					info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_cmds_[i]));
			}
			if (info_.joints[i].command_interfaces[ci].name == hardware_interface::HW_IF_VELOCITY) 
			{
				command_interfaces.emplace_back(hardware_interface::CommandInterface(
					info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_cmds_[i]));
			}
		}
	}
	return command_interfaces;
}

hardware_interface::CallbackReturn WaveshareServos::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// set position commands to current positions before any movement to not move on start
	for (size_t i = 0; i < all_ids_.size(); i++)
  	{
    	double init_raw_pos = get_position(all_ids_[i]);  // returns inverted for ID=2
    	pos_cmds_[i] = init_raw_pos - pos_offsets_[i];
	}
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WaveshareServos::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	// set velocities to 0 on close, doesn't work on ctrl-C
	for (size_t i = 0; i < vel_cmds_.size(); i++)
	{
    	vel_cmds_[i] = 0.0;
  	}
	auto now    = rclcpp::Clock().now();
  	auto period = rclcpp::Duration(0, 0);  // zero duration
  	this->write(now, period);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WaveshareServos::read(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & period)  // <-- use period
{
	const double dt = std::max(1e-6, period.seconds());               // <-- added

	for (size_t i = 0; i < all_ids_.size(); i++)
	{
		const int id = all_ids_[i];

		// ---- CHANGED: unwrap raw servo angle (no sign) ----
		double raw_no_sign = sm_st.ReadPos(id) * 2.0 * M_PI / steps_;  // [0, 2π)
		if (!std::isfinite(last_raw_[i])) {
			last_raw_[i] = raw_no_sign;  // first sample
		} else {
			double d = raw_no_sign - last_raw_[i];
			if (d >  M_PI) rev_count_[i] -= 1;  // crossed 2π -> 0
			if (d < -M_PI) rev_count_[i] += 1;  // crossed 0  -> 2π
			last_raw_[i] = raw_no_sign;
		}
		double unwrapped = raw_no_sign + rev_count_[i] * 2.0 * M_PI;

		// apply sign (ID==2 inverted) and your configured offset
		const double sign = (id == 2) ? -1.0 : 1.0;
		const double pos_meas = sign * unwrapped - pos_offsets_[i];

    	pos_states_[i] = pos_meas;

		// ---- CHANGED: velocity from unwrapped position (consistent & spike-free) ----
		if (std::isfinite(last_pos_meas_[i])) {
			vel_states_[i] = (pos_meas - last_pos_meas_[i]) / dt;
		} else {
			vel_states_[i] = 0.0;
		}
		last_pos_meas_[i] = pos_meas;
		// ----------------------------------------------------

		torq_states_[i] = get_torque(id);
		temp_states_[i] = get_temperature(id);
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type WaveshareServos::write(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	// position-mode servos
	for (size_t i = 0; i < pos_is_.size(); i++)
	{
		// Determine sign based on the actual servo ID for this index
		const int servo_id = pos_ids_[i];
		const double sign = (servo_id == 2) ? -1.0 : 1.0;

		// Desired robot-frame absolute angle is (cmd + offset).
		// Servo-frame target should flip sign for ID=2.
		double robot_target = pos_cmds_[pos_is_[i]] + pos_offsets_[pos_is_[i]];
		double servo_target = sign * robot_target;

		p_pos_ar_[i] = static_cast<s16>((servo_target * steps_) / (2 * M_PI));

		// For position-mode speed, many servos expect magnitude (u16). Use abs().
		double robot_speed = vel_cmds_[pos_is_[i]];
		double servo_speed_mag = std::abs(sign * robot_speed);
		p_vel_ar_[i] = static_cast<u16>((servo_speed_mag * steps_) / (2 * M_PI));
	}

	// wheel-mode servos (velocity control)
	for (size_t i = 0; i < vel_is_.size(); i++)
	{
		const int servo_id = vel_ids_[i];
		const double sign = (servo_id == 2) ? -1.0 : 1.0;

		double robot_vel = vel_cmds_[vel_is_[i]];
		double servo_vel = sign * robot_vel;

		v_vel_ar_[i] = static_cast<s16>((servo_vel * steps_) / (2 * M_PI));
	}

    sm_st.SyncWritePosEx(p_ids_pnt_, static_cast<u8>(pos_ids_.size()), 
		p_pos_ar_, p_vel_ar_, p_acc_ar_); 
	sm_st.SyncWriteSpe(v_ids_pnt_, static_cast<u8>(vel_ids_.size()), 
		v_vel_ar_, v_acc_ar_); 
	return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn WaveshareServos::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
	sm_st.end();
	delete[] p_pos_ar_;
	delete[] p_vel_ar_;
	delete[] p_acc_ar_;
	delete[] v_vel_ar_;
	delete[] v_acc_ar_;
	return hardware_interface::CallbackReturn::SUCCESS;
}

double WaveshareServos::get_position(int ID)
{
    // Flip sign for motor ID 2 so robot-frame position is negated
    double pos = sm_st.ReadPos(ID) * 2 * M_PI / steps_;
    if (ID == 2) pos = -pos;
    return pos;
}

double WaveshareServos::get_velocity(int ID)
{
    // Flip sign for motor ID 2 so robot-frame velocity is negated
    double vel = sm_st.ReadSpeed(ID) * 2 * M_PI / steps_; // rads / s
    if (ID == 2) vel = -vel;
    return vel;
}

double WaveshareServos::get_torque(int ID)
{
    // ReadCurrent(ID) return unitless value, multiply by static current (6mA)
    int current = sm_st.ReadCurrent(ID) * 6.0 / 1000.0;
    double torque = current * KT_;
    return torque;
}

double WaveshareServos::get_temperature(int ID)
{
    double temp = static_cast<double>(sm_st.ReadTemper(ID));
    return temp;
}

}  // namespace waveshare_servos

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	waveshare_servos::WaveshareServos, hardware_interface::SystemInterface)
