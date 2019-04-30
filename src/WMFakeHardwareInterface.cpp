//
// Created by philippe on 03/05/17.
//

#include "WMFakeHardwareInterface.h"
#include <nodelet/nodelet.h>

namespace wm_fake_hardware_interface {


    hardware_interface::VelocityJointInterface WMFakeHardwareInterface::joint_velocity_interface_;
    hardware_interface::PositionJointInterface WMFakeHardwareInterface::joint_position_interface_;
    hardware_interface::JointStateInterface    WMFakeHardwareInterface::joint_state_interface_;

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>

	bool WMFakeHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
		using namespace hardware_interface;
        ROS_INFO("Loading fake joint");
		
		// Get parameters
        mode = "velocity";
        std::vector<std::string> Joints;
        if (!robot_hw_nh.getParam("joints", Joints)) { return false; }
        robot_hw_nh.getParam("mode", mode);
        Name = Joints[0];
        lastRead = ros::Time::now();

		// Initialise interface variables
		cmd = 0;	//command
		pos = 0;    //position
		vel = 0;    //velocity
		eff = 0;    //effort


        ROS_INFO("In %s mode.", mode.c_str());

        // Register interfaces
        joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
        registerInterface(&joint_state_interface_);
        if ( !mode.compare("velocity") ){
            joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
            registerInterface(&joint_velocity_interface_);
        } else if ( !mode.compare("position") ){
            joint_position_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
            registerInterface(&joint_position_interface_);
        } else {
            ROS_ERROR("Invalid fake joint mode");
        }
        ROS_INFO("Successfult loaded fake joint");
		return true;
	}
	
	void WMFakeHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        if ( mode == "velocity" )
            pos += cmd * (ros::Time::now()-lastRead).toSec();
            lastRead = ros::Time::now();
        if ( mode == "position" )
            pos = cmd;
	}
	
	void WMFakeHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {

	}

}
PLUGINLIB_EXPORT_CLASS(wm_fake_hardware_interface::WMFakeHardwareInterface, hardware_interface::RobotHW)
