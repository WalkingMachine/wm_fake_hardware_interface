//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMFakeHardwareInterface_H
#define PROJECT_WMFakeHardwareInterface_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <pluginlib/class_list_macros.h>

namespace wm_fake_hardware_interface
{
    class WMFakeHardwareInterface : public hardware_interface::RobotHW {
    public:
        // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
        // Functions
        virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
        virtual void read(const ros::Time &time, const ros::Duration &period);
        virtual void write(const ros::Time &time, const ros::Duration &period);

        // Interface variables
        std::string Name;
        double cmd;
        double pos;
        double vel;
        double eff;

    private:
        // Variables
        static hardware_interface::VelocityJointInterface joint_velocity_interface_;
        static hardware_interface::PositionJointInterface joint_position_interface_;
        static hardware_interface::JointStateInterface joint_state_interface_;
        //std::string port;
        ros::NodeHandle nh;
        std::string mode;
        ros::Time lastRead;

    };
}
#endif //PROJECT_WMFakeHardwareInterface_H
