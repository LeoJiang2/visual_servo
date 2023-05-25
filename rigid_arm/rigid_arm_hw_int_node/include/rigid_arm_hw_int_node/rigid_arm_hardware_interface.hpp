#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "pi_comm_pkg/arm_encoders.h"

class RigidArm : public hardware_interface::RobotHW 
{
    public:
        RigidArm(ros::NodeHandle& nh);
        ~RigidArm();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        double joint_position_update_[3];
        
    protected:
		void EncoderCallback(const pi_comm_pkg::arm_encoders::ConstPtr& msg);
        hardware_interface::JointStateInterface joint_state_interface_;
        //hardware_interface::EffortJointInterface effort_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        //joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        

        double joint_position_[6];
        double joint_velocity_[6];
        double joint_effort_[6];
        //double joint_effort_command_[2];
        double joint_position_command_[6];
        
        
        ros::NodeHandle nh_;
        ros::Publisher servo_pub_;

        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
