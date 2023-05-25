#include <rigid_arm_hw_int_node/rigid_arm_hardware_interface.hpp>
#include "pi_comm_pkg/servo_arm.h"
//##include "pi_comm_pkg/arm_encoders.h"

RigidArm::RigidArm(ros::NodeHandle& nh) : nh_(nh) {
// Create a publisher for joint angles
    servo_pub_ = nh_.advertise<pi_comm_pkg::servo_arm>("pi_comm/servo_arm", 10);
    
 
// Subscribe to arm encoders
	nh_.subscribe("pi_comm/arm_encoders", 10, &RigidArm::EncoderCallback, this);
	
// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();

// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

//Set the frequency of the control loop.
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &RigidArm::update, this);
}


RigidArm::~RigidArm() {
}


void RigidArm::EncoderCallback(const pi_comm_pkg::arm_encoders::ConstPtr& msg)
{
  for (int i = 0; i < 3; i++) {
        joint_position_update_[i] =  msg->arm_encoders_rad[i];
    }
}


void RigidArm::init() {
    int num_joints = 6;
    std::string joint_names[] = {"joint_01", "joint_02", "joint_03", "joint_04", "fake_joint_05", "fake_joint_06"};

    for (int i = 0; i < num_joints; i++) {
        // Create joint_state_interface for Joint X
        hardware_interface::JointStateHandle jointStateHandle(joint_names[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

// Create position joint interface as Joint X accepts position command.
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);
// Create Joint Limit interface for Joint X
        joint_limits_interface::getJointLimits(joint_names[i], nh_, limits);
        joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
        positionJointSaturationInterface.registerHandle(jointLimitsHandle);
        
        
    }


// Register all joints interfaces
    registerInterface(&joint_state_interface_);
    // registerInterface(&effort_joint_interface_);
    registerInterface(&position_joint_interface_);
    // registerInterface(&effortJointSaturationInterface);
    registerInterface(&positionJointSaturationInterface);


}

//This is the control loop
void RigidArm::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void RigidArm::read() {


//Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort

    //from robot.
    // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]

    for (int i = 0; i < 3; i++) {
        joint_position_[i] = joint_position_update_[i];
    }
    
    joint_position_[3] = joint_position_command_[3];
    joint_position_[4] = joint_position_command_[4];
    joint_position_[5] = joint_position_command_[5];

}

void RigidArm::write(ros::Duration elapsed_time) {
    // Safety
    //effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
    positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC


    // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
    // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and

    //joint_position_command_ for JointC.



    for (int i = 0; i < 6; i++) {
        std::cout << joint_position_command_[i] << std::endl;
    }


    pi_comm_pkg::servo_arm msg;
    msg.servo_arm_rad = {joint_position_command_[0], joint_position_command_[1], joint_position_command_[2]};
    // Publish the message
    std::cout << "Publishing" << std::endl;
    servo_pub_.publish(msg);
}


int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "rigid_arm_hardware_interface_node");
    ros::NodeHandle nh;

    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers

    ros::MultiThreadedSpinner spinner(2);


    // Create the object of the robot hardware_interface class and spin the thread.
    RigidArm ROBOT(nh);
    spinner.spin();

    return 0;
}
