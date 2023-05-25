#include <ros/ros.h>
#include <ikfast.h>
#include "pi_comm_pkg/servo_arm.h"
#include <iostream>
#include "task_space_pkg/rigid_arm_position_desired.h"
#include "pi_comm_pkg/arm_state.h"
#include <cmath>

/*
https://answers.ros.org/question/290277/use-ikfast-in-separate-package/
*/


class TaskSpace {
private:
    ros::Publisher servo_pub;
    //#ros::Subscriber patriot_sub;
    ros::Subscriber rigid_arm_position_desired_sub;
    ros::Subscriber arm_state_sub;

    double joint_1_current = 0;
    double joint_2_current = 0;
    double joint_3_current = 0;
    double theta_4_current = 0; // Kept to send to publisher
    //#float x_w, y_w, z_w;
    //#float x_arm, y_arm, z_arm;
    
    float x_d, y_d, z_d;
    bool update_flag = false;
    

    double distance(std::vector<IkReal> test){
        double distance = sqrt(pow((joint_1_current - test[0]),2) + pow((joint_2_current - test[1]),2) + pow((joint_3_current - test[2]),2));
        return distance;
    }

public:
    TaskSpace(ros::NodeHandle *nh) {
        // Publisher
        servo_pub = nh->advertise<pi_comm_pkg::servo_arm>("pi_comm/servo_arm", 10);
        // Subscriber
        //#patriot_sub = nh->subscribe("patriot/arm_pose", 10, &TaskSpace::patriot_callback, this);
		rigid_arm_position_desired_sub = nh->subscribe("task_space/rigid_arm_position_desired", 10, &TaskSpace::move_rigid_arm_callback, this);
        arm_state_sub = nh->subscribe("pi_comm/arm_state", 10, &TaskSpace::arm_state_update_callback, this);

        movement_loop();
    }

	
	void arm_state_update_callback(const pi_comm_pkg::arm_state &msg){
		theta_4_current = msg.theta_4_rad;
	}
	
    void movement_loop() {
	
		ros::Rate rate(20);
		while(ros::ok()){
			if(update_flag == true){
            // Calculate IK
            ikfast::IkSolutionList<IkReal> solutions;
            std::vector<IkReal> vfree(GetNumFreeParameters());
            IkReal eerot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            IkReal eetrans[3] = {x_d, y_d, z_d};

            bool bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

            if ( !bSuccess ) {
                fprintf(stderr, "Failed to get ik solution, %f, %f, %f \n", x_d, y_d, z_d);
            }
            else {
                // Publish the values
                printf("Found %d ik solutions: %f, %f, %f \n", (int)solutions.GetNumSolutions(), x_d, y_d, z_d);



                std::vector<IkReal> solvalues(GetNumJoints());
                std::vector<IkReal> nearest_sol(GetNumJoints());
                float current_distance = 10000;
                // int best_sol = 0;

                for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
                    const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);

                    //#printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
                    std::vector<IkReal> vsolfree(sol.GetFree().size());
                    sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
                    //#for( std::size_t j = 0; j < solvalues.size(); ++j)
                        //#printf("%.15f, ", solvalues[j]);

                    //#printf("\n");
                    double dist = distance(solvalues);
                    //#std::cout<<dist<<std::endl;
                    if (dist < current_distance){
                        nearest_sol = solvalues;
                        current_distance = dist;
                    }

                 }





                std::cout << "Joint 1: " << nearest_sol[0] * 180.0 / 3.14 << std::endl;
                std::cout << "Joint 2: " << nearest_sol[1] * 180.0 / 3.14 << std::endl;
                std::cout << "Joint 3: " << nearest_sol[2] * 180.0 / 3.14 << std::endl;

                pi_comm_pkg::servo_arm msg;
                msg.servo_arm_rad = {nearest_sol[0], nearest_sol[1], nearest_sol[2], theta_4_current};
                // Publish the message
                //#std::cout << "Publishing" << std::endl;
                servo_pub.publish(msg);
                joint_1_current = nearest_sol[0];
                joint_2_current = nearest_sol[1];
                joint_3_current = nearest_sol[2];
				

				
	
            } // else - successful ik
            update_flag = false;
		} //if
		ros::spinOnce();
		rate.sleep();
        } //while loop
    }


	//#void patriot_callback(const patriot_pkg::arm_pose &msg){
		//#std::cout << "Callback" << std::endl;
		//#x_w = msg.rigid_arm_pose_w[0];
		//#y_w = msg.rigid_arm_pose_w[1];
		//#z_w = msg.rigid_arm_pose_w[2];
		
		//#x_arm = x_w/100.0 + 0.0495;
		//#y_arm = -y_w/100.0 + 0.1425;
		//#z_arm = -z_w/100.0 + 0.057; 
	//#}

	void move_rigid_arm_callback(const task_space_pkg::rigid_arm_position_desired &msg){
		//#std::cout << "Callback" << std::endl;
		x_d = msg.rigid_arm_position_desired[0];
		y_d = msg.rigid_arm_position_desired[1];
		z_d = msg.rigid_arm_position_desired[2];
		update_flag = true;
		
	}

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "task_space_node");
    ros::NodeHandle nh;
    TaskSpace task = TaskSpace(&nh);
    ros::spin();
}
