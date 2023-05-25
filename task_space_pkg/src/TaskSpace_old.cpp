#include <ros/ros.h>
#include <ikfast.h>
#include "i2c_comm_pkg/servo_arm.h"
#include <iostream>
#include "patriot_pkg/arm_pose.h"
#include <cmath>

/*
https://answers.ros.org/question/290277/use-ikfast-in-separate-package/
*/


class TaskSpace {
private:
    ros::Publisher servo_pub;
    ros::Subscriber sub;

    double joint_1_current = 0;
    double joint_2_current = 0;
    double joint_3_current = 0;
    float x_w, y_w, z_w;
    float x_arm, y_arm, z_arm;
    

    double distance(std::vector<IkReal> test){
        double distance = sqrt(pow((joint_1_current - test[0]),2) + pow((joint_2_current - test[1]),2) + pow((joint_3_current - test[2]),2));
        return distance;
    }

public:
    TaskSpace(ros::NodeHandle *nh) {
        // Publisher
        servo_pub = nh->advertise<i2c_comm_pkg::servo_arm>("i2c_comm/servo_arm", 10);
        // Subscriber
        sub = nh->subscribe("patriot/arm_pose", 10, &TaskSpace::patriot_callback, this);

        movement_loop();
    }


    void movement_loop() {
        bool flag = true;
        while (flag  == true) {
            // Get input
            float x;
            float y;
            float z;
            std::cout << "Please enter X: ";
            std::cin >> x;
            std::cout << "Please enter Y: ";
            std::cin >> y;
            std::cout << "Please enter Z (from arm base, not ground): ";
            std::cin >> z;


            // Calculate IK
            ikfast::IkSolutionList<IkReal> solutions;
            std::vector<IkReal> vfree(GetNumFreeParameters());
            IkReal eerot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            IkReal eetrans[3] = {x, y, z};

            bool bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

            if ( !bSuccess ) {
                fprintf(stderr, "Failed to get ik solution\n");
            }
            else {
                // Publish the values
                printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());



                std::vector<IkReal> solvalues(GetNumJoints());
                std::vector<IkReal> nearest_sol(GetNumJoints());
                float current_distance = 10000;
                // int best_sol = 0;

                for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
                    const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);

                    printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
                    std::vector<IkReal> vsolfree(sol.GetFree().size());
                    sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
                    for( std::size_t j = 0; j < solvalues.size(); ++j)
                        printf("%.15f, ", solvalues[j]);

                    printf("\n");
                    double dist = distance(solvalues);
                    std::cout<<dist<<std::endl;
                    if (dist < current_distance){
                        nearest_sol = solvalues;
                        current_distance = dist;
                    }

                    }




                // for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
                //     const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
                //     printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
                //     std::vector<IkReal> vsolfree(sol.GetFree().size());
                //     sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
                //     for( std::size_t j = 0; j < solvalues.size(); ++j)
                //         printf("%.15f, ", solvalues[j]);
                //     printf("\n");
                //     }

                // const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(0);
                // sol.GetSolution(&solvalues[0], NULL);

                std::cout << "Joint 1: " << nearest_sol[0] * 180.0 / 3.14 << std::endl;
                std::cout << "Joint 2: " << nearest_sol[1] * 180.0 / 3.14 << std::endl;
                std::cout << "Joint 3: " << nearest_sol[2] * 180.0 / 3.14 << std::endl;

                i2c_comm_pkg::servo_arm msg;
                msg.servo_arm_deg = {nearest_sol[0] * 180.0 / 3.14, nearest_sol[1] * 180.0 / 3.14, nearest_sol[2] * 180.0 / 3.14, 0};
                // Publish the message
                std::cout << "Publishing" << std::endl;
                servo_pub.publish(msg);
                joint_1_current = nearest_sol[0];
                joint_2_current = nearest_sol[1];
                joint_3_current = nearest_sol[2];
				
				float prior_x = 1000;
				float prior_y = 1000;
				float prior_z = 1000;

				//Doesn't work...
				//#while(true){
					std::cout << std::abs(x_arm - prior_x) << std::endl;
					std::cout << std::abs(y_arm - prior_y) << std::endl;
					std::cout << std::abs(z_arm - prior_z) << std::endl;
					//#if(std::abs(x_arm - prior_x) < 0.005 && std::abs(y_arm - prior_y) < 0.005 && std::abs(z_arm - prior_z) < 0.005){
						//#break;
					//#}
				//#prior_x = x_arm;
				//#prior_y = y_arm;
				//#prior_z = z_arm;
				//#ros::spinOnce();
				//#}
				
				ros::spinOnce();
				std::cout << "X: " << x_arm << std::endl;
                std::cout << "Y: " << y_arm << std::endl;
                std::cout << "Z: " << z_arm << std::endl;

                char input;
                std::cout << "Continue(y/n)? " << std::endl;
                std::cin >> input;

                if(input != 'y'){
                    flag = false;
                }
            }
        }
    }


	void patriot_callback(const patriot_pkg::arm_pose &msg){
		//#std::cout << "Callback" << std::endl;
		x_w = msg.rigid_arm_pose_w[0];
		y_w = msg.rigid_arm_pose_w[1];
		z_w = msg.rigid_arm_pose_w[2];
		
		x_arm = x_w/100.0 + 0.0495;
		y_arm = -y_w/100.0 + 0.1425;
		z_arm = -z_w/100.0 + 0.057; 
	}

    // void callback_number(const std_msgs::Int64& msg) {
    //     counter += msg.data;
    //     std_msgs::Int64 new_msg;
    //     new_msg.data = counter;
    //     pub.publish(new_msg);
    // }
    // bool callback_reset_counter(std_srvs::SetBool::Request &req,
    //                             std_srvs::SetBool::Response &res)
    // {
    //     if (req.data) {
    //         counter = 0;
    //         res.success = true;
    //         res.message = "Counter has been successfully reset";
    //     }
    //     else {
    //         res.success = false;
    //         res.message = "Counter has not been reset";
    //     }
    //     return true;
    // }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "task_space_node");
    ros::NodeHandle nh;
    TaskSpace task = TaskSpace(&nh);
    ros::spin();
}
