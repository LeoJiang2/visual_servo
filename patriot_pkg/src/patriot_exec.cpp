#include <ros/ros.h>
#include "patriot_pkg/Patriot.hpp"
#include "patriot_pkg/arm_pose.h"
#include <algorithm>
#include <cmath>

class PatriotNode {
    private:
    
    int num_stations;
    int soft_station_num;
    int cur_hemisphere[3];
    float second_sensor_pose[7];
    float cur_soft_arm_pose[7];
    //#int master_axis;

 
    ros::Publisher arm_pose_pub;
    
    public:
    PatriotNode(ros::NodeHandle &nh) {
		nh.getParam("/soft_arm_tip_station", soft_station_num);
		Patriot PatriotDevice;
		set_hemisphere(PatriotDevice, 0, 0, -1); //Upper hemisphere
		num_stations = PatriotDevice.nstations;
        arm_pose_pub = nh.advertise<patriot_pkg::arm_pose>("patriot/arm_pose", 10);

        get_pose_loop(PatriotDevice);
    }
    

    template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

    void set_hemisphere(Patriot &device, int x, int y, int z){
		//#std::cout << "Set Hemisphere" << std::endl;
        device.set_hemisphere(x, y, z); 
        cur_hemisphere[0] = x;
        cur_hemisphere[1] = y;
        cur_hemisphere[2] = z;
        //#if(x != 0){
            //#master_axis = x;
        //#}else if(y != 0){
            //#master_axis = y*2;
        //#}else{
            //#master_axis = z*3;
        //#}
    }

/*
This is designed to select the best hemisphere for the magnetic sensor
It is not the smartest method.  A master axis is defined, which is the 
current axis of the hemisphere.  If that axis gets closer than 3cm by either
sensor, it changes the master axis.  The new axis is decided by the axis
with the greatest distance.
There might be a few corner cases, such as all less than 2 cm (should not be 
possible...).  Some cases might result in frequent switching.

*/
/*
    void check_hemisphere(Patriot &device){
		
		//#std::cout << "Hemisphere: " << cur_hemisphere[0] << ", " << cur_hemisphere[1] << ", " << cur_hemisphere[2] << std::endl;

        //#float rigid_test = cur_rigid_arm_pose[std::abs(master_axis) - 1];
        float soft_test = cur_soft_arm_pose[std::abs(master_axis) - 1];

        //#float test = std::min(std::abs(soft_test),std::abs(rigid_test));
        float test = std::abs(soft_test);

        if(test <2.0){
			//#std::cout << "Change of Hemisphere" << std::endl;
            // Switch to a new master axis
            float cur_max = -1;
            int cur_max_axis;
            for(int n=0;n<3;n++){
                //#rigid_test = cur_rigid_arm_pose[n];
                soft_test = cur_soft_arm_pose[n];
                //#test = std::min(std::abs(soft_test),std::abs(rigid_test));
                test = std::abs(soft_test);
                if(test > cur_max){
                    cur_max = test;
                    cur_max_axis = n;
                }

            }
            if(cur_max_axis == 0){ //x axis
                int sign_set = sgn(cur_soft_arm_pose[0]);
                set_hemisphere(device, sign_set*1, 0, 0);
            }else if(cur_max_axis == 1){ //y axis
                int sign_set = sgn(cur_soft_arm_pose[1]);
                set_hemisphere(device, 0, sign_set*1, 0);
            }else{ //z axis
                int sign_set = sgn(cur_soft_arm_pose[2]);
                set_hemisphere(device, 0, 0, sign_set*1);
            }

        }
 
    }
*/
    void get_pose_loop(Patriot &PatriotDevice){
		ros::Rate rate(20);
		while(ros::ok()) {
			patriot_pkg::arm_pose msg;
			for(int i=0;i < num_stations; i++){
			std::vector<float> pose = PatriotDevice.get_pose(i);
			// std::cout << pose[0] << std::endl;
            if(i == soft_station_num){

                for (int n=0;n<7;n++){
                    cur_soft_arm_pose[n] = pose[n];
                }
                //std::cout << "X: " << pose[0] << " Y: " << pose[1] << " Z: " << pose[2] << std::endl;
                msg.soft_arm_pose = pose;
            }else{
					//std::cout << "No sensor action given" << std::endl;
                for (int n=0;n<3;n++){
                    //#//msg.soft_arm_pose_w[n] = pose[n];
                    //cur_soft_arm_pose[n] = pose[n];
                    second_sensor_pose[n] = pose[n];
                }
                msg.second_sensor_pose = pose;
            }
            //#std::cout << "F" << std::endl;
		}
		//#msg.rigid_arm_pose_w = cur_rigid_arm_pose;
		//#msg.soft_arm_pose_w = cur_soft_arm_pose;
        arm_pose_pub.publish(msg);
        //#check_hemisphere(PatriotDevice);
		rate.sleep();
		}
		
		
	}

};


int main (int argc, char **argv)
{
    ros::init(argc, argv, "patriot_pkg_node");
    ros::NodeHandle nh;
    PatriotNode patriot = PatriotNode(nh);
    ros::spin();
}
