#!/home/companion/tensorflow_ros/bin/python

import rospy
from berry_detection_mrcnn_pkg import util
from std_msgs.msg import String
from std_msgs.msg import Bool
# from berry_detection.msg import berry_pos
from geometry_msgs.msg import Point
from state_machine_pkg.srv import berry_detect, berry_detectResponse
from berry_detection_mrcnn_pkg import Kinematics as kin


class berry_detector:

    def __init__(self):
        # Starts a new node
        rospy.init_node('berry_detect', anonymous=True)
        self._kinematics = kin.Kinematics() # Create kinematics class

        # Create a publisher that publishes to /turtle1/cmd_vel
        # self._berry_point_publisher = rospy.Publisher('/berry_detector', berry_pos, queue_size=10)
        # self._berry_detector_call = rospy.Subscriber('/berry_detector_call', Bool, self.find_points)

        rospy.Service('state_machine_pkg/berry_detect', berry_detect, self._berry_detect_callback)


        # Specify publishing rate of 10
        self.rate = rospy.Rate(10)


#     # Find Berries in the image
#     def find_points(self, msg):
# ## FILL IN THE OBJECT DETECTION CODE OR INSERT THE ROS_WRAPPER - Added berry detection code in util##


#         # while not rospy.is_shutdown():
#         save_result = berry_pos()
#         points_output = util.find_berry_points()
#         # points_output =[[0.16, -0.45, 0.15]] #util.find_berry_points()
#         print(f'berry_detect_callback: {points_output} in mm')
#         num_detected = len(points_output)
#         for i in range(num_detected):
#             x = points_output[i][0]/1000.0
#             y = points_output[i][1]/1000.0
#             z = points_output[i][2]/1000.0
#             current_point = Point(x, y, z)
#             save_result.points.append(current_point)
#         self._berry_point_publisher.publish(save_result)
#         self.rate.sleep()


    # Find Berries in the image
    def _berry_detect_callback(self, msg):
## FILL IN THE OBJECT DETECTION CODE OR INSERT THE ROS_WRAPPER - Added berry detection code in util##


        # while not rospy.is_shutdown():
        save_result = []#berry_pos()
        points_output = util.find_berry_points()
        # points_output =[[0.16, -0.45, 0.15]] #util.find_berry_points()
        # points_output =[]
        print("Pan {} Tilt: {}".format(msg.camera_config_d[0], msg.camera_config_d[1]))
        print(f'berry_detect_callback: {points_output} in mm')
        num_detected = len(points_output)
        if (num_detected != 0):
            for i in range(num_detected):
                x = points_output[i][0]/1000.0
                y = points_output[i][1]/1000.0
                z = points_output[i][2]/1000.0
                point_base = self._kinematics._camera2arm_base([x,y,z], msg.camera_config_d[0], msg.camera_config_d[1])
                current_point = Point() # Point([point_base[0], point_base[1], point_base[2]])
                current_point.x = point_base[0]
                current_point.y = point_base[1]
                current_point.z = point_base[2]

                save_result.append(current_point)
        # else:
            # print("C")
            # current_point = Point()
            # current_point.x = 0
            # current_point.y = 0
            # current_point.z = 0
            # save_result.append(current_point)
            # save_result = []
        # self._berry_point_publisher.publish(save_result)
        # self.rate.sleep()
        # print("B")
        # temp = Point(0.4, 0.0, 0.5)
        # temp1 = Point(0.4, 0.1, 0.5)
        # temp2 = Point(0.45, 0.0, 0.55)
        # save_result = [temp, temp1, temp2]
        return berry_detectResponse(save_result)


if __name__ == '__main__':
    try:
        berry_detect_node = berry_detector()
       # berry_detect_node.find_points()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
