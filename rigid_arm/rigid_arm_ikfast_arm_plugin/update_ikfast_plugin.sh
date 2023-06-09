search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=rigid_arm.srdf
robot_name_in_srdf=rigid_arm
moveit_config_pkg=rigid_arm_moveit_config
robot_name=rigid_arm
planning_group_name=arm
ikfast_plugin_pkg=rigid_arm_ikfast_arm_plugin
base_link_name=base_link
eef_link_name=fake_link3
ikfast_output_path=/home/ben/moveit_ws/src/rigid_arm/rigid_arm_ikfast_arm_plugin/src/rigid_arm_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
