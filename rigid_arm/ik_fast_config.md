# Generating an IKFast Plug OF arm with "Transform6D"

sudo apt install ros-melodic-collada-urdf

export MYROBOT_NAME="rigid_arm"

## In robot description folder
rosrun xacro xacro -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME".urdf.xacro

## Move URDF to motion_palning directory (rigid_arm)
### Go to working director and source it if not already done
rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae

openrave-robot.py "$MYROBOT_NAME".dae --info links


## So many attempts to make this work...
name       index parents   
---------------------------
world      0               
ts_body    1     world     
base_link  2     ts_body   
link1      3     base_link 
link2      4     link1     
link3      5     link2     
fake_link1 6     link3     
fake_link2 7     fake_link1
fake_link3 8     fake_link2
ee_link    9     fake_link3
link5      10    ee_link   
---------------------------
name       index parents 

export PLANNING_GROUP="arm"
export BASE_LINK="2"
export EEF_LINK="8"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp


## Generate the C++ file - Should take a few minutes, but more than 10 means something is wrong
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=transform6d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --savefile="$IKFAST_OUTPUT_PATH"

## Alternate generation for 3D - requires only 3 joints, can't submit all 6 joints.

python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot="$MYROBOT_NAME".dae --iktype=translation3d --baselink="$BASE_LINK" --eelink="$EEF_LINK" --savefile="$IKFAST_OUTPUT_PATH"

Note that this excludes theta4

## Information about options for ik types
http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types

### Getting a segfault
Will get a segfault, but file is made due to edit seen here:
github.com/ros-planning/moveit_tutorials/issues/417
Note: This happened on the NUC, but not the laptop.  Maybe there was an error in the NUC install, as it was quite troublsome.


## Creating a plugin
export MOVEIT_IK_PLUGIN_PKG="$MYROBOT_NAME"_ikfast_"$PLANNING_GROUP"_plugin
cd ~/catkin_ws/src/rigid_arm   (If needed...)
catkin_create_pkg "$MOVEIT_IK_PLUGIN_PKG"

<!--catkin_create_pkg rigid_arm_ikfast_arm_plugin
cd ..
catkin build  #This brought up an error  catkin_make?? Doesn't seem to hurt.-->

### For transform6d
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" base_link fake_link3 "$IKFAST_OUTPUT_PATH"

### For translation3d
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" base_link link3 "$IKFAST_OUTPUT_PATH"
rosrun moveit_kinematics create_ikfast_moveit_plugin.py "$MYROBOT_NAME" "$PLANNING_GROUP" "$MOVEIT_IK_PLUGIN_PKG" base_link fake_link3 "$IKFAST_OUTPUT_PATH"


## return to working directory and rebuild or you will get an error about not finding kinematic library
catkin_make


## Edit CMakeLists.txt

# This was added to allow using IKFast with other packages
# Note there is some repetition, but this doesn't seem to cause issues

catkin_package(
  # Other pkgs can use ikfast.h
  INCLUDE_DIRS
    include
  LIBRARIES
    ik_solver_lib
)

add_library(ik_solver_lib src/rigid_arm_arm_ikfast_solver.cpp)
target_link_libraries(ik_solver_lib ${catkin_LIBRARIES})

# And include in installs
install(TARGETS ${IKFAST_LIBRARY_NAME} ik_solver_lib LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})



## Edit ikfast.h

// Needed to allow use of IKFast in other packages
#define IKFAST_HAS_LIBRARY



name       index parents   
---------------------------
world      0               
ts_body    1     world     
base_link  2     ts_body   
link1      3     base_link 
link2      4     link1     
link3      5     link2     
link4      6     link3     
fake_link1 7     link4     
fake_link2 8     fake_link1
ee_link    9     fake_link2
link5      10    ee_link   
---------------------------
name       index parents  

export BASE_LINK="2"
export EEF_LINK="8"

## For 3D
name      index parents  
-------------------------
world     0              
ts_body   1     world    
base_link 2     ts_body  
link1     3     base_link
link2     4     link1    
link3     5     link2    
link4     6     link3    
ee_link   7     link4    
link5     8     ee_link  
-------------------------
name      index parents  

export PLANNING_GROUP="arm"
export BASE_LINK="2"
export EEF_LINK="5"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp

## 3d attempt 2
name      index parents  
-------------------------
world     0              
ts_body   1     world    
base_link 2     ts_body  
link1     3     base_link
link2     4     link1    
link3     5     link2    
ee_link   6     link3    
link5     7     ee_link  
-------------------------
name      index parents  


export PLANNING_GROUP="arm"
export BASE_LINK="2"
export EEF_LINK="5"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61_"$PLANNING_GROUP".cpp


