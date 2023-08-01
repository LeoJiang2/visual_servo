# Terrasentia berry picking

Berry Detection with Intel RealSense and MRCNN:

The Intel RealSense camera provides depth information along with RGB images.
MRCNN processes the RGB image and gives an output of a mask.
By using MRCNN's detection output, the robot arm can get the approximate location of the berry in the camera frame.

Coordinate Transformation to World Frame:

Once the berry is detected in the camera frame, the robot arm needs to convert the pixel coordinates into the world frame.
This transformation involves using the depth information from the RealSense camera to calculate the 3D position of the berry in the world coordinate system.
With this transformation, the robot arm can estimate the position of the berry in its workspace.

Image-Based Visual Servoing (IBVS):
The robot arm needs to use its tip-mounted camera to keep track of the berry's position in real time.
The IBVS algorithm processes the image from the tip camera and calculates the visual error, which is the difference between the desired position (the detected berry's location) and the current position of the berry in the tip camera frame.
The IBVS system then generates control signals that direct the robot arm's movements to minimize the visual error and guide the arm towards the berry.
This process is iterative and is continuously performed until the berry is aligned with the desired position.
Control and Arm Movement:

Based on the control signals from the IBVS system, the robot arm's controller adjusts the arm's joint angles to move it closer to the berry.
The arm's end effector (e.g., gripper) can be maneuvered using inverse kinematics or other motion planning techniques to reach the berry's position accurately.
