This package generates three subscribers that help calculate the 
forward/inverse position kinematics and inverse velocity kinematics for a RRR
robot as shown below -

![Alt text](https://github.com/WaliaRohan/WaliaRohan/blob/main/fk/RRR.jpg "RRR Robot Diagram")

The subscribers subscribe to the following topics - 

1) transform_fk - takes a geometry_msgs/Vector3 message containing the joint angle values to calculate end effector position
2) transform_ik - takes a geometry_msgs/Pose message containing end-effector position to calculate joint angle values
3) transform_vel - takes a geometry_msgs/Twist message containing end-effector velocity values to calculate joint velocities  

Assumed link lengths: 1 unit for each link

Assumed values for calculating the Jacobian for velocity kinematics - 
  1. Joint angles: 0, pi/4, pi/4
  2. Link lengths: 1, 1, 1

To run the nodes in the package, call the node in this package using following command (after running roscore) - 

**rosrun fk <topic_name>**

where <topic_name> is transform_fk, transform_ik or transform_vel

