/**
 * @file RRR_kinematics.cpp
 * @author Rohan Walia (walia.rohan@gmail.com)
 * @brief Creates three subscribers that implement forward position
 *        kinematics, inverse position kinematics, and inverse
 *        velocity kinematics for 3 DOF RRR robot.
 * @version 0.1
 * @date 2021-04-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h" // user input for joint variables
#include "geometry_msgs/Pose.h" // user input for pose
#include "geometry_msgs/Quaternion.h" // extracting orientation from pose
#include "geometry_msgs/Point.h" // extracting point from pose
#include "geometry_msgs/Twist.h" // extracting velocity
#include <math.h> //for calculating transformation matrix
#include <vector> //for storing transformation matrix
#include <string> //for printing vector
#include <Eigen/Dense> // for performing vector operations

using namespace Eigen;

/**
 * @brief Prints a vector of float vectors
 * 
 * @param vect vector of float vectors
 */
void printVector(std::vector<std::vector<float>> vect)
{
	std::string str = "\nTransformation matrix - \n";
	
	for (int i = 0; i < vect.size(); i++) { 
        for (int j = 0; j < vect[i].size(); j++) 
            {
				str = str + std::to_string(vect[i][j]) + " ";
			} 
			str = str + "\n"; 
	}
	ROS_INFO_STREAM(str);
}

/**
 * @brief Callback function for accepting user input for joint variable
 *        values and calculating final transformation matrix. This function
 *        also calls the helper function 'printVector' to print calculated
 *        matrix.
 * 
 * @param msg Vector3 messaging containing <x, y, z> location of end-effector
 */
void pos_fk_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  ROS_INFO(" \n Received joint variable values -> q1: [%.3f] q2: [%.3f] q3: [%.3f]",
   msg->x, msg->y, msg->z);
  
  float q1, q2, q3, l1, l2, l3;
  
  l1 = 1; l2 = 1; l3 = 1;
  
  q1 = msg->x;
  q2 = msg->y;
  q3 = msg->z;
  
  float r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34,
		r41, r42, r43, r44;

	r11 = cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3);
	r12 = -cos(q1)*cos(q2)*sin(q3) - cos(q1)*sin(q2)*cos(q3);
	r13 = sin(q1);
	r14 = l2*cos(q1)*cos(q2) + l3*cos(q1)*cos(q2)*cos(q3) - l3*cos(q1)*sin(q2)*sin(q3);
	
	r21 = sin(q1)*cos(q2)*cos(q3) - sin(q1)*sin(q2)*sin(q3);
	r22 = -sin(q1)*cos(q2)*sin(q3) - sin(q1)*sin(q2)*cos(q3);
	r23 = -cos(q1);
	r24 = l2*sin(q1)*cos(q2) + l3*sin(q1)*cos(q2)*cos(q3) - l3*sin(q1)*sin(q2)*sin(q3);
	
	r31 = cos(q2)*sin(q3) + sin(q2)*cos(q3);
	r32 = cos(q2)*cos(q3) - sin(q2)*sin(q3);
	r33 = 0;
	r34 = l1 + l2*sin(q2) + l3*cos(q2)*sin(q3) + l3*sin(q2)*cos(q3);	
	
	r41 = 0;
	r42 = 0;
	r43 = 0;
	r44 = 1;
  
  std::vector <std::vector <float>> vect{ 
							{r11, r12, r13, r14},
							{r21, r22, r23, r24},
							{r31, r32, r33, r34},
							{r41, r42, r43, r44} };  
							
	printVector(vect);
}

/**
 * @brief Function for claculating inverse position kinematics for the robot.
 * 
 * @param msg message containing position (<x, y, z>) of the end-effector
 */
void pos_ik_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
	geometry_msgs::Point p = msg->position;
	geometry_msgs::Quaternion q = msg->orientation;
	
	ROS_INFO(" \n Received point values -> x: [%.3f] y: [%.3f] z: [%.3f]",
    p.x, p.y, p.z);
   
    ROS_INFO(" \n Received oreintation values -> x: [%.3f] y: [%.3f] z: [%.3f] w[%.3f]",
    q.x, q.y, q.z, q.w);
	
	float Xc = p.x, Yc = p.y, Zc = p.z;
   
   	float d1 = 1, d2 = 1, d3 = 1; //joint variables and link lengths
   
   	float t1 = atan2(Yc, Xc);
   
   	float r = sqrt(pow(Xc, 2) + pow(Yc, 2));
	
   	float t3 = acos(pow(d2, 2) +  pow(d3, 2) - pow(r, 2) - pow((Zc - d1), 2));
   
   	float t2 = atan2(Zc - d1, r) - atan2(d3*sin(t3), d2 + d3*cos(t3));
   
   	std::string str = "\nJoint variables-> q1: " + std::to_string(t1) +
   	" q2: " + std::to_string(t2) + " q3: " + std::to_string(t3) + "\n";
   
   	ROS_INFO_STREAM(str);
}

/**
 * @brief function for calculating the joint velocities based on
 * 		  end-effector velocity
 * 
 * @param msg message containing end-effector velocity (<v_x, v_y, v_z>)
 */
void vel_ik_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Vector3 velocity = msg->linear;
	
	std::string str1 = "\nReceived velocities-> Vx: " 
   + std::to_string(velocity.x) + " Vy: " 
   + std::to_string(velocity.y) + " Vz: "
   + std::to_string(velocity.z);
   
   std::cout << str1 << std::endl;
   
	// Jacobian constructed assuming the following values - 
	// theta1 = 0, theta2 = pi/4, theta3 = pi/4
	// d1 = 1, d2 = 1, d3 = 1
	
	float t1, t2, t3; float d1, d2, d3;
	
	//t1 = 0; t2 = M_PI/4; t3 = M_PI/4; d1 = 1; d2 = 1; d3 = 1;
	
	t1 = 0; t2 = 0; t3 = 0; d1 = 1; d2 = 1; d3 = 1;
	
	std::string str2 = "\nAssumed joint values-> q1: " 
   + std::to_string(t1) + " q2: " 
   + std::to_string(t2) + " q3: "
   + std::to_string(t3);
   
   std::cout << str2 << std::endl;
   
   std::string str3 = "\nAssumed link lengths-> d1: " 
   + std::to_string(d1) + " d2: " 
   + std::to_string(d2) + " d3: "
   + std::to_string(d3);
   
   std::cout << str3 << std::endl;
	
	Vector3d z0(0, 0, 1);
	Vector3d z1(sin(t1), -cos(t1), 0);
	Vector3d z2(sin(t1), -cos(t1), 0);
	
	Vector3d O0(0, 0, 0);
	Vector3d O1(0, 0, d1);
	Vector3d O2(d2*cos(t1)*cos(t2), d2*cos(t2)*sin(t1), (d1+d2*sin(t2)));
	
	float O3_x, O3_y, O3_z;
	
	O3_x = d2*cos(t1)*cos(t2) + d3*cos(t1)*cos(t2)*cos(t3) - 
	d3*cos(t1)*sin(t2)*sin(t3);
	
	O3_y = d2*cos(t2)*sin(t1) + d3*cos(t2)*cos(t3)*sin(t1) -
	d3*sin(t1)*sin(t2)*sin(t3);
	
	O3_z = d1 + d2*sin(t2) + d3*cos(t2)*sin(t3) + d3*cos(t3)*sin(t2);
	
	Vector3d O3(O3_x, O3_y, O3_z);
	
	Vector3d v1 = z0.cross(O3 - O0);
	Vector3d v2 = z1.cross(O3 - O1);
	Vector3d v3 = z2.cross(O3 - O2);
	
	Matrix3d j;
	
	j << v1,
		 v2,
		 v3;

	std::cout << "Jacobian -" << std::endl << std::endl <<
	j << std::endl << std::endl;
	
	Matrix3d inv_j = j.inverse();
	
	std::cout << "Jacobian inverse -" << std::endl << std::endl << 
	inv_j << std::endl << std::endl;
	
	Vector3d vel(velocity.x, velocity.y, velocity.z);
	
	std::cout << "Joint Velocities -" << std::endl << std::endl << 
	inv_j*vel << std::endl << std::endl;
}

/**
 * @brief The main function for this node
 * 
 * @param argc 0
 * @param argv function does not take any values
 * @return int 0 fir normal exit code
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "fkSpace");

  ros::NodeHandle n;

  ros::Subscriber forward_kinematics = n.subscribe("transform_fk", 1000, pos_fk_callback);
  ros::Subscriber inverse_kinematics = n.subscribe("transform_ik", 1000, pos_ik_callback);S
  ros::Subscriber velocity_kinematics = n.subscribe("transform_vel", 1000, vel_ik_callback);

  ros::spin();

  return 0;
}
