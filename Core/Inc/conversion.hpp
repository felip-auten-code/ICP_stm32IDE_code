
//ROS
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
//#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <math.h>

// Third Party
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.1415
// ICP
//#include "icp_cpp/conversion.h"

float Rad2Deg(const float rad) { return rad * 180.0 / PI; }

float Deg2Rad(const float deg) { return deg * PI / 180.0; }

Eigen::MatrixXd LaserScan360ToCartesianMatrix(sensor_msgs::LaserScan &scan){
	uint16_t ct=0;
	float ang, x, y;
	Eigen::MatrixXd m;
	for(int i =0; i<360;i++){
		if(scan.ranges[i]!= INFINITY && scan.ranges[i]!= -INFINITY){
			ang = Deg2Rad((float)i);
			x = scan.ranges[i]*cos(ang);
			y = scan.ranges[i]*sin(ang);
			ct++;
			m.resize(ct,2);
			m(ct-1, 0) = x;
			m(ct-1, 1) = y;
		}
	}

	return m;
}


Eigen::Matrix4d OdometryToMatrix(const nav_msgs::Odometry& odom) {
  Eigen::Matrix4d out = Eigen::MatrixXd::Identity(4, 4);
  double x = odom.pose.pose.position.x;
  double y = odom.pose.pose.position.y;
  double z = odom.pose.pose.position.z;

  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  Eigen::Quaterniond q (qw, qx, qy, qz);

  Eigen::Vector3d translation_vec(x, y, z);
  out.block<3,1>(0,3) = translation_vec;
  out.block<3,3>(0,0) = q.toRotationMatrix();
  return out;
}



