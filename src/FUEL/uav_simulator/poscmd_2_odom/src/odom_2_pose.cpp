#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Subscriber _odom_sub;
ros::Publisher  _pose_pub;

nav_msgs::Odometry _odom;
double _init_x, _init_y, _init_z;
bool  rcv_odom =false;

void rcvOdomCallBack(const nav_msgs::Odometry odom)
{	
	rcv_odom = true;
	_odom    = odom;
}

void pubOdom()
{	
	geometry_msgs::PoseStamped pose; 
	pose.header.stamp  = _odom.header.stamp;
	pose.header.frame_id = "/map";

	if(rcv_odom)
	{                                                                                           
        pose.pose = _odom.pose.pose;
		pose.pose.position=_odom.pose.pose.position;
		tf2::Quaternion quat_tf ,q2,qr;
		tf2::fromMsg(_odom.pose.pose.orientation, quat_tf);
		q2[0] = 0.5;
		q2[1] = -0.5;
		q2[2] = 0.5;
		q2[3] = -0.5;
		qr=quat_tf*q2;
		pose.pose = _odom.pose.pose;
		pose.pose.position=_odom.pose.pose.position;
		pose.pose.orientation = tf2::toMsg(qr);
		_pose_pub.publish(pose);
	}
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "pose_generator");
    ros::NodeHandle nh( "~" );
    nh.param("init_x", _init_x,  0.0);
    nh.param("init_y", _init_y,  0.0);
    nh.param("init_z", _init_z,  0.0);
    _odom_sub  = nh.subscribe( "odometry", 1, rcvOdomCallBack );
    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);                      

    ros::Rate rate(15);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom();                   
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}