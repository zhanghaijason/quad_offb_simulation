#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "pid.hpp"
#include "control_mapping.h"
using namespace Eigen;
mavros_msgs::State current_state;
double x_t, y_t, z_t, Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Ki_x, Ki_y, Ki_z;
float a_zf = 0.0; //feedback linear acceleration in z direction

Eigen::Vector3d mavPos_;

Eigen::Vector3d toEigen(const geometry_msgs::Point&p){
    Eigen::Vector3d ev3(p.x, p.y, p.z);
    return ev3;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& c_pose){   //retrive position feedback
    mavPos_ = toEigen(c_pose->pose.position);
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr & l_acce){
    a_zf = l_acce->linear_acceleration.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");  // have to define a private handle to get parameters from launch file
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
  	    ("mavros/state", 1, state_cb);
    ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 1, &mavposeCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber az_sub = nh.subscribe("/mavros/imu/data", 1, &IMUCallback ,ros::TransportHints().tcpNoDelay());
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	nhPrivate.getParam("target_x", x_t);
	nhPrivate.getParam("target_y", y_t);
	nhPrivate.getParam("target_z", z_t);
	nhPrivate.getParam("controller_px", Kp_x);
	nhPrivate.getParam("controller_py", Kp_y);
	nhPrivate.getParam("controller_pz", Kp_z);
	nhPrivate.getParam("controller_dx", Kd_x);
	nhPrivate.getParam("controller_dy", Kd_y);
	nhPrivate.getParam("controller_dz", Kd_z);
	nhPrivate.getParam("controller_ix", Ki_x);
	nhPrivate.getParam("controller_iy", Ki_y);
	nhPrivate.getParam("controller_iz", Ki_z);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
   // pose.pose.position.x = 0;
   // pose.pose.position.y = 0;
   // pose.pose.orientation.z = 2;
    

     
    mavros_msgs::AttitudeTarget att_msg;

    PID controller_x(Kp_x,Kd_x,Ki_x,-100,100,-100,100, "x_controller");
    PID controller_y(Kp_y,Kd_y,Ki_y,-100,100,-100,100, "y_controller");
    PID controller_z(Kp_z,Kd_z,Ki_z,-100,100,-100,100, "z_controller");
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

     //   local_pos_pub.publish(pose);
	att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
	float u_x = controller_x.update(mavPos_[0], x_t);
	float u_y = controller_y.update(mavPos_[1], y_t);
	float u_z = controller_z.update(mavPos_[2], z_t);
	//std::cout << "COntroller parameter: " << Kp_z << ", " << Kd_z <<std::endl;
	std::cout <<"Controller u: "<< u_x << "," << u_y << "," << u_z <<std::endl;
	vector<float> tem_res = control_mapping(u_x, u_y, u_z);	
	tf2::Quaternion goal;
	std::cout << "Mapping result: "<< tem_res[1]<< ", " << tem_res[2] << std::endl;
	goal.setRPY(tem_res[1],tem_res[2],0);
	att_msg.orientation.x = goal[0];
	att_msg.orientation.y = goal[1];
	att_msg.orientation.z = goal[2];
	att_msg.orientation.w = goal[3];
	att_msg.thrust = tem_res[0];
	att_pub.publish(att_msg);
	std::cout << "Feedback: " << mavPos_(0)<< ", " << mavPos_(1) <<", "<< mavPos_(2) <<std::endl;
        std::cout << "Z acceleration: " << a_zf << std::endl;
	ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
