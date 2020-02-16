#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
float z_f = 0;
float v_zf = 0;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void state_position(const geometry_msgs::PoseStamped::ConstPtr& c_pose){
    //ROS_INFO("Feedback position: x: %.4f, y: %.4f, z: %.4f", c_pose->pose.position.x, c_pose->pose.position.y, c_pose->pose.position.z);
	z_f = c_pose->pose.position.z;
}

void velCallback(const geometry_msgs::TwistStamped::ConstPtr& vel){
	v_zf = vel->twist.linear.z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber p_f = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, state_position);  /// feedback position
    ros::Subscriber v_f = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 1, velCallback); //velocity_body is not on mavros wiki, but in rostopic list 
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 4;

	geometry_msgs::Twist velocity_target;
	velocity_target.linear.x = 0;
	velocity_target.linear.y = 0;
    velocity_target.linear.z = 1.5;
	velocity_target.angular.x = 0;
	velocity_target.angular.y = 0;
	velocity_target.angular.z = 0;

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
		if(z_f <= 4){
			velocity_pub.publish(velocity_target);
			std::cout << "Velocity feedback: "<< v_zf << std::endl;
		}
		else
    		    	local_pos_pub.publish(pose);
		std::cout << "Position feedback: " << z_f <<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

