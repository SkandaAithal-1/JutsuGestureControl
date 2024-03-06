#include <gnc_functions.hpp> //already includes ros/ros.h so no need to include again
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

ros::Publisher vel_ctrl;
ros::Subscriber yaw_sub;
ros::Subscriber stop_sub;
float yaw;
bool stopped;

// call this function

// called whenever current_heading_g is updated; passing value for no reason
void yaw_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    yaw = (current_heading_g + local_offset_g)*M_PI/180;
}

// start/stop commands need to call this to start too
void stop_cb(const std_msgs::Bool::ConstPtr& msg)
{
    // set stopped state to received data
    stopped = (*msg).data;
}

void stop()
{
    geometry_msgs::TwistStamped cmd;
    cmd.twist.linear.x = 0;
    cmd.twist.linear.y = 0;
    cmd.twist.linear.z = 0;
    cmd.twist.angular.z = 0;
    for (int i = 0; i < 30; i++)
    {
        vel_ctrl.publish(cmd);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
}

void forward(float vel = 1, float t = 3)
{
    // basic definitions
    ros::Rate rate(3.0);
    ros::Time begin = ros::Time::now();
    geometry_msgs::TwistStamped cmd;
    std::cout << "Heading towards: " << yaw << std::endl;

    // constantly publishing vel to go forward for limitted time
    while(ros::Time::now().toSec() - begin.toSec() < t && ros::ok())
    {
        if(stopped)
        {
            ROS_INFO("Stopping before finish");
            break;
        }
        cmd.twist.linear.x = vel*cos(yaw);
        cmd.twist.linear.y = vel*sin(yaw);
        vel_ctrl.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
   
    // stop once moved enough
    stop();
}

void up(float vel = 1, float t = 3)
{
    // basic definitions
    ros::Rate rate(3.0);
    ros::Time begin = ros::Time::now();
    geometry_msgs::TwistStamped cmd;
    std::cout << "Heading upwards" << std::endl;

    // constantly publishing vel to go forward for limitted time
    while(ros::Time::now().toSec() - begin.toSec() < t && ros::ok())
    {
        if(stopped)
        {
            ROS_INFO("Stopping before finish");
            break;
        }
        cmd.twist.linear.z = vel;
        vel_ctrl.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
   
    // stop once moved enough
    stop();
}

void rotate(float vel = 1, float t = 3)
{
    // basic definitions
    ros::Rate rate(3.0);
    ros::Time begin = ros::Time::now();
    geometry_msgs::TwistStamped cmd;
    std::cout << "Rotating" << std::endl;

    // constantly publishing vel to go forward for limitted time
    while(ros::Time::now().toSec() - begin.toSec() < t && ros::ok())
    {
        if(stopped)
        {
            ROS_INFO("Stopping before finish");
            break;
        }
        cmd.twist.angular.z = vel;
        vel_ctrl.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
   
    // stop once moved enough
    stop();
}

void sasta_init(ros::NodeHandle controlnode)
{
    // using mehenga init first
    init_publisher_subscriber(controlnode);

    // making init for command velocity control
    std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{
		ROS_INFO("using default namespace");
	}
    else
    {
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}

	vel_ctrl = controlnode.advertise<geometry_msgs::TwistStamped>((ros_namespace + "/mavros/setpoint_velocity/cmd_vel").c_str(), 10);
    yaw_sub = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, yaw_cb);
    stop_sub = controlnode.subscribe<std_msgs::Bool>((ros_namespace + "/stop_topic").c_str(), 10, stop_cb);
}

