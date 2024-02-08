#include <drone_com/gnc_functions.hpp>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int16.h>

void unarm()
{
  
  set_destination(0,0,0,0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint_g);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	// arming
	ROS_INFO("Unarming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = false;
	while (current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
		return 0;
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}
}

void command(const std_msgs::Int16 &msg)
{
  if (msg.data == 1){
    takeoff(3);
  }
  else if (msg.data == 2){
    land();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "droneCommand");
  ros::NodeHandle droneCommand;

  init_publisher_subscriber(droneCommand);
  ros::Subscriber sub = droneCommand.subscribe("gesture", 10, command);
  // wait for flight controller to connect
  wait4connect();
  // wait used to switch to mode GUIDED
  wait4start();
  //crate local reference frame
  initialize_local_frame();

  ros::Rate rate(2.0);
  int counter=0;
  while(ros::ok())
  {
    ROS_INFO_STREAM("Subsciber is subscribing");
    ros::spinOnce();
    rate.sleep();
  }
}
