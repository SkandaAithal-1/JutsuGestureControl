#include <custom_header.hpp> //already includes ros/ros.h so no need to include again

// main function
int main(int argc, char **argv)
{
    //setup sort of part, all the initialisations of rosnode
    ros::init(argc, argv, "command_node");
    ros::NodeHandle command_nh("~");
    sasta_init(command_nh);
    wait4start();
    initialize_local_frame();
    takeoff(3);

    ros::Rate rate(3.0);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        if(check_waypoint_reached() == 1) 
        {
            break;
        }
    }
    
    stopped = false;
    forward();
    up();
    rotate();
    return 0;
}

