#include "ros/ros.h"
#include <xarm_driver.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_move_test");
	ros::NodeHandle nh;
	ros::Rate loop_rate(25);
	ros::ServiceClient motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
	ros::ServiceClient set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
	ros::ServiceClient set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  	ros::ServiceClient go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
	ros::ServiceClient move_line_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_line");
	ros::ServiceClient move_servo_cart_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart",true);

    xarm_msgs::SetAxis set_axis_srv_;
    xarm_msgs::SetInt16 set_int16_srv_;
    xarm_msgs::Move move_srv_;

    set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = 1;
    //motion_ctrl_client_.call(set_axis_srv_);
    


    if(motion_ctrl_client_.call(set_axis_srv_))
    {
        ROS_INFO("%s\n", set_axis_srv_.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service motion_ctrl");
        return 1;
    }


    set_int16_srv_.request.data = 0;
    set_mode_client_.call(set_int16_srv_);

    set_int16_srv_.request.data = 0;
    set_state_client_.call(set_int16_srv_);

    ros::Duration time(1);
    sleep(1);

    move_srv_.request.mvvelo = 20.0/57.0;
    move_srv_.request.mvacc = 1000;
    move_srv_.request.mvtime = 0;
    go_home_client_.call(move_srv_);

    std::vector<float> firstpose = {250, 100, 300, 3.14, 0, 0};
 
    move_srv_.request.mvvelo = 50;
    move_srv_.request.mvacc = 1000;
    move_srv_.request.mvtime = 0;
    move_srv_.request.mvradii = 0;
    move_srv_.request.pose = firstpose;
    move_line_client_.call(move_srv_);

    sleep(8);

    set_int16_srv_.request.data = 1;
    set_mode_client_.call(set_int16_srv_);

    set_int16_srv_.request.data = 0;
    set_state_client_.call(set_int16_srv_);
    sleep(1);
 
    move_srv_.request.mvvelo = 0;
    move_srv_.request.mvacc = 0;
    move_srv_.request.mvtime = 0;
    move_srv_.request.mvradii = 0;
    std::cout << firstpose[0];
    for (int i = 0; i <20; i++){
     firstpose[0] += 1;
     move_srv_.request.pose = firstpose;
     move_servo_cart_client_.call(move_srv_);
     std::cout << firstpose[0]<<"\n";
     ros::spinOnce();
     loop_rate.sleep();
     }
     return 0;
}


