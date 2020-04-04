#include "ros/ros.h"
#include <xarm_driver.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"



using namespace std;
// declare global messages
xarm_msgs::SetAxis set_axis_srv_;
xarm_msgs::SetInt16 set_int16_srv_;
xarm_msgs::Move move_srv_;
xarm_msgs::Move move_cart_;
// declare global services
ros::ServiceClient set_mode_client_;

// declare global variables
std::vector<float> desiredPose = {250, 100, 300, 3.14, 0, 0};
std::vector<float> currentPose = {250, 100, 300, 3.14, 0, 0};
float cmd_vel[3] = {0,0,0};
const float VEL_SCALE = 120;
int button_index_linear_velocity=0;
int button_index_free_drive=1;
bool system_initialized = false;

void twist_msg_CB(const geometry_msgs::Twist& msg)
{		
		cmd_vel[0] = VEL_SCALE*msg.linear.x;
		cmd_vel[1] = VEL_SCALE*msg.linear.y;
		cmd_vel[2] = VEL_SCALE*msg.linear.z;         
}
 
void current_pose_CB(const xarm_msgs::RobotMsg& msg)
{
  for (int i=0; i<3;i++)
  {
    currentPose[i] = msg.pose[i];
  }
}

void joy_msg_CB(const sensor_msgs::Joy& msg)
{
  // to do, parse a button
  if (msg.buttons[button_index_linear_velocity]==1)
  {
    set_int16_srv_.request.data = 1;
    set_mode_client_.call(set_int16_srv_);
    sleep(1);
    for(int i=0; i<3;i++)
    {
      desiredPose[i] = currentPose[i];
    }
  }
  if (msg.buttons[button_index_free_drive]==1)
  {
    set_int16_srv_.request.data = 2;
    set_mode_client_.call(set_int16_srv_);
    sleep(1);
  }
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "servo_test");
	ros::NodeHandle nh;
	int LOOP_RATE_HZ = 25;
	ros::Rate loop_rate(LOOP_RATE_HZ);
	float system_dt = 1.0/LOOP_RATE_HZ;
	
	ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, twist_msg_CB);
  	ros::Subscriber sub_current_pose = nh.subscribe("/xarm/xarm_states", 1, current_pose_CB);
	ros::ServiceClient motion_ctrl_client_ = nh.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
	// ros::ServiceClient set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
	set_mode_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
  ros::ServiceClient set_state_client_ = nh.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  	ros::ServiceClient go_home_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/go_home");
	ros::ServiceClient move_line_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_line");
	ros::ServiceClient move_servo_cart_client_ = nh.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart",true); 
    
    set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = 1;

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
 
    move_cart_.request.mvvelo = 0;
    move_cart_.request.mvacc = 0;
    move_cart_.request.mvtime = 0;
    move_cart_.request.mvradii = 0;

    system_initialized = true;

    while (true)
    {
   
	     for (int i = 0; i <3; i++){
				desiredPose[i] += system_dt*cmd_vel[i];
	     }
	     
            move_cart_.request.pose = desiredPose;
            std::cout << "Desired pos: ";
            std::cout << " x: ";             
            std::cout << desiredPose[0];
			      std::cout << ", y: ";             
            std::cout << desiredPose[1];
            std::cout << ", z: ";             
            std::cout << desiredPose[2];	
			std::cout << "  "; 			           
          if(move_servo_cart_client_.call(move_cart_))
          {
              ROS_INFO("%s\n", move_cart_.response.message.c_str());
          }
          else
          {
              ROS_ERROR("Failed to call service move_servo_cart");
					}
     ros::spinOnce();
     loop_rate.sleep();
   } 

    return 0;
}


