#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class tdmFinderJoy
{
 public:
  tdmFinderJoy()
  {
    ros::NodeHandle node;
    joy0_sub_ = node.subscribe("joy", 1, &tdmFinderJoy::joyCallback0,this);
    joy1_sub_ = node.subscribe("joy1", 1, &tdmFinderJoy::joyCallback1,this);
    twist_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
  }
  
  void joyCallback0(const sensor_msgs::Joy &joy_msg)
  {
    ROS_INFO("aa");
    if(joy_msg.buttons[0] == 1)
    {
      geometry_msgs::Twist twist;
      twist.linear.x = joy_msg.axes[1] * 0.5;
      twist.angular.z = joy_msg.axes[0] * 1.0;
      twist_pub_.publish(twist);
      std::cout<<"abc"<<std::endl;
    }else if (joy_msg.buttons[11] == 1){
      std::cout<<"Jog Move to Right"<<std::endl;
    }else if (joy_msg.buttons[13] == 1){
      std::cout<<"Jog Move to Left"<<std::endl;
    }else if (joy_msg.buttons[10] == 1){
      std::cout<<"Jog Move to Top"<<std::endl;
    }else if (joy_msg.buttons[12] == 1){
      std::cout<<"Jog Move to Bottom"<<std::endl;
    }else if (joy_msg.axes[3] == 1){
      std::cout<<"Step Move to Left"<<std::endl;
    }else if (joy_msg.axes[3] == -1){
      std::cout<<"Step Move to Right"<<std::endl;
    }else if (joy_msg.axes[4] == 1){
      std::cout<<"Step Move to Top"<<std::endl;
    }else if (joy_msg.axes[4] == -1){
      std::cout<<"Step Move to Bottom"<<std::endl;
    }
  }
 void joyCallback1(const sensor_msgs::Joy &joy_msg)
  {
    if(joy_msg.buttons[0] == 1)
    {
      std::cout<<"abc2"<<std::endl;
    }else if (joy_msg.buttons[11] == 1){
      std::cout<<"Jog2 Move to Right"<<std::endl;
    }else if (joy_msg.buttons[13] == 1){
      std::cout<<"Jog2 Move to Left"<<std::endl;
    }else if (joy_msg.buttons[10] == 1){
      std::cout<<"Jog2 Move to Top"<<std::endl;
    }else if (joy_msg.buttons[12] == 1){
      std::cout<<"Jog2 Move to Bottom"<<std::endl;
    }else if (joy_msg.axes[3] == 1){
      std::cout<<"Step2 Move to Left"<<std::endl;
    }else if (joy_msg.axes[3] == -1){
      std::cout<<"Step2 Move to Right"<<std::endl;
    }else if (joy_msg.axes[4] == 1){
      std::cout<<"Step2 Move to Top"<<std::endl;
    }else if (joy_msg.axes[4] == -1){
      std::cout<<"Step2 Move to Bottom"<<std::endl;
    }
  }
 
 private:
  ros::Subscriber joy0_sub_;
  ros::Subscriber joy1_sub_;
  ros::Publisher twist_pub_;
};

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"joy_twist");
  tdmFinderJoy joy_twist;
  ros::spin();
  return 0;
}

