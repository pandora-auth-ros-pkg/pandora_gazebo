#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>

#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_H 'h'
#define KEYCODE_W 'w'
#define KEYCODE_A 'a'
#define KEYCODE_S 's'
#define KEYCODE_D 'd'

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(1),
  a_scale_(0.5)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    //~ linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
    
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        angular_ += 0.25;
        dirty = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("MAX_LEFT");
        angular_ = 2.0;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        angular_ += -0.25;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("MAX_RIGHT");
        angular_ = -2.0;
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("UP");
        linear_ += 0.25;
        dirty = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("MAX_UP");
        linear_ = 2.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("DOWN");
        linear_ += -0.25;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("MAX_DOWN");
        linear_ = -2.0;
        dirty = true;
        break;
      case KEYCODE_H:
        ROS_DEBUG("HALT");
        linear_ = 0.0;
        angular_= 0.0;
        dirty = true;
        break;
      default:
    std::cout <<std::hex <<  (int)c  << '\n';
        
        
    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}



