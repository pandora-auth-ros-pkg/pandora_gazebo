#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
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

int kfd = 0 ; 

struct termios cooked , raw ; 

class Teleoperation { 

  public: 
  
    Teleoperation ( double max_linear , double max_angular ) ; 
    ~Teleoperation ( void ) ; 
    
    double getLinearScale ( void ) ; 
    double getAngularScale ( void ) ; 
    
    void publishTwist ( void ) ; 
    
    void keyLoop ( void ) ; 

  private: 

    ros ::NodeHandle nh_ ; 
    
    double linear_ , linear_scale_ ; 
    double angular_ , angular_scale_ ; 
   
    ros ::Publisher twist_pub_ ; 
    
    boost ::thread pub_thread_ ; 
    boost ::mutex lock_ ; 
  
} ;

Teleoperation ::Teleoperation ( double max_linear = 0.5 , 
                                double max_angular = 0.8 ) : 
                                
  linear_ ( 0 ) , 
  angular_ ( 0 ) , 
  linear_scale_ ( max_linear ) , 
  angular_scale_ ( max_angular ) 
  
{

  nh_ .param ( "scale_linear" , linear_scale_ , linear_scale_ ) ; 
  nh_ .param ( "scale_angular" , angular_scale_ , angular_scale_ ) ; 

  twist_pub_ = nh_ .advertise < geometry_msgs ::Twist > ( "/cmd_vel" , 1 ) ; 
  
  pub_thread_  = boost ::thread ( & Teleoperation ::publishTwist , this ) ; 
  
}

Teleoperation ::~Teleoperation ( void ) { 

  pub_thread_ .join ( ) ; 
  
}

double Teleoperation ::getLinearScale ( void ) { 

  return linear_scale_ ; 
  
}

double Teleoperation ::getAngularScale ( void ) { 

  return angular_scale_ ; 
  
}

void Teleoperation ::publishTwist ( void ) { 

  geometry_msgs ::Twist twist ; 
  
  ros ::Rate rate ( 100 ) ; 
  
  while ( ros:: ok ) { 
      
    twist .linear .x = linear_ * linear_scale_ ; 
    twist .angular .z = angular_ * angular_scale_ ; 
    
    {

      boost ::mutex ::scoped_lock lock ( lock_ ) ; 
    
      twist_pub_ .publish ( twist ) ; 
    
    }
    
    rate .sleep ( ) ; 
    
  }

}

void Teleoperation ::keyLoop ( void ) { 

  char c ; 
  bool dirty = false ; 

  // get the console in raw mode                                                              
  tcgetattr ( kfd , & cooked ) ; 
  memcpy ( & raw , & cooked , sizeof ( struct termios ) ) ; 
  raw .c_lflag &=~ ( ICANON | ECHO ) ; 
  
  // Setting a new line, then end of file                         
  raw .c_cc [ VEOL ] = 1 ; 
  raw .c_cc [ VEOF ] = 2 ; 
  tcsetattr ( kfd , TCSANOW , & raw ) ; 

  puts ( "---------------------" ) ; 
  puts ( "Reading from keyboard" ) ; 
  puts ( "---------------------" ) ; 
  puts ( "\nUse arrow keys to move the robot:\n" ) ; 

  for ( ; ; ) { 
  
    // get the next event from the keyboard  
    if ( read ( kfd , & c , 1 ) < 0 ) { 
    
      perror ( "read():" ) ; 
      exit ( - 1 ) ; 
      
    }

    //~ linear_=angular_=0;
    ROS_DEBUG ( "value: 0x%02X\n" , c ) ; 
    
    switch ( c ) { 
    
      case KEYCODE_LEFT:
      
        ROS_DEBUG ( "LEFT" ) ; 
        
        angular_ += 0.1 ; 
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_A:
      
        ROS_DEBUG ( "LEFT_MAX" ) ; 
        
        angular_ = 1.0 ; 
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_RIGHT:
      
        ROS_DEBUG ( "RIGHT" ) ; 
        
        angular_ += - 0.1 ; 
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_D:
      
        ROS_DEBUG ( "RIGHT_MAX" ) ; 
        
        angular_ = - 1.0;
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_UP:
      
        ROS_DEBUG ( "UP" ) ; 
        
        linear_ += 0.2 ; 
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_W:
      
        ROS_DEBUG ( "UP_MAX" ) ; 
        
        linear_ = 1.0 ; 
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_DOWN:
      
        ROS_DEBUG ( "DOWN" ) ; 
        
        linear_ += - 0.2 ; 
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_S:
      
        ROS_DEBUG("DOWN_MAX");
        
        linear_ = - 1.0 ; 
        
        dirty = true ; 
        
        break ; 
        
      case KEYCODE_H:
      
        ROS_DEBUG ( "HALT" ) ; 
        
        linear_ = 0.0 ; 
        angular_ = 0.0 ; 
        
        dirty = true ; 
        
        break ; 
                
    } 
    
    if ( dirty == true ) { 
      
      if ( linear_ > 1.0 ) 
      
        linear_ = 1.0 ; 
      
      if ( linear_ < - 1.0 ) 
      
        linear_ = - 1.0 ; 
      
      if ( angular_ > 1.0 ) 
      
        angular_ = 1.0 ; 
      
      if ( angular_ < - 1.0 ) 
      
        angular_ = - 1.0 ; 
        
      std ::cout << "Linear  : " << linear_ * 100 << " \% , " 
                                 << linear_ * linear_scale_ << " m/s\n" ; 
      std ::cout << "Angular : " << angular_ * 100 << " \% , " 
                                 << angular_ * angular_scale_ << " r/s\n" ; 
      std ::cout << "########################" << '\n' ; 
      
      dirty = false ; 
      
    }
    
  }
  
  return ; 
  
}

void quit ( int sig ) { 

  tcsetattr ( kfd , TCSANOW , & cooked ) ; 
  ros ::shutdown ( ) ; 
  exit ( 0 ) ; 
  
}

int main ( int argc , char ** argv ) 

{ 

  ros ::init ( argc , argv , "teleoperation" ) ; 
  
  Teleoperation * teleoperation ; 
  
  if ( argc == 3 ) { 
  
    teleoperation = new Teleoperation ( atof ( argv [ 1 ] ) , 
                                        atof ( argv [ 2 ] ) ) ; 
    
  }
    
  else {
  
    teleoperation = new Teleoperation ( ) ; 
    
    std ::cout << "---------------------\n" 
               << "Maximum linear velocity was not specified, defaults to " 
               << teleoperation ->getLinearScale ( ) 
               << " m/s.\n" ; 
    
    std ::cout << "---------------------\n" 
               << "Maximum angular velocity was not specified, defaults to " 
               << teleoperation ->getAngularScale ( ) 
               << " r/s.\n" ; 
    
  }

  signal ( SIGINT , quit ) ; 

  teleoperation ->keyLoop ( ) ; 
  
  return ( 0 ) ; 
  
}
