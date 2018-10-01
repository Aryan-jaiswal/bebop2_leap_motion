#include "bebop_lm_controller.h"
#include "leap_motion_controller/Set.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#define thres 0.46


using namespace Leap;

namespace leap_motion_controller
{
  ros::NodeHandle* nhPtr;
}

void BebopListener::onInit(const Controller& controller) {
  std::cout << "Leap Motion Initialized" << std::endl;
}

void BebopListener::onConnect(const Controller& controller) {
  std::cout << "Leap Motion Connected" << std::endl;
  ///////////////////////////////////////////////////
  ///We can configure each gesture accordingly//////
  /////////////////////////////////////////////////
  //controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  //controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  //controller.enableGesture(Gesture::TYPE_SWIPE);
}
void BebopListener::onDisconnect(const Controller& controller) {
  
  std::cout << "Leap Motion Disconnected" << std::endl;
}

void BebopListener::onExit(const Controller& controller) {
  
  std::cout << "Bye Bye!" << std::endl;
}
void BebopListener::onFocusGained(const Controller& controller) {
  
  std::cout << "Focus Gained" << std::endl;
}

void BebopListener::onFocusLost(const Controller& controller) {
  
  std::cout << "Focus Lost" << std::endl;
}
void BebopListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void BebopListener::onServiceConnect(const Controller& controller) {

  std::cout << "Service Connected" << std::endl;
}

void BebopListener::onServiceDisconnect(const Controller& controller) {
 
  std::cout << "Service Disconnected" << std::endl;
}
//All the compuations are done in this function//
void BebopListener::onFrame(const Controller& controller) {
	
  const Frame frame = controller.frame();
	// std::cout << "Frame id: " << frame.id()
 //            << ", timestamp: " << frame.timestamp()
 //            << ", hands: " << frame.hands().count()
 //            << ", extended fingers: " << frame.fingers().extended().count()
 //            << ", tools: " << frame.tools().count()
 //            << ", gestures: " << frame.gestures().count() << std::endl;
  
  HandList hands = frame.hands();

  leap_motion_controller::Set ros_msg;
  geometry_msgs::Twist sting;
  std_msgs::Empty emp;


  ros_msg.header.stamp = ros::Time::now();

   // Set ROS frame_id in ros_msg.header
  ros_msg.header.frame_id = "leap_motion";

  if(hands.count() >= 2 ){
    land.publish(emp);
    ROS_INFO("Successfully Landed!!");
    sleep(10);
    //ROS_INFO("There are more than one hand! Don't mess with the controls!");
  }
  else if(hands.count() == 0)
    ROS_INFO("To initiate control mechanism introduce a hand in the Frame of Reference!");
   else {
    const Hand hand = hands[0];
    std::string handType = hand.isLeft()?"Left":"Right";
    if(handType == "Left")
      ROS_INFO("Life without Left Hand wouldn't be right! ");
    else
    {
      // NOTE: Leap Motion roll-pitch-yaw is from the perspective of human, so I am mapping it so that roll is about x-, pitch about y-, and yaw about z-axis.
      float roll = hand.palmNormal().roll();
      float pitch = hand.direction().pitch();
      float yaw = -hand.direction().yaw();      // Negating to comply with the right hand rule.
      if(hand.pinchStrength()>0.85)
        {
          toff.publish(emp);
          ROS_INFO("Bebop Udd Gaya!!");
          sleep(5);
          //system( "rostopic pub /bebop/land std_msgs/Empty");

        }
        std::cout<<roll<<std::endl;
        std::cout<<pitch<<std::endl;
        std::cout<<yaw<<std::endl;
      if(pitch > thres)
      {
        sting.linear.x = -0.2;
        // ROS_INFO("+Ry ");
        // std::cout<<roll<<std::endl;
      }
      else if(pitch < -thres )
      {
        sting.linear.x = 0.2;
        // ROS_INFO("-Ry");
        // std::cout<<roll<<std::endl;
      }
      if(roll > thres)
      {
        sting.linear.y = 0.2;
        // ROS_INFO("+Px");
        // std::cout<<pitch<<std::endl;
      }
      else if(roll < -thres)
      {
        sting.linear.y = -0.2;
        // ROS_INFO("-Px");
        // std::cout<<pitch<<std::endl;
      }

    }

  }

  const GestureList gestures = frame.gestures();
  for (int g = 0; g < gestures.count(); ++g) {
    Gesture gesture = gestures[g];

    switch (gesture.type()) {
      
      case Gesture::TYPE_CIRCLE:
      {
        CircleGesture circle = gesture;
        //perform actions
        break;
      }
      case Gesture::TYPE_SWIPE:
      {
        SwipeGesture swipe = gesture;

        break;
      }
       case Gesture::TYPE_KEY_TAP:
      {
        KeyTapGesture tap = gesture;
        // sleep(10);
        
        break;
      }
       case Gesture::TYPE_SCREEN_TAP:
      {
        ScreenTapGesture screentap = gesture;


        break;
      }
      default:
      {
        std::cout << std::string(2, ' ')  << "Unknown gesture type." << std::endl;
        break;
      }
    }
  }
  if (!frame.hands().isEmpty() || !gestures.isEmpty()) {
    std::cout << std::endl;  
  }
  // Publish the ROS message based on this Leap Motion Controller frame.
  pub_.publish( ros_msg );
  control.publish(sting);

  // Throttle the loop
  ros::Duration(0.1).sleep();

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "leap_motion");
	// Create a sample listener and controller
  BebopListener listener;
  Controller controller;
  ros::NodeHandle nh;
  leap_motion_controller::nhPtr = &nh; // Allow other functions to interact with the nodehandle via this pointer

  //Add a Ros Publisher
  listener.pub_ = nh.advertise<leap_motion_controller::Set>("leap_motion_output", 10);
  listener.toff = nh.advertise<std_msgs::Empty>("/takeoff",1);
  listener.land = nh.advertise<std_msgs::Empty>("/land",1);
  listener.control = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

  // Have the listener receive events from the controller
  controller.addListener(listener);

  ros::spin();

  controller.removeListener(listener);

  return 0;
  }
