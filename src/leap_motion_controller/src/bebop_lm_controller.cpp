#include "bebop_lm_controller.h"

using namespace Leap;

void BebopListener::onInit(const Controller& controller) {
  std::cout << "Leap Motion Initialized" << std::endl;
}

void BebopListener::onConnect(const Controller& controller) {
  std::cout << "Leap Motion Connected" << std::endl;
  ///////////////////////////////////////////////////
  ///We can configure each gesture accordingly//////
  /////////////////////////////////////////////////
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
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

void BebopListener::onServiceConnect(const Controller& controller) {

  std::cout << "Service Connected" << std::endl;
}

void BebopListener::onServiceDisconnect(const Controller& controller) {
 
  std::cout << "Service Disconnected" << std::endl;
}
//All the compuations are done in this function//
void BebopListener::onFrame(const Controller& controller) {
	const Frame frame = controller.frame();
	std::cout << "Frame id: " << frame.id()
            << ", timestamp: " << frame.timestamp()
            << ", hands: " << frame.hands().count()
            << ", extended fingers: " << frame.fingers().extended().count()
            << ", tools: " << frame.tools().count()
            << ", gestures: " << frame.gestures().count() << std::endl;

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "leap_motion");
	// Create a sample listener and controller
	BebopListener listener;
  	Controller controller;
  	ros::NodeHandle nh;
  	//Add a Ros Publisher
  	 listener.ros_publisher_ = nh.advertise<leap_motion_controller::Set>("leap_motion_output", 10);

  	// Have the listener receive events from the controller
  	controller.addListener(listener);

  	ros::spin();

  	controller.removeListener(listener);

  	return 0;
  }
