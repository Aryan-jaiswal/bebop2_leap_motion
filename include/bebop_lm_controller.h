#include "Leap.h"
#include <iostream>
#include <cstring>
#include "ros/ros.h"
#define thres 0.46
#define max_h 235
#define min_h 70

using namespace Leap;
class BebopListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
    // ROS publisher for Leap Motion Controller data
    ros::Publisher pub_;
    ros::Publisher toff;
    ros::Publisher land;
    ros::Publisher control;

  private:
	const std::string fingerNames[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
	const std::string boneNames[4] = {"Metacarpal", "Proximal", "Middle", "Distal"};
	const std::string stateNames[4] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};
};

