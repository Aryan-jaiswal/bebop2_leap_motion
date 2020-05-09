# Bebop Controller Using Leap Motion

It is a hand-gesture based bebop2 controller which has been built using leap motion. (https://www.leapmotion.com). It currently supports all the basic functionalities to manoeuvre the drone in a safe environment.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

1. Download the Leap Motion V2 Desktop SDK.(https://developer-archive.leapmotion.com/downloads/external/skeletal-beta/linux?version=2.3.1.31549)

2. The downloaded archive also contains a folder named LeapSDK. Copy this LeapSDK folder to any desired location, e.g. your home folder 

3. CMakeLists.txt expects to find LeapSDK at $LEAP_SDK, so add, for example, the following line to your .bashrc file:

	`export LEAP_SDK=~/LeapSDK`

4. Install ros-kinetic
 
5. Clone this repository to your catkin workspace and compile using catkin_make or catkin build.

6. Install bebop_autonomy to connect to the drone.

### Installing

1. To install bebop_autonomy, follow : (https://bebop-autonomy.readthedocs.io/en/latest/installation.html)

2. To install ros-kinetic, follow : (http://wiki.ros.org/ROS/Installation)

## Running the tests

1. Start the leap motion daemon :
	`leapd`
	
2. Connect your system to the drone.

3. Run the following command
	 
	 `rosrun leap_motion_controller leap_motion`

## Contributing

Feel free to contribute to this code, by sending in pull requests.



