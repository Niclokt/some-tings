#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <obstacle_detector/Obstacles.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

#define BASELINK_TO_CAMERA 2.2 // in meters

using std::cout;
using std::endl;
using std::vector;


class ObstacleVisualisation 
{
public: 

//constructor
ObstacleVisualisation();

//Destructor

//Public Variables/ Methods

private:
//private Variables

//ROS subscriber 

//ROS publisher





};
