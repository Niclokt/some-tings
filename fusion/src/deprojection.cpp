#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>


#include <obstacle_detector/Obstacles.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

#define BASELINK_TO_CAMERA 2.2 // in meters

using std::cout;
using std::endl;
using std::vector;

class Deprojection
{
public:
  // Constructor
  Deprojection();

  // Destructor
  virtual ~Deprojection(){};

  // Public Variables

  // Public Methods

private:
  // Private Variables
    const double FOV_ = 78; // in degrees

  const vector<double> img_size = {1280, 720};

  vector<vector<double>> rviz_obstacles; // obstacle data to  publish to rviz
  vector<vector<double>> img_obs; // Obstacle data from ImageReader [0-x, 1-y, 2-radius]
  vector<double> zed_obs; // Obstacle data from ZED - Depth


  // ROS Subscribers
  ros::Subscriber image_sub;
 
  // ROS Publishers
  ros::Publisher rviz_obstacles_pub;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  //timer
  ros::Timer timer;

 
  void getObsDataFromImage();
  void getDepthFromZED();

  void twoDeeToThreeDee();
  void imageCallBack();
  void zedCallBack();

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  
  void mainTimerCallback();
};

//My Own Constructor
Deprojection::Deprojection() : tf_listener(tf_buffer)
{

  // topics
	std::string image_topic_; //(??) Do I name the topic myself? Or is there an actual name for this? 
	std::string zed_topic_; //(??) Do I name the topic myself?
  
  std::string obstacle_topic_;
	
  // ROS_ASSERT (to assign constant values in launch file. No need to compile when changes are made to these values)
  ROS_ASSERT(private_nh.getParam("camera_fov", FOV_));
  ROS_ASSERT(private_nh.getParam("image_resolution", img_size));
  
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
	ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));       

  // ROS_ASSERT(private_nh.getParam("image_dimensions", img_size));

  // Subscribe & Advertise
  image_sub = nh.subscribe(image_topic_, 1, &Deprojection::imageCallBack, this);
  zed_sub = nh.subscribe(zed_topic_, 1, &Deprojection::zedCallBack, this);
  obstacle_sub = nh.subscribe(obstacle_topic_, 1, &SensorFusion::obstacleCallBack, this);


  // publish to another topic
  fused_obstacles_pub = nh.advertise<Float64MultiArray>(fusion_topic_, 1);

  // Initialize the timer
  timer = nh.createTimer(ros::Duration(1.0 / fusion_frequency_), &SensorFusion::mainTimerCallback, this);
}

void Deprojection::mainTimerCallback()
{
  ROS_DEBUG("timer start");

  // Check if all required data are in position

  if ((obstacles.size() == 0) && (image_obstacle_center_x.size() == 0))
  {
    ROS_WARN("Empty");
    return;
  }

  // Call the methods
  getPolarCoordinate();
  getImageObstacleAngle();
  compareLidarWithCamera();
  
  // TODO: Write clear code for deprojection
  // image_obstacle_class.clear();
  // image_obstacle_confidence.clear();
  // image_obstacle_center_x.clear();
  // image_obstacle_width.clear();
  // image_obstacles.clear();

}

void Deprojection::imageCallBack()
{

}

void Deprojection::zedCallBack()
{

}

void Deprojection::twoDeeToThreeDee()
{
  for(i = 0; i<image_scan.size(); i++) //image_scan refers to 2d array of 2d info
  {
    double threeDeePoint =     

  } 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_fusion");
  // Construct a sensor fusion object
  SensorFusion sensor_fusion_obj; // = SensorFusion();
  ros::spin();                    //spin the ros node.
  return 0;
}