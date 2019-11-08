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

// Functions (not methods)
constexpr double pi() { return M_PI; }
double deg2rad(double angle) { return angle * pi() / 180; }
double rad2deg(double angle) { return angle / pi() * 180; }

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
  const double FOV_ = 90; // in degrees

  double car_x;
  double car_y;
  double car_yaw;

  const vector<double> img_size = {1280, 720};

  vector<vector<double>> camera_pose; //Position of camera in world map [x, y]
  vector<vector<double>> obstacle_in_image; // position of obstacle in image [class_id, width, height, depth, centre x, centre y]
  vector<vector<double>> obstacle_wrtGlobal; //3D obstacle data wrt World Frame [class_id, width, height, obstacle_x, obstacle_y]

  
  // ROS Subscribers
  ros::Subscriber camera_obstacle_sub;
  ros::Subscriber camera_pose_sub;

  // ROS Publishers
  ros::Publisher deprojected_obstacle_pub;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  //timer
  ros::Timer timer;

  // Private Methods / Functions
  void deprojectImage();

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void cameraImageCallBack(const const std_msgs::Float64MultiArray::ConstPtr &obstacle_msg)

  void mainTimerCallback();
};

//My Own Constructor
Deprojection::Deprojection() : tf_listener(tf_buffer)
{

  // topics
	std::string odom_topic_;
  std::string deprojection_topic_;
	// std::string obstacle_topic_;
	// std::string obstacle2_topic_;
  std::string camera_obstacle_topic_;

  // ROS_ASSERT (to assign constant values in launch file. No need to compile when changes are made to these values)
  ROS_ASSERT(private_nh.getParam("fusion_frequency", fusion_frequency_)); //do we need a freq??
  ROS_ASSERT(private_nh.getParam("camera_fov", FOV_));
  // ROS_ASSERT(private_nh.getParam("tolerance_for_comparison", tolerance));
  
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
	// ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));       
	// ROS_ASSERT(private_nh.getParam("obstacle2_topic", obstacle2_topic_));
  ROS_ASSERT(private_nh.getParam("camera_obstacle_topic", camera_obstacle_topic_));

  // ROS_ASSERT(private_nh.getParam("image_dimensions", img_size));

  // Subscribe & Advertise
  odom_sub = nh.subscribe(odom_topic_, 1, &SensorFusion::odomCallback, this);
  camera_obstacle_sub = nh.subscribe(camera_obstacle_topic_, 1, &SensorFusion::cameraObstacleCallback, this);

  // publish to another topic
  deprojected_obstacle_pub = nh.advertise<obstacle_detector::Obstacles>(deprojection_topic_, 1);

  // Initialize the timer
  timer = nh.createTimer(ros::Duration(1.0 / fusion_frequency_), &SensorFusion::mainTimerCallback, this);
}

void SensorFusion::mainTimerCallback()
{
  ROS_DEBUG("timer start");

  // TODO: (07/11) Check if all required data are in position

  if ((obstacles.size() == 0) && (image_obstacle_center_x.size() == 0))
  {
    ROS_WARN("Empty");
    return;
  }

  // Call the methods
  deprojectImage();

  car_x.clear();
  car_y.clear();
  car_yaw.clear();
  camera_pose.clear();
  obstacle_in_image.clear();
}


////////////////////////////////////////CALLBACKS///////////////////////////////////
//Retrieve co-ordinates of buggy wrt Global Frame
void SensorFusion::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
  pose_before_transform.header.frame_id = odom_msg->header.frame_id;
  pose_before_transform.header.stamp = odom_msg->header.stamp;
  pose_before_transform.pose = odom_msg->pose.pose;
  tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

  tf::Quaternion q(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
                   pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, car_yaw);

  // Current XY of robot (map frame)
  car_x = pose_after_transform.pose.position.x;
  car_y = pose_after_transform.pose.position.y;
}

//Retrieve camera(ZED) detected obstacle data
void Deprojection::cameraImageCallBack()
{
  for (i=0; i < obstacle_msg->data.size(); i++)
  {
    double class_id = obstacle_msg->data[0];
    double radius = obstacle_msg->data[2];  //Find Radius of Obstacle
    double height = obstacle_msg->data[3];
    //double depth = obstacle_msg->data[];  //Find index no of depth info
    double center_x = obstacle_msg->data[4];
    double center_y = obstacle_msg->data[5];

    obstacle_in_image.pushback({class_id, radius, height, depth, center_x, center_y});
    //remember to find array index of depth first!
  }
  return;
} 

/////////////////////////////////////////METHODS////////////////////////////////////

//Calculate position of Camera with reference to World Map

void Deprojection::deprojectImage()
{
  for (i=0; i<obstacle_in_image.size(); i++)
  {
    //Find camera pose wrt global frame
    camera_pose[i][0] = car_x + BASELINK_TO_CAMERA * cos(car_yaw);  //camera_x
    camera_pose[i][1] = car_y + BASELINK_TO_CAMERA * sin(car_yaw);  //camera_y

    //Find delta x & y of obstacle from camera 
    double delta_x = obstacle_in_image[i][3] * cos(car_yaw);
    double delta_y = obstacle_in_image[i][4] * sin(car_yaw);

    //Find pose of obstacles wrt global frame
    double obstacle_x = camera_pose[i][0] + delta_x;
    double obstacle_y = camera_pose[i][1] + delta_y;

    //obstacle_in_image [class_id, radius, height, depth, center_x, center_y]
    //Save 3D obstacle data into 2D vector [class_id, radius, height, x, y]
    obstacle_wrtGlobal[i][0] = obstacle_in_image[0];  //class_id 
    obstacle_wrtGlobal[i][1] = obstacle_in_image[1];  //radius
    obstacle_wrtGlobal[i][2] = obstacle_in_image[2];  //height
    obstacle_wrtGlobal[i].push_back(obstacle_x);
    obstacle_wrtGlobal[i].push_back(obstacle_y);  //x, y
  }
  return;
}

void Deprojection::publishDeprojectionResult()
{
  obstacle_detector::Obstacles deprojection_msg;
  
  for (int i=0; i < obstacle_wrtGlobal.size(); i++)
  {
    deprojection_msg.circles[i].radius = obstacle_wrtGlobal[i][0];      //obstacle class_id --> TODO: Do we need to check class_id?
    deprojection_msg.circles[i].true_radius = obstacle_wrtGlobal[i][1]; //obstacle radius
    deprojection_msg.circles[i].center.x = obstacle_wrtGlobal[i][3];    //obstacle_x
    deprojection_msg.circles[i].center.y = obstacle_wrtGlobal[i][4];    //obstacle_y

    if (obstacle_wrtGlobal[i][0] != 0) 
    {
      deprojection_msg.circles[i].radius = obstacle_wrtGlobal.at(i).at(0); //safer way of accessing memory .at() function wont crash just give error
    }
    else 
    {
      deprojection_msg.circles[i].radius = 0;
    }
      
  }
  deprojected_obstacle_pub.publish(deprojection_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_fusion");
  // Construct a sensor fusion object
  Deprojection deprojecton_obj; // = SensorFusion();
  ros::spin();                    //spin the ros node.
  return 0;
}