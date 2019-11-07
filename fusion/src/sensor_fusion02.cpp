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

class SensorFusion
{
public:
  // Constructor
  SensorFusion();

  // Destructor
  virtual ~SensorFusion(){};

  // Public Variables

  // Public Methods

private:
  // Private Variables
  double fusion_frequency_;

  const double FOV_ = 78; // in degrees

  double tolerance = 0.035; // in radians
  double car_x;
  double car_y;
  double car_yaw;

  const vector<double> img_size = {640, 480};

  vector<vector<double>> obstacles;   // From /obstacles topic -- 3D LiDAR
  vector<vector<double>> obstacles_2; // From /obstacles2 topic -- 2D LiDAR, not using it at this stage (29/10/2019)
  vector<vector<double>> image_obstacles; 
  vector<double> image_obstacle_class, image_obstacle_confidence, image_obstacle_center_x, image_obstacle_width; // From /detection results topic
  vector<double> scan_obstacle_x, scan_obstacle_y, scan_obstacle_r, scan_obstacle_vx, scan_obstacle_vy; // Extract x, y, and radius from /obstacles topic and /obstacles2 topic. Not not using it at this stage (29/10/2019)
  
  vector<double> radius; // Distance from camera to obstacles
  vector<double> lidar_theta_center, lidar_theta_left, lidar_theta_right, camera_theta_center, camera_theta_left, camera_theta_right; // For comparison purpose
  vector<vector<double>> fused_obstacles;

  // ROS Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber obstacle2_sub;
  ros::Subscriber camera_obstacle_sub;

  // ROS Publishers
  ros::Publisher fused_obstacles_pub;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  //timer
  ros::Timer timer;

  // Private Methods / Functions
  // void fuse2DAnd3DObstacles(); Only use 3D LiDAR scan at this stage (29/10/2019)
  void getPolarCoordinate();
  void getImageObstacleAngle();
  void compareLidarWithCamera();
  void addCameraObstacleData(int i, int j);
  void lidarObstacleDataOnly(int i);

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr &obstacle_msg);
  void obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr &obstacle_msg);
  void cameraObstacleCallback(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msg);

  void mainTimerCallback();
};

//My Own Constructor
SensorFusion::SensorFusion() : tf_listener(tf_buffer)
{

  // topics
	std::string odom_topic_;
	std::string obstacle_topic_;
	std::string obstacle2_topic_;
  std::string camera_obstacle_topic_;
	std::string fusion_topic_;

  // ROS_ASSERT (to assign constant values in launch file. No need to compile when changes are made to these values)
  ROS_ASSERT(private_nh.getParam("fusion_frequency", fusion_frequency_));
  ROS_ASSERT(private_nh.getParam("camera_fov", FOV_));
  ROS_ASSERT(private_nh.getParam("tolerance_for_comparison", tolerance));
  
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
	ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));       
	ROS_ASSERT(private_nh.getParam("obstacle2_topic", obstacle2_topic_));
  ROS_ASSERT(private_nh.getParam("camera_obstacle_topic", camera_obstacle_topic_));

  // ROS_ASSERT(private_nh.getParam("image_dimensions", img_size));

  // Subscribe & Advertise
  odom_sub = nh.subscribe(odom_topic_, 1, &SensorFusion::odomCallback, this);
  obstacle_sub = nh.subscribe(obstacle_topic_, 1, &SensorFusion::obstacleCallback, this);
  obstacle2_sub = nh.subscribe(obstacle2_topic_, 1, &SensorFusion::obstacle2Callback, this);
  camera_obstacle_sub = nh.subscribe(camera_obstacle_topic_, 1, &SensorFusion::cameraObstacleCallback, this);

  // publish to another topic
  fused_obstacles_pub = nh.advertise<Float64MultiArray>(fusion_topic_, 1);

  // Initialize the timer
  timer = nh.createTimer(ros::Duration(1.0 / fusion_frequency_), &SensorFusion::mainTimerCallback, this);
}

void SensorFusion::mainTimerCallback()
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

  image_obstacle_class.clear();
  image_obstacle_confidence.clear();
  image_obstacle_center_x.clear();
  image_obstacle_width.clear();
  image_obstacles.clear();

}

/*
void SensorFusion::fuse2DAnd3DObstacles()
{
  int i, j; 
  // int count = 0;
  // i = obstacles.size(); 
  // j = obstacles2.size();

  // vector<double> helper[obstacles.size()] = {0}; 
  vector<double> checker[obstacles2.size()] = {0}; //To check if a certain obstacle has been matched or not
  vector<double> count[] = {0};

  for (i=0;i<obstacles.size();i++)
  {
    count.clear();
    for (j=0;j<obstacles2.size();j++)
    {
      if (checker[j] == 0)
      
    }
  }
  return;
}
*/

void SensorFusion::getPolarCoordinate()
{
  // Compute lidar obstacles into camera polar coordinate
  int i = 0;
  for (i; i < scan_obstacle_x.size(); i++)
  {
    double camera_x = car_x + BASELINK_TO_CAMERA * cos(car_yaw);
    double camera_y = car_y + BASELINK_TO_CAMERA * sin(car_yaw);
    double r = sqrt(pow(scan_obstacle_x[i] - camera_x, 2) + pow(scan_obstacle_y[i] - camera_y, 2)); //object radius from camera
    double scan_theta_center = atan2(scan_obstacle_y[i] - camera_y, scan_obstacle_x[i] - camera_x) - car_yaw;

    double alpha = deg2rad(90) - car_yaw;
    double scan_theta_left = atan2(scan_obstacle_y[i] + scan_obstacle_r[i] * sin(alpha) - camera_y, scan_obstacle_x[i] - scan_obstacle_r[i] * cos(alpha) - camera_x) - car_yaw;
    double scan_theta_right = atan2(scan_obstacle_y[i] - scan_obstacle_r[i] * sin(alpha) - camera_y, scan_obstacle_x[i] + scan_obstacle_r[i] * cos(alpha) - camera_x) - car_yaw;

    lidar_theta_center.push_back(scan_theta_center);
    lidar_theta_left.push_back(scan_theta_left);
    lidar_theta_right.push_back(scan_theta_right);
    radius.push_back(r);

    // cout << "camera_x: " << camera_x << ", camera_y: " << camera_y
    //      << ", r: " << r << ", scan_theta: " << scan_theta << endl;
  }
}

void SensorFusion::getImageObstacleAngle()
{
  int i = 0;
  for (i; i < image_obstacle_class.size(); i++)
  {
    double alpha = deg2rad(FOV_ / 2);
    double img_theta_center = atan(tan(alpha) * ((img_size[0] / 2) - image_obstacle_center_x[i]) / (img_size[0] / 2));
    double img_theta_left = atan(tan(alpha) * ((img_size[0] / 2) - image_obstacle_center_x[i] + image_obstacle_width[i] / 2) / (img_size[0] / 2));
    double img_theta_right = atan(tan(alpha) * ((img_size[0] / 2) - image_obstacle_center_x[i] - image_obstacle_width[i] / 2) / (img_size[0] / 2));

    camera_theta_center.push_back(img_theta_center);
    camera_theta_left.push_back(img_theta_left);
    camera_theta_right.push_back(img_theta_right);
    // cout << "alpha: " << alpha << ", img_theta_center: " << img_theta_center << endl;
  }
}

void SensorFusion::compareLidarWithCamera()
{
  fused_obstacles.clear();
  int flag = 0, smallest;

  vector<double> count[lidar_theta_center.size] = {0};
  vector<double> find_the_smallest[lidar_theta_center.size] = {0};

  for (int j = 0; j < camera_theta_center.size(); j++)
  {
    for (int i = 0; i < lidar_theta_center.size(); i++) // Calculate the cost
    {
      find_the_smallest.push_back(abs(lidar_theta_center[i] - camera_theta_center[j]) * abs(lidar_theta_left[i] - camera_theta_left[j]) * abs(lidar_theta_right[i] - camera_theta_right[j]))
    }

    // Since the camera is supposed to detect much less number of obstacles than LiDARs,
    // and every object detected by camera should be able to be detected by LiDARs,
    // here we assume there must exist an object detected by LiDAR that matches the camera
    smallest = find_the_smallest[0];            // Find the smallest cost
    for (i = 0; i < find_the_smallest.size(); i++)
    {
      if (find_the_smallest[i] < smallest)
      {
        smallest = find_the_smallest[i];
        flag = i;
      }
    }

    addCameraObstacleData(flag, j);           // Add camera data to lidar
    count[flag] = 1;                          // Mark this lidar data as matched

    flag = 0;
  }

  for (flag = 0; flag < count.size(); flag++) // For those unmatched LiDAR data
  {
    if (count[flag] == 0)
    {
      lidarObstacleDataOnly(flag);
    }
  }
}

void SensorFusion::addCameraObstacleData(int i, int j)
{
  fused_obstacles.push_back({obstacles[i], radius[i], image_obstacles[j]}); // combine camera data to lidar
  // cout << fused_obstacles[i][0] << " " << fused_obstacles[i][1] << " " << fused_obstacles[i][2] << " " << fused_obstacles[i][3] << endl;
}

void SensorFusion::lidarObstacleDataOnly(int i)
{
  fused_obstacles.push_back({obstacles[i], radius[i]});                     // include lidar data only
  // cout << fused_obstacles[i][0] << " " << fused_obstacles[i][1] << " " << fused_obstacles[i][2] << " " << fused_obstacles[i][3] << endl;
}

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

void SensorFusion::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr &obstacle_msg)
{
  obstacles.clear();
  scan_obstacle_x.clear();
  scan_obstacle_y.clear();
  scan_obstacle_r.clear();

  for (int i = 0; i < obstacle_msg->circles.size(); i++)
  {
    double x = obstacle_msg->circles[i].center.x;
    double y = obstacle_msg->circles[i].center.y;
    double r = obstacle_msg->circles[i].true_radius;
    double vx = obstacle_msg->circles[i].velocity.x;
    double vy = obstacle_msg->circles[i].velocity.y;

    obstacles.push_back({x, y, r, vx, vy});

    scan_obstacle_x[i] = obstacles[i][0];
    scan_obstacle_y[i] = obstacles[i][1];
    scan_obstacle_r[i] = obstacles[i][2];
    scan_obstacle_vx[i] = obstacles[i][3];
    scan_obstacle_vy[i] = obstacles[i][4];
  }
  return;
}

void SensorFusion::obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr &obstacle_msg)
{
  obstacles_2.clear();

  for (int i = 0; i < obstacle_msg->circles.size(); i++)
  {
    double x = obstacle_msg->circles[i].center.x;
    double y = obstacle_msg->circles[i].center.y;
    double r = obstacle_msg->circles[i].true_radius;
    double vx = obstacle_msg->circles[i].velocity.x;
    double vy = obstacle_msg->circles[i].velocity.y;

    obstacles_2.push_back({x, y, r, vx, vy});
  }
  return;
}

void SensorFusion::cameraObstacleCallback(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msg);
{
  // image_obstacle_class.clear();
  // image_obstacle_confidence.clear();
  // image_obstacle_center_x.clear();
  // image_obstacle_width.clear();
  // image_obstacles.clear();

  //for (int i = 0; i < obstacle_msg->data.size(); i++)
  //{
    double class_id = obstacle_msg->data[0];
    double confidence = obstacle_msg->data[1]];
    double width = obstacle_msg->data[2];
    double height = obstacle_msg->data[3];
    double center_x = obstacle_msg->data[4];
    double center_y = obstacle_msg->data[5];

    image_obstacles.push_back({class_id, confidence, width, height, center_x, center_y});

    image_obstacle_class.push_back(class_id);
    image_obstacle_confidence.push_back(confidence);
    image_obstacle_center_x.push_back(center_x);
    image_obstacle_width.push_back(width);
  //}
  return;
}

publishFusionResult()
{
  obstacle_detector::Obstacles fusion_msg;
  
  for (int i=0; i < fused_obstacles.size(); i++)
  {
    fusion_msg.circles[i].center.x = fused_obstacles[i][0];
    fusion_msg.circles[i].center.y = fused_obstacles[i][1];
    fusion_msg.circles[i].true_radius = fused_obstacles[i][2];
    fusion_msg.circles[i].velocity.x = fused_obstacles[i][3];
    fusion_msg.circles[i].velocity.y = fused_obstacles[i][4];

    if (fused_obstacles[i][6] != 0) fusion_msg.circles[i].radius = fused_obstacles[i][6]; // Data stored inside is actually class id
    else fusion_msg.circles[i].radius = 0;
  }
  

  fused_obstacles_pub.publish(fusion_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_fusion");
  // Construct a sensor fusion object
  SensorFusion sensor_fusion_obj; // = SensorFusion();
  ros::spin();                    //spin the ros node.
  return 0;
}