#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

#define BASELINK_TO_CAMERA 2.2 // in [m]

using namespace std;

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
  
  const double FOV_ = 78; // in degrees

  double tolerance = 0.035; // in radians
  double car_x;
  double car_y;
  double car_yaw;
  double image_obstacle_x;

  const vector<double> img_size = {640, 480};

  vector<double> obstacle_x, obstacle_y;
  vector<double> lidar_theta, radius, camera_theta;
  vector <vector<double> > fused_obstacles;

  // ROS Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber obstacles_sub;
  ros::Subscriber camera_sub;

  // ROS Publishers
  ros::Publisher fused_obstacles_pub;

  // Variables and Functions for subscribe to odom topic
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;

  // Private Functions
  constexpr double pi();
  double deg2rad(double angle);
  double rad2deg(double angle);

  // Private Methods
  void getPolarCoordinate(double ob_x, double ob_y);
  void getImageObstacleAngle();
  void compareLidarWithCamera();
  void addCameraObstacleData(int i, int j);
  void lidarObstacleDataOnly(int i);
};

//My Own Constructor
SensorFusion::SensorFusion() : tf_listener(tf_buffer)
{
  // Subscribe & Advertise
	polarCoord_sub = nh.subscribe(polarCoord_topic_, 1, &SensorFusion::polarCoordCallback, this);
  imgObAngle_sub = nh.subscribe(imgObAngle_topic_, 1, &SensorFusion::imgObAngleCallback, this);

	//ROS_ASSERT (to assign constant values in launch file. No need to compile when changes are made to these values)
  ROS_ASSERT(private_nh.getParam("camera_fov", FOV_));
  ROS_ASSERT(private_nh.getParam("tolerance_for_comparison", tolerance));
  ROS_ASSERT(private_nh.getParam("image_dimensions", img_size));
}

constexpr double pi() { return M_PI; }
double deg2rad(double angle) { return angle * pi() / 180; }
double rad2deg(double angle) { return angle / pi() * 180; }

void SensorFusion::getPolarCoordinate(double ob_x, double ob_y)
{
  // Compute lidar obstacles into camera polar coordinate
  double camera_x = car_x + BASELINK_TO_CAMERA * cos(car_yaw);
  double camera_y = car_y + BASELINK_TO_CAMERA * sin(car_yaw);
  double r = sqrt(pow(ob_x - camera_x, 2) + pow(ob_y - camera_y, 2));
  double scan_theta = atan2(ob_y - camera_y, ob_x - camera_x) - car_yaw;

  lidar_theta.push_back(scan_theta);
  radius.push_back(r);

  cout << "camera_x: " << camera_x << ", camera_y: " << camera_y 
       << ", r: " << r << ", scan_theta: " << scan_theta << endl;
}

void SensorFusion::getImageObstacleAngle()
{
  double alpha = deg2rad(FOV_ / 2);
  double img_theta = atan(tan(alpha) * ((img_size[0]/ 2) - image_obstacle_x) / (img_size[0]/ 2));

  camera_theta.push_back(img_theta);
  
  cout << "alpha: " << alpha << ", img_theta: " << img_theta << endl;
}

void SensorFusion::compareLidarWithCamera()
{
  // int k = 0;
  bool flag = 0;
  int i = 0; // index for lidar
  int j = 0; // index for camera

  for ( i; i < lidar_theta.size(); i++)
    {
        for ( j; j < camera_theta.size(); j++)
        {
            if (abs(lidar_theta[i] - camera_theta[j]) <= tolerance)
            {
                addCameraObstacleData(i, j); // to add camera data to lidar
                // k++;
                flag = 1;
                break;
            }
        }

        if (flag == 0)
        {
            lidarObstacleDataOnly(i);
            // k++;
        }

        flag = 0;
    }
}

void SensorFusion::addCameraObstacleData(int i, int j)
{
  fused_obstacles.push_back({obstacle_x[i], obstacle_y[i], lidar_theta[i], radius[i]}); // add camera data to lidar
  cout << fused_obstacles[i][0] << " " << fused_obstacles[i][1] << " " << fused_obstacles[i][2] << " " << fused_obstacles[i][3] << endl;
}

void SensorFusion::lidarObstacleDataOnly(int i)
{
  fused_obstacles.push_back({obstacle_x[i], obstacle_y[i], lidar_theta[i], radius[i]}); // include lidar data only
  cout << fused_obstacles[i][0] << " " << fused_obstacles[i][1] << " " << fused_obstacles[i][2] << " " << fused_obstacles[i][3] << endl;
}

int main(int argc, char** argv)
{
	// ros::init(argc, argv, "sensor_fusion");
	// Construct a sensor fusion object
	SensorFusion sensor_fusion_obj;// = SensorFusion();
	// ros::spin(); //spin the ros node.
	return 0;
}