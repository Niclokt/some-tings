// #include <ros/ros.h>
// #include <ros/console.h>
// #include <tf/tf.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

#define BASELINK_TO_CAMERA 2.2 // in [m]

using std::vector;
using std::cout;
using std::endl;

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


  vector<vector<double>> obstacles; // from /obstacles topic -- 3D LiDAR
	vector<vector<double>> obstacles_2; // from /obstacles2 topic -- 2D LiDAR
  vector<double> image_obstacle_x;

  vector<double> scan_obstacle_x, scan_obstacle_y; // extract x and y from /obstacles topic and /obstacles2 topic
  vector<double> lidar_theta, radius, camera_theta;
  vector<vector<double>> fused_obstacles;

  // ROS Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber obstacle2_sub;

  // ROS Publishers
  ros::Publisher fused_obstacles_pub;

  // Variables and Functions for subscribe to odom topic
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;

  //timer
	ros::Timer timer;

  // Private Methods / Functions
  void getPolarCoordinate();
  void getImageObstacleAngle();
  void compareLidarWithCamera();
  void addCameraObstacleData(int i, int j);
  void lidarObstacleDataOnly(int i);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);
  void obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);

  void mainTimerCallback();
};

//My Own Constructor
SensorFusion::SensorFusion() : tf_listener(tf_buffer)
{
  // Subscribe & Advertise

  // odom_topic 
  // obstacle_topic (Lidar obs sub)
  // detection_results (cam obs sub)
  // publish to another topic


	odom_sub = nh.subscribe(odom_topic_, 1, &SensorFusion::odomCallback, this);
  obstacle_sub = nh.subscribe(obstacle_topic_, 1, &SensorFusion::obstacleCallback, this);
  obstacle2_sub = nh.subscribe(obstacle2_topic_, 1, &SensorFusion::obstacle2Callback, this);
	//ROS_ASSERT (to assign constant values in launch file. No need to compile when changes are made to these values)
  ROS_ASSERT(private_nh.getParam("fusion_frequency", fusion_frequency_));
  ROS_ASSERT(private_nh.getParam("camera_fov", FOV_));
  ROS_ASSERT(private_nh.getParam("tolerance_for_comparison", tolerance));
  ROS_ASSERT(private_nh.getParam("image_dimensions", img_size));

  // Initialize the timer
	timer = nh.createTimer(ros::Duration(1.0/fusion_frequency_), &AgvPlanner::mainTimerCallback, this);
}

void SensorFusion::mainTimerCallback()
{
  // TODO: Main Logic Here
  ;
}

void SensorFusion::getPolarCoordinate()
{
  // Compute lidar obstacles into camera polar coordinate
  int i=0;
  for (i; i<scan_obstacle_x.size();i++)
  {
    double camera_x = car_x + BASELINK_TO_CAMERA * cos(car_yaw);
    double camera_y = car_y + BASELINK_TO_CAMERA * sin(car_yaw);
    double r = sqrt(pow(scan_obstacle_x[i] - camera_x, 2) + pow(scan_obstacle_y[i] - camera_y, 2));
    double scan_theta = atan2(scan_obstacle_y[i] - camera_y, scan_obstacle_x[i] - camera_x) - car_yaw;

    lidar_theta.push_back(scan_theta);
    radius.push_back(r);

    cout << "camera_x: " << camera_x << ", camera_y: " << camera_y 
          << ", r: " << r << ", scan_theta: " << scan_theta << endl;
  }
}

void SensorFusion::getImageObstacleAngle()
{
  int i=0;
  for (i; i<image_obstacle_x.size();i++)
  {
    double alpha = deg2rad(FOV_ / 2);
    double img_theta = atan(tan(alpha) * ((img_size[0]/ 2) - image_obstacle_x[i]) / (img_size[0]/ 2));
    camera_theta.push_back(img_theta);
    
    cout << "alpha: " << alpha << ", img_theta: " << img_theta << endl;
  }

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
  fused_obstacles.push_back({scan_obstacle_x[i], scan_obstacle_y[i], lidar_theta[i], radius[i]}); // combine camera data to lidar
  cout << fused_obstacles[i][0] << " " << fused_obstacles[i][1] << " " << fused_obstacles[i][2] << " " << fused_obstacles[i][3] << endl;
}

void SensorFusion::lidarObstacleDataOnly(int i)
{
  fused_obstacles.push_back({scan_obstacle_x[i], scan_obstacle_y[i], lidar_theta[i], radius[i]}); // include lidar data only
  cout << fused_obstacles[i][0] << " " << fused_obstacles[i][1] << " " << fused_obstacles[i][2] << " " << fused_obstacles[i][3] << endl;
}

void SensorFusion::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	geometry_msgs::TransformStamped transform_stamped;
	try
	{
	transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
	}
	catch (tf2::TransformException& ex)
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

void SensorFusion::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles.clear();

	for (int i =0; i < obstacle_msg->circles.size(); i++) 
	{
		double x = obstacle_msg->circles[i].center.x;
		double y = obstacle_msg->circles[i].center.y;
		double r = obstacle_msg->circles[i].radius;
		double vx = obstacle_msg->circles[i].velocity.x;
		double vy = obstacle_msg->circles[i].velocity.y;
		
    obstacles.push_back({x, y, r, vx, vy});

    scan_obstacle_x[i] = obstacles[i][0]; //check if order of array is correct
    scan_obstacle_y[i] = obstacles[i][1];
	}
	return;
}

//assuming this is camera data
void SensorFusion::obstacle2Callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
	 obstacles_2.clear();
	
	for (int i =0; i < obstacle_msg->circles.size(); i++) 
	{
		double x = obstacle_msg->circles[i].center.x;
		double y = obstacle_msg->circles[i].center.y;
		double r = obstacle_msg->circles[i].radius;
		double vx = obstacle_msg->circles[i].velocity.x;
		double vy = obstacle_msg->circles[i].velocity.y;
		obstacles_2.push_back({x, y, r, vx, vy});
    
    image_obstacle_x[i] = obstacles_2[i][0]; //check if order of array is correct

	return;
}

int main(int argc, char** argv)
{
	// ros::init(argc, argv, "sensor_fusion");
	// Construct a sensor fusion object
	SensorFusion sensor_fusion_obj;// = SensorFusion();
	// ros::spin(); //spin the ros node.
	return 0;
}