#include "../include/sensor_fusion.h"

#define BASELINK_TO_CAMERA 2.2 // in [m]


//using std::cout;
//using std::endl;
//using std::max;
//using std::vector;

// Constructor
SensorFusion::SensorFusion(){};


// Destructor
SensorFusion::~SensorFusion() {};

constexpr double pi() { return M_PI; }
double deg2rad(double angle) 
{ 
    return angle * pi() / 180; 
}
double rad2deg(double angle) { return angle / pi() * 180; }

std::vector<double> SensorFusion::getPolarCoordinate(double ob_x, double ob_y, double car_x, double car_y, double car_yaw)
{
    // Compute lidar obstacles into camera polar coordinate
    double camera_x = car_x + BASELINK_TO_CAMERA * cos(car_yaw);
    double camera_y = car_y + BASELINK_TO_CAMERA * sin(car_yaw);
    double r = sqrt(pow(ob_x - camera_x, 2) + pow(ob_y - camera_y, 2));

    double theta = atan2(ob_y - camera_y, ob_x - camera_x) - car_yaw;

    std::cout << "camera_x: " << camera_x << ", camera_y: " << camera_y
         << ", r: " << r << ", theta: " << theta << std::endl;
    
    return {r, theta};
}

std::vector<double> SensorFusion::getImgObsAngle(double fov, double img_size, double img_width)
{
    // Get image obstacle's thteta angle
  double alpha = deg2rad(fov / 2);

  double img_theta = atan(tan(alpha) * ((img_size/ 2) - img_width) / (img_size/ 2));
  camera_theta.push_back(img_theta);
  
  std::cout << "alpha: " << alpha << ", img_theta: " << img_theta << std::endl;
}

std::vector<double> SensorFusion::addCamObsData(double ob_x, double ob_y, double lidar_theta, double radius)
{
    // Add Camera Data
  fused_obstacles.push_back({ob_x[i], ob__y[i], lidar_theta[i], radius[i]}); // to include camera data
  std::cout << fused_obstacles[k][0] << " " << fused_obstacles[k][1] << " " << fused_obstacles[k][2] << " " << fused_obstacles[k][3] << std::endl;
}

// ################################################################

int main()
{
    SensorFusion fusion_object = SensorFusion();

    // Camera Properties
    double fov = 78; //in deg
    const std::vector<double> img_shape = {640, 480};
    double tolerance = 0.035; // in radians
    int k = 0;
    bool flag = 0;

    // double x, y, current_x, current_y, current_yaw, l;
    double camera_x, camera_y, r, theta;

    std::vector<double> radius;
    std::vector<double> x = {3, 3};
    std::vector<double> y = {5, 4};
    std::vector<double> current_x = {0, 2};
    std::vector<double> current_y = {0, 1};
    std::vector<double> current_yaw = {M_PI / 2, -M_PI / 2};
    std::vector<double> lidar_theta, camera_theta;
    

    std::vector<double> img_x = {500, 29};
    std::vector<double> img_y = {};

    for (int i = 0; i < x.size(); i++)
    {
        // Compute lidar obstacles into camera polar coordinate
        std::vector<double> polar = fusion_object.getPolarCoordinate(x[i], y[i], current_x[i], current_y[i], current_yaw[i]);
    }

    for (int i = 0; i < img_x.size(); i++)
    {
        // TODO: compute image obstacles' theta angle
       std::vector<double> theta = fusion_object.getImgObsAngle(fov, img_shape, img_x[i]);
    }

    for (int i = 0; i < lidar_theta.size(); i++)
    {
        for (int j = 0; j < camera_theta.size(); j++)
        {
            if (abs(lidar_theta[i] - camera_theta[j]) <= tolerance)
            {
                fusion_object.addCamObsData(x[i], y[i], lidar_theta[i], radius[i]);
                k++;
                flag = 1;
                break;
            }
        }

        if (flag == 0)
        {
            fused_obstacles.push_back({x[i], y[i], lidar_angle[i], r[i]}); // to include camera data
            std::cout << fused_obstacles[k][0] << " " << fused_obstacles[k][1] << " " << fused_obstacles[k][2] << " " << fused_obstacles[k][3] << std::endl;
            k++;
        }

        flag = 0;
    }

    return 0;
}