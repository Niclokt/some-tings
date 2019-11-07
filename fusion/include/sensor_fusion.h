#ifndef SENSOR_FUSION_H_
#define SENSOR_FUSION_H_

//#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>


class SensorFusion
{
 public:
  // Constructor
  SensorFusion();
  SensorFusion(double a, double b);
  // Destructor
  virtual ~SensorFusion();

  // Public Variables

  // Public Methods
  std::vector<double> getImgObsAngle(double fov, double img_size, double img_width);
  std::vector<double> addCamObsData(double ob_x, double ob_y, double lidar_theta, double radius);


 private:
  // Private Variables
  double a_;
  double b_;
  std::vector <std::vector<double> > fused_obstacles;


  // Private Functions
  constexpr double pi();
  double deg2rad(double angle);
  double rad2deg(double angle);
  std::vector<double> getPolarCoordinate(double ob_x, double ob_y, double car_x, double car_y, double car_yaw);
  
  vector<double> 




};



#endif // SENSOR_FUSION_H_