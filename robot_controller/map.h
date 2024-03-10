#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <map>
#include <optional>

#ifndef _MAP_H_
#define _MAP_H_

void plotPoints(webots::GPS *gps, float theta, int w, int h, bool plot_regions=true);
void update_regions_map(webots::GPS *gps, const float *lidar_image, float theta=0);
void update_camera_map(webots::GPS *gps, const float *lidar_image, float theta=0, webots::Camera *camera);
std::vector<std::pair<double, double>> getLidarPoints();
std::vector<std::pair<double, double>> getCameraPoints();
size_t getCount();
void clearPointCloud();

struct REGION {
  //points
  std::map<std::pair<double,double>, bool> points;
};

REGION* get_region(webots::GPS *gps);

#endif