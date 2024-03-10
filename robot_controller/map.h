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
void update_camera_map(webots::GPS *gps, const float *lidar_image, webots::Camera *camera, float theta=0);
std::vector<std::pair<double, double>>& getLidarPoints();
std::vector<std::pair<double, double>>& getCameraPoints();
size_t getCount();
void clearPointCloud();

//thanks stack overflow
template <class T>
inline void hash_combine(std::size_t & s, const T & v)
{
    std::hash<T> h;
    s ^= h(v) + 0x9e3779b9 + (s<< 6) + (s>> 2);
}

struct POINT_TYPE {
  bool wall : 4;
  bool camera : 4;

  POINT_TYPE() : wall(false), camera(false) {}
};

struct REGION {
  //points
  std::map<std::pair<double,double>, POINT_TYPE> points;
};

REGION* get_region(webots::GPS *gps);

#endif