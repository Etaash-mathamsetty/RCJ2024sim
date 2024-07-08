#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <unordered_set>
#include <map>
#include <optional>
#include "helper.hpp"

#ifndef _MAP_H_
#define _MAP_H_

class RobotInstance;

void plotPoints(RobotInstance *rb, int w, int h);
void update_regions_map(RobotInstance *rb, const float *lidar_image, float theta=0);
pdd lidarToPoint(const pdd& pos, long double dist, long double absAngle);
std::vector<std::pair<double, double>>& getLidarPoints();
std::vector<std::pair<double, double>>& getBlackHoles();
std::vector<std::pair<double, double>>& getCameraPoints();
void addLidarPoint(const pdd& point);
void addBlackHolePoint(const pdd& point);
void addVictim(std::pair<double, double> point);
void reportVictim(std::pair<double, double> point);
bool notBeenDetected(pdd victim);
size_t getCount();
void clearPointCloud();
double clampAngle(double theta);
std::pair<pdd, pdd> get_lidar_minmax_opt();

struct POINT_TYPE {
  bool wall : 4;
  bool camera : 4;

  POINT_TYPE() : wall(false), camera(false) {}
};

inline bool nearly_equal(double a, double b, const double thresh = 0.02)
{
    return std::abs(a-b) <= thresh;
}

struct GPS_position
{
    double x;
    int y; //floor number
    double z;

    GPS_position(const double *pos)
    {
        x = *pos;
        y = *(pos+1);
        z = *(pos+2);
    }

    bool operator==(const GPS_position& other) const
    {
        return nearly_equal(other.x, x) && other.y == y && nearly_equal(other.z, z);
    }

    explicit operator std::string() const
    {
        return std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z);
    }
};

struct REGION {
  //points
  std::unordered_map<std::pair<double,double>, POINT_TYPE, pair_hash_combiner<double>> points;
};

REGION* get_region(webots::GPS *gps);
std::vector<REGION*> get_neighboring_regions(const std::pair<double, double>& pt, double radius = TRAVERSABLE_RADIUS);

#endif