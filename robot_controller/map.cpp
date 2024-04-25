#include "imgui/imgui.h"
#include "imgui/implot.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <optional>
#include "map.h"
#include "constants.h"
#include "RobotInstance.hpp"

#define pdd std::pair<double, double>
#define f first
#define s second

using namespace webots;

const double region_size = 0.1;

inline bool nearly_equal(double a, double b, const double thresh = 0.02)
{
    return std::abs(a-b) <= thresh;
}

inline double round_to(double value, const double precision = 0.01)
{
    return std::round(value / precision) * precision;
}

inline double floor_to(double value, const double precision = 0.01)
{
    return std::floor(value / precision) * precision;
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

};

struct GPS_hash
{
    std::size_t operator()(const GPS_position& pos) const
    {
        std::size_t res = 0;
        hash_combine(res, pos.x);
        hash_combine(res, pos.y);
        hash_combine(res, pos.z);
        return res;
    }
};

std::unordered_map<GPS_position, REGION, GPS_hash> regions;
std::vector<std::pair<double, double>> vecLidarPoints;
std::vector<std::pair<double, double>> vecCameraPoints;

//theta is in radians
void update_regions_map(GPS *gps, const float *lidar_image, float theta)
{
    vecLidarPoints.reserve(20000);

    for(int i = 0; i < 512; i++)
    {
        float dist = lidar_image[i];
        if(std::isinf(dist))
            continue;
        const float angle = i/512.0 * 2.0 * M_PI;
        double pos[3];
        double pos_rounded[3];
        memcpy(pos, gps->getValues(), 3 * sizeof(double));
        pos[2] *= -1;
        //force single floor for now
        pos[1] = 0;
        pos_rounded[0] = floor_to(pos[0], region_size);
        pos_rounded[1] = floor_to(pos[1], region_size);
        pos_rounded[2] = floor_to(pos[2], region_size);
        double x = dist*sin(angle - theta) + (pos[0] - pos_rounded[0]);
        double y = dist*cos(angle - theta) + (pos[2] - pos_rounded[2]);

        x = floor_to(x);
        y = floor_to(y);

        while(x < 0)
        {
            x += region_size;
            pos_rounded[0] -= region_size;
        }

        while(y < 0)
        {
            y += region_size;
            pos_rounded[2] -= region_size;
        }

        while(y >= region_size)
        {
            y -= region_size;
            pos_rounded[2] += region_size;
        }

        while(x >= region_size)
        {
            x -= region_size;
            pos_rounded[0] += region_size;
        }

        const auto coord = std::make_pair(x,y);
        //std::cout << "x: " << x << " y: " << y << std::endl;
        const auto coord2 = std::make_pair(x + pos_rounded[0], y + pos_rounded[2]);
        if(regions[GPS_position(pos_rounded)].points.count(coord) == 0 || !regions[GPS_position(pos_rounded)].points[coord].wall)
        {
            vecLidarPoints.push_back(coord2);
            regions[GPS_position(pos_rounded)].points[coord].wall = true;
        }
    }
}

double clampAngle(double theta)
{
    if (abs(theta) > M_PI)
    {
        while (abs(theta) > M_PI)
        {
            theta -= copysign(2, theta) * M_PI;
        }
    }
    return theta;
}

bool isBetween(double theta, double start, double end)
{
    end -= start;
    theta -= start;
    if (theta < 0)
    {
        theta += 2 * M_PI;
    }
    if (end < 0)
    {
        end += 2 * M_PI;
    }
    return theta <= end;
}

void update_camera_map(GPS* gps, const float* lidar_image, Camera* camera, float theta)
{
    double fov = CAMERA_FOV; //radians

    double leftAngle = clampAngle(theta - M_PI / 2);
    double rightAngle = clampAngle(theta + M_PI / 2);

    //sweeping counterclockwise
    pdd leftEndpoints = pdd(clampAngle(leftAngle - fov / 2),
        clampAngle(leftAngle + fov / 2));
    pdd rightEndpoints = pdd(clampAngle(rightAngle - fov / 2),
        clampAngle(rightAngle + fov / 2));

    vecCameraPoints.reserve(20000);

    for (int i = 0; i < 512; i++)
    {
        float dist = lidar_image[i];
        if (std::isinf(dist))
            continue;
        const float angle = clampAngle(i / 512.0 * 2.0 * M_PI);
        if (!isBetween(angle - theta, leftEndpoints.f, leftEndpoints.s) &&
            !isBetween(angle - theta, rightEndpoints.f, rightEndpoints.s))
            continue;
        if (dist > MAX_VIC_DETECTION_RANGE)
            continue;
        double pos[3];
        double pos_rounded[3];
        memcpy(pos, gps->getValues(), 3 * sizeof(double));
        pos[2] *= -1;
        //force single floor for now
        pos[1] = 0;
        pos_rounded[0] = floor_to(pos[0], region_size);
        pos_rounded[1] = floor_to(pos[1], region_size);
        pos_rounded[2] = floor_to(pos[2], region_size);
        double x = dist * sin(angle - theta) + (pos[0] - pos_rounded[0]);
        double y = dist * cos(angle - theta) + (pos[2] - pos_rounded[2]);

        x = floor_to(x);
        y = floor_to(y);

        while (x < 0)
        {
            x += region_size;
            pos_rounded[0] -= region_size;
        }

        while (y < 0)
        {
            y += region_size;
            pos_rounded[2] -= region_size;
        }

        while (y >= region_size)
        {
            y -= region_size;
            pos_rounded[2] += region_size;
        }

        while (x >= region_size)
        {
            x -= region_size;
            pos_rounded[0] += region_size;
        }

        const auto coord = std::make_pair(x, y);
        //cout << "x: " << x << " y: " << y << endl;
        const auto coord2 = std::make_pair(x + pos_rounded[0], y + pos_rounded[2]);
        if (regions[GPS_position(pos_rounded)].points.count(coord) == 0 || !regions[GPS_position(pos_rounded)].points[coord].camera)
        {
            vecCameraPoints.push_back(coord2);
            regions[GPS_position(pos_rounded)].points[coord].camera = true;
        }
    }
}

std::vector<std::pair<double, double>>& getLidarPoints()
{
    return vecLidarPoints;
}

std::vector<std::pair<double, double>>& getCameraPoints()
{
    return vecCameraPoints;
}

void addLidarPoint(pdd point)
{
    vecLidarPoints.push_back(point);
}

ImPlotPoint getPointFromMap(int idx, void *_map)
{
    return ImPlotPoint(vecLidarPoints[idx].first, vecLidarPoints[idx].second);
}

ImPlotPoint getCameraPointFromMap(int idx, void *_map)
{
    return ImPlotPoint(vecCameraPoints[idx].first, vecCameraPoints[idx].second);
}

ImPlotPoint getVisitedPlotPt(int idx, void *param)
{
    auto it = getVisited().begin();
    std::advance(it, idx);
    return ImPlotPoint(it->first, it->second);
}

size_t getCount()
{
    return vecLidarPoints.size();
}

size_t getCameraCount()
{
    return vecCameraPoints.size();
}

void clearPointCloud()
{
    regions.clear();
    vecLidarPoints.clear();
    vecCameraPoints.clear();
}

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif



void plotPoints(RobotInstance *rb, int w, int h)
{
    webots::GPS *gps = rb->getGPS();
    double theta = rb->getIMU()->getRollPitchYaw()[2];
    double pos[3];
    memcpy(pos, gps->getValues(), 3 * sizeof(double));
    pos[2] *= -1;

    ImVec2 GraphSize = ImVec2(w-50, h-100);

    if(regions.size() > 0 && ImPlot::BeginPlot("Debug View", GraphSize))
    {
        ImPlot::SetupLegend(ImPlotLocation_North, ImPlotLegendFlags_Outside | ImPlotLegendFlags_Horizontal);
        ImPlot::SetNextLineStyle(ImVec4(0,0.4,1.0,1));
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Asterisk, 3);
        ImPlot::PlotScatterG("Lidar Points", getPointFromMap, nullptr, getCount(), ImPlotItemFlags_NoFit);
        ImPlot::SetNextLineStyle(ImVec4(1.0,0.4,0,1));
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Plus);
        ImPlot::PlotScatterG("Camera Points", getCameraPointFromMap, nullptr, getCameraCount(), ImPlotItemFlags_NoFit);
        {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Cross, 4);
            ImPlot::PlotScatterG("Visited", getVisitedPlotPt, nullptr, getVisited().size(), ImPlotItemFlags_NoFit);
        }
        for(const auto& r : regions)
        {
            ImPlot::SetNextLineStyle(ImVec4(0.8,0.8,0,0.5));
            double xs[] = {r.first.x, r.first.x+0.1, r.first.x, r.first.x};
            double ys[] = {r.first.z, r.first.z, r.first.z, r.first.z+0.1};
            ImPlot::PlotLine("Regions", xs, ys, 4);
        }
        ImPlot::SetNextLineStyle(ImVec4(0.0, 0.8, 0, 1));
        ImPlot::PlotScatter("Robot", pos, pos + 2, 1, ImPlotItemFlags_NoFit);
        {
            double xs[] = { pos[0] + MAX_VIC_DETECTION_RANGE * sin(-theta + CAMERA_FOV), pos[0] - MAX_VIC_DETECTION_RANGE * sin(-theta - CAMERA_FOV),
            pos[0] + MAX_VIC_DETECTION_RANGE * sin(-theta + M_PI + CAMERA_FOV), pos[0] - MAX_VIC_DETECTION_RANGE * sin(-theta + M_PI - CAMERA_FOV)};
            double ys[] = { pos[2] + MAX_VIC_DETECTION_RANGE * cos(-theta + CAMERA_FOV), pos[2] -  MAX_VIC_DETECTION_RANGE * cos(-theta - CAMERA_FOV),
            pos[2] + MAX_VIC_DETECTION_RANGE * cos(-theta + M_PI + CAMERA_FOV), pos[2] - MAX_VIC_DETECTION_RANGE * cos(-theta + M_PI - CAMERA_FOV)};
            ImPlot::SetNextLineStyle(ImVec4(1.0,0.4,0,1));
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond);
            ImPlot::PlotScatter("Camera Detection Range", xs, ys, ARRAY_SIZE(xs), ImPlotItemFlags_NoFit);
        }
        {
            double xs[] = { pos[0], pos[0] + 0.1*sin(-theta) };
            double ys[] = { pos[2], pos[2] + 0.1*cos(-theta) };
            ImPlot::SetNextLineStyle(ImVec4(0.0, 0.8, 0, 1));
            ImPlot::PlotLine("Robot", xs, ys, 2, ImPlotItemFlags_NoFit);
        }
        {
            double xs[] = { rb->getCurrentGPSPosition().first, rb->getTargetPos().first };
            double ys[] = { rb->getCurrentGPSPosition().second, rb->getTargetPos().second };

            ImPlot::SetNextLineStyle(ImVec4(0.8,0.8,0.8,1));
            ImPlot::PlotLine("Path", xs, ys, 2, ImPlotItemFlags_NoFit);
        }
        ImPlot::EndPlot();
    }

}

REGION* get_region(webots::GPS *gps)
{
  double pos[3];
  double pos_rounded[3];
  memcpy(pos, gps->getValues(), 3 * sizeof(double));
  pos[2] *= -1;
  //force single floor for now
  pos[1] = 0;
  pos_rounded[0] = floor_to(pos[0], region_size);
  pos_rounded[1] = floor_to(pos[1], region_size);
  pos_rounded[2] = floor_to(pos[2], region_size);

  if(regions.count(GPS_position(pos_rounded)))
    return &regions[pos_rounded];

  return nullptr;
}