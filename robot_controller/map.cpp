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
#include "mapping.h"
#include "RobotInstance.hpp"

#define pdd std::pair<double, double>
#define f first
#define s second

using namespace webots;

const double region_size = 0.1;

inline double round_to(double value, const double precision = 0.01)
{
    return std::round(value / precision) * precision;
}

inline double floor_to(double value, const double precision = 0.01)
{
    return std::floor(value / precision) * precision;
}

inline double ceil_to(double value, const double precision = 0.01)
{
    return std::ceil(value / precision) * precision;
}

//does the opposite of truncation
inline double anti_trunc_to(double value, const double precision = 0.01)
{
    if(value > 0)
    {
        return std::ceil(value / precision) * precision;
    }
    else
    {
        return std::floor(value / precision) * precision;
    }
}


std::map<pdd, REGION> regions;
std::vector<pdd> vecLidarPoints;
std::vector<pdd> vecCameraPoints;
std::vector<pdd> victims;
std::vector<pdd> reportedVictims;

std::pair<pdd, pdd> lidarToPoint(const pdd& pos, double dist, double absAngle)
{
    pdd region_pos;

    double x = dist*sin(absAngle) + pos.first;
    double y = dist*cos(absAngle) + pos.second;

    //x = round_to(x, 0.02);
    //y = round_to(y, 0.02);

    region_pos.second = floor_to(y, region_size);
    region_pos.first = floor_to(x, region_size);

    return std::make_pair(pdd(x, y), r2d(region_pos));
}

//theta is in radians
void update_regions_map(RobotInstance* rb, const float *lidar_image, float theta)
{
    vecLidarPoints.reserve(20000);
    
    pdd pos = rb->getRawGPSPosition();

    for(int i = 0; i < 512; i++)
    {
        double dist = lidar_image[i];

        if(std::isinf(dist))
            continue;

        dist *= cos(LIDAR_TILT_ANGLE);

        if(dist > 0.3)
            continue;

        const double angle = i/512.0 * 2.0 * M_PI;

        const auto coord = lidarToPoint(pos, dist, angle - theta);
        
        addLidarPoint(coord.first);
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
    if(!isTraversableOpt(point, 0.005)) return;

    pdd rcoord;
    rcoord.first = floor_to(point.first, region_size);
    rcoord.second = floor_to(point.second, region_size);
    rcoord = r2d(rcoord);

    if(regions[rcoord].points.count(point) == 0 || !regions[rcoord].points[point].wall)
    {
        vecLidarPoints.push_back(point);
        regions[rcoord].points[point].wall = true;
    }
}

void addVictim(pdd point)
{
    if (find(victims.begin(), victims.end(), point) == victims.end())
    {
        victims.push_back(point);
    }
}

void reportVictim(pdd point)
{
    if (find(reportedVictims.begin(), reportedVictims.end(), point) == reportedVictims.end())
    {
        reportedVictims.push_back(point);
    }
}

bool notBeenDetected(pdd victim)
{
    const double dist = 0.03;
    std::cout << reportedVictims.size() << " " << isTraversable(victim, reportedVictims, dist) << std::endl;
    return isTraversable(victim, reportedVictims, dist);
}

ImPlotPoint getPointFromMap(int idx, void *_map)
{
    return {vecLidarPoints[idx].first, vecLidarPoints[idx].second};
}

ImPlotPoint getCameraPointFromMap(int idx, void *_map)
{
    return {vecCameraPoints[idx].first, vecCameraPoints[idx].second};
}

ImPlotPoint getOnWallPoint(int idx, void* doesntmatter)
{
    auto it = getOnWall().begin();
    std::advance(it, idx);
    return {it->first, it->second};
}

ImPlotPoint getVisitedPlotPt(int idx, void *param)
{
    auto it = getVisited().begin();
    std::advance(it, idx);
    return {it->first, it->second};
}

ImPlotPoint getVictimPoint(int idx, void *param)
{
    return {victims[idx].first, victims[idx].second};
}

size_t getCount()
{
    return vecLidarPoints.size();
}

size_t getCameraCount()
{
    return vecCameraPoints.size();
}

size_t getVictimCount()
{
    return victims.size();
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

    ImVec2 GraphSize = ImVec2(w-50, h-115);

    ImGui::Text("Number of Regions %ld", regions.size());

    if(!regions.empty() && ImPlot::BeginPlot("Debug View", GraphSize))
    {
        ImPlot::SetupLegend(ImPlotLocation_North, ImPlotLegendFlags_Outside | ImPlotLegendFlags_Horizontal);
        ImPlot::SetNextLineStyle(ImVec4(0,0.4,1.0,1));
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Asterisk, 3);
        ImPlot::PlotScatterG("Lidar Points", getPointFromMap, nullptr, getCount(), ImPlotItemFlags_NoFit);
        // ImPlot::SetNextLineStyle(ImVec4(1.0,0.4,0,1));
        // ImPlot::SetNextMarkerStyle(ImPlotMarker_Plus);
        // ImPlot::PlotScatterG("Camera Points", getCameraPointFromMap, nullptr, getCameraCount(), ImPlotItemFlags_NoFit);
        {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Cross, 4);
            ImPlot::PlotScatterG("Visited", getVisitedPlotPt, nullptr, getVisited().size(), ImPlotItemFlags_NoFit);
        }
        {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 4);
            ImPlot::PlotScatterG("On Wall", getOnWallPoint, nullptr, getOnWall().size(), ImPlotItemFlags_NoFit);
        }
        {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 4);
            ImPlot::PlotScatterG("Victims", getVictimPoint, nullptr, getVictimCount(), ImPlotItemFlags_NoFit);
        }
        for(const auto& r : regions)
        {
            ImPlot::SetNextLineStyle(ImVec4(0.8,0.8,0,0.5));
            double xs[] = {r.first.first, r.first.first+0.1, r.first.first + 0.1, r.first.first};
            double ys[] = {r.first.second, r.first.second, r.first.second + 0.1, r.first.second+0.1};
            ImPlot::PlotLine("Regions", xs, ys, 4, ImPlotLineFlags_Loop);
        }
        ImPlot::SetNextLineStyle(ImVec4(0.0, 0.8, 0, 1));
        ImPlot::PlotScatter("Robot", pos, pos + 2, 1, ImPlotItemFlags_NoFit);
        double startx = rb->getStartPos().first, starty = rb->getStartPos().second;
        ImPlot::PlotScatter("Start", &startx, &starty, 1, ImPlotItemFlags_NoFit);
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
        if(getBfsPath().size() > 0)
        {
            std::vector<double> xs, ys;
            std::stack<pdd> path = getBfsPath();
            xs.push_back(rb->getRawGPSPosition().first);
            ys.push_back(rb->getRawGPSPosition().second);
            while(!path.empty())
            {
                xs.push_back(path.top().first);
                ys.push_back(path.top().second);
                path.pop();
            }
            ImPlot::SetNextLineStyle(ImVec4(0.8,0.8,0.8,1));
            ImPlot::PlotLine("BFS Path", xs.data(), ys.data(), xs.size(), ImPlotItemFlags_NoFit);
            ImPlot::PlotScatter("BFS Path", xs.data(), ys.data(), xs.size(), ImPlotItemFlags_NoFit);
        }
        else
        {
            double xs[] = { rb->getRawGPSPosition().first, rb->getTargetPos().first };
            double ys[] = { rb->getRawGPSPosition().second, rb->getTargetPos().second };

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


  const auto rcoord = r2d(std::make_pair(pos_rounded[0], pos_rounded[2]));

  if(regions.count(rcoord))
    return &regions[rcoord];

  return nullptr;
}

std::pair<pdd, pdd> get_lidar_minmax_opt()
{
    double minx = 100000;
    double maxx = -100000;
    double miny = 100000;
    double maxy = -100000;
    for(const auto& r : regions)
    {
        if(maxx < r.first.first)
        {
            maxx = r.first.first;
        }

        if(maxy < r.first.second)
        {
            maxy = r.first.second;
        }

        if(minx > r.first.first)
        {
            minx = r.first.first;
        }

        if(miny > r.first.second)
        {
            miny = r.first.second;
        }
    }
    maxx += region_size;
    maxy += region_size;
    return make_pair(pdd(minx, miny), pdd(maxx, maxy));
}

std::vector<REGION*> get_neighboring_regions(const pdd& pt, double rad)
{
    pdd min;
    pdd max;
    min.first = floor_to(pt.first - rad, region_size);
    min.second = floor_to(pt.second - rad, region_size);
    max.first = floor_to(pt.first + rad, region_size);
    max.second = floor_to(pt.second + rad, region_size);

    min = r2d(min);
    max = r2d(max);

    std::vector<REGION*> ret;

    ret.reserve(20);

    for(pdd cur = min; cur.first <= max.first; cur.first = r2d(cur.first + region_size))
    {
        for(cur.second = min.second; cur.second <= max.second; cur.second = r2d(cur.second + region_size))
        {
            //printPoint(rcoord);
            if(regions.count(cur) > 0)
            {
                ret.push_back(&regions[cur]);
            }
        }
     }

    // std::cout << pointToString(pt) << " " << "nearest regions: " << ret.size() << std::endl;

    return ret;
}