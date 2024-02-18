#include "imgui/imgui.h"
#include "imgui/implot.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <iostream>
#include <cmath>

#include "map.h"

#define PI 3.141592653589793238462643383
#define pdd std::pair<double, double>
#define f first
#define s second

using namespace webots;

const double region_size = 0.1;

//thanks stack overflow
template <class T>
inline void hash_combine(std::size_t & s, const T & v)
{
    std::hash<T> h;
    s ^= h(v) + 0x9e3779b9 + (s<< 6) + (s>> 2);
}

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

struct REGION {
    //points
    std::map<std::pair<double,double>, bool> points;
};

std::unordered_map<GPS_position, REGION, GPS_hash> regions;
std::vector<std::pair<double, double>> vec_points;

//theta is in radians
void update_regions_map(GPS *gps, const float *lidar_image, float theta)
{
    vec_points.reserve(20000);

    for(int i = 0; i < 512; i++)
    {
        const float dist = lidar_image[i];
        if(std::isinf(dist))
            continue;
        const float angle = i/512.0 * 2.0 * PI;
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

        x = round_to(x);
        y = round_to(y);

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

        x = round_to(x);
        y = round_to(y);

        const auto coord = std::make_pair(x,y);
        //std::cout << "x: " << x << " y: " << y << std::endl;
        const auto coord2 = std::make_pair(x + pos_rounded[0], y + pos_rounded[2]);
        if(regions[GPS_position(pos_rounded)].points.count(coord) == 0)
        {
            vec_points.push_back(coord2);
            regions[GPS_position(pos_rounded)].points[coord] = true;
        }
    }
}

double clampAngle(double theta)
{
    if(abs(theta) > PI)
    {
        while(abs(theta) > PI)
        {
            theta -= copysign(2, theta) * PI;
        }
    }
    return theta;
}

bool isBetween(double theta, double start, double end)
{
    end -= start;
    theta -= start;
    if(theta < 0)
    {
        theta += 2 * PI;
    }
    if(end < 0)
    {
        end += 2 * PI;
    }
    return theta <= end;
}

std::vector<std::pair<double, double>> cameraPoints;

void update_camera_map(GPS *gps, const float *lidar_image, float theta, Camera* camera)
{
    cameraPoints.reserve(20000);
    double fov = camera->getFov(); //radians

    double leftAngle = clampAngle(theta - PI / 2);
    double rightAngle = clampAngle(theta + PI / 2);

    //sweeping counterclockwise
    pdd leftEndpoints = pdd(clampAngle(leftAngle + fov / 2),
        clampAngle(leftAngle - fov / 2));
    pdd rightEndpoints = pdd(clampAngle(rightAngle + fov / 2),
        clampAngle(rightAngle - fov / 2));

    cameraPoints.reserve(20000);

    for(int i = 0; i < 512; i++)
    {
        const float dist = lidar_image[i];
        if(std::isinf(dist))
            continue;
        const float angle = clampAngle(i/512.0 * 2.0 * PI);
        if(!isBetween(angle, leftEndpoints.f, leftEndpoints.s) &&
            !isBetween(angle, rightEndpoints.f, rightEndpoints.s))
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
        double x = dist*sin(angle - theta) + (pos[0] - pos_rounded[0]);
        double y = dist*cos(angle - theta) + (pos[2] - pos_rounded[2]);

        x = round_to(x);
        y = round_to(y);

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

        x = round_to(x);
        y = round_to(y);

        const auto coord = std::make_pair(x,y);
        //std::cout << "x: " << x << " y: " << y << std::endl;
        const auto coord2 = std::make_pair(x + pos_rounded[0], y + pos_rounded[2]);
        if(regions[GPS_position(pos_rounded)].points.count(coord) == 0)
        {
            cameraPoints.push_back(coord2);
            regions[GPS_position(pos_rounded)].points[coord] = true;
        }
    }
}

std::vector<std::pair<double, double>> getLidarPoints()
{
    return vec_points;
}

std::vector<std::pair<double, double>> getCameraPoints()
{
    return cameraPoints;
}

ImPlotPoint getPointFromMap(int idx, void *_map)
{
    return ImPlotPoint(vec_points[idx].first, vec_points[idx].second);
}

size_t getCount()
{
    return vec_points.size();
}

void clearPointCloud()
{
    regions.clear();
    vec_points.clear();
}

void plotPoints(int w, int h, bool plot_regions)
{
    const size_t count = getCount();

    if(regions.size() > 0 && ImPlot::BeginPlot("point cloud", ImVec2(w-50, h-100)))
    {
        ImPlot::SetNextLineStyle(ImVec4(0,0.4,1.0,1));
        ImPlot::PlotScatterG("", getPointFromMap, nullptr, count, ImPlotItemFlags_NoFit);
        if(plot_regions)
        for(const auto& r : regions)
        {
            ImPlot::SetNextLineStyle(ImVec4(0.8,0.8,0,0.5));
            double xs[] = {r.first.x, r.first.x+0.1, r.first.x, r.first.x};
            double ys[] = {r.first.z, r.first.z, r.first.z, r.first.z+0.1};
            ImPlot::PlotLine("", xs, ys, 4);
        }
        ImPlot::EndPlot();
    }

}