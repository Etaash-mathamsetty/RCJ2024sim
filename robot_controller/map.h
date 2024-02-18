#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>

void plotPoints(int w, int h, bool plot_regions=true);
void update_regions_map(webots::GPS *gps, const float *lidar_image, float theta=0);
size_t getCount();
void clearPointCloud();
