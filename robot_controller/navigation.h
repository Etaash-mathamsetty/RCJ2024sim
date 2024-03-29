#include <stack>
#include <vector>
#include "constants.h"
#include <webots/GPS.hpp>
#include <webots/Lidar.hpp>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

bool onRoute(std::stack<pdd> pts, pdd point);
bool isTraversable(pdd pos, std::vector<pdd> points, double robotRadius);
std::pair<pdd, pdd> getMinMax(std::vector<pdd> list);
std::stack<pdd> pointBfs(pdd cur, pdd tar, std::pair<pdd, pdd> minMax);
void addVisitedPoint(webots::GPS *gps);
pdd getCurrentPosition(webots::GPS* gps);
pdd chooseMove(webots::GPS* gps, webots::Lidar* lidar, DIR currentRotation);

#endif