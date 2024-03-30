#include <stack>
#include <vector>
#include "constants.h"
#include <webots/GPS.hpp>
#include <webots/Lidar.hpp>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "RobotInstance.hpp"

bool onRoute(std::stack<pdd> pts, pdd point);
bool isTraversable(pdd pos, std::vector<pdd> points);
std::pair<pdd, pdd> getMinMax(std::vector<pdd> list);
std::stack<pdd> pointBfs(pdd cur, pdd tar, std::pair<pdd, pdd> minMax);
void addVisited(pdd pt);
pdd getCurrentPosition(webots::GPS* gps);
pdd chooseMove(RobotInstance *rb, double currentRotation);

#endif