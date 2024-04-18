#include <stack>
#include <vector>
#include "constants.h"
#include <webots/GPS.hpp>
#include <webots/Lidar.hpp>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "RobotInstance.hpp"

double getDist(pdd  pt1, pdd pt2);
bool onRoute(std::stack<pdd> pts, pdd point);
bool isTraversable(pdd pos, std::vector<pdd> points);
bool canSee(pdd cur, pdd tar, const std::vector<pdd>& points);
pdd pointTo(pdd point, double dir);
std::pair<pdd, pdd> getMinMax(std::vector<pdd> list);
std::stack<pdd> pointBfs(pdd cur, pdd tar, std::pair<pdd, pdd> minMax);
bool isVisited(pdd point);
void addVisited(pdd pt);
void addToVisit(pdd point);
bool isInToVisit(pdd point);
pdd chooseMove(RobotInstance *rb, double currentRotation);
const std::set<pdd>& getVisited();

#endif