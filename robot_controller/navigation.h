#include <stack>
#include <vector>
#include "constants.h"
#include <webots/GPS.hpp>
#include <webots/Lidar.hpp>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "RobotInstance.hpp"

double getDist(const pdd& pt1,const pdd& pt2);
bool onRoute(std::stack<pdd> pts, pdd point);
bool isTraversable(const pdd& pos, const std::vector<pdd>& points);
bool isTraversableOpt(const pdd& pos);
bool canSee(pdd cur, pdd tar, const std::vector<pdd>& points);
pdd pointTo(pdd point, double dir);
std::pair<pdd, pdd> getMinMax(const std::vector<pdd>& list);
std::stack<pdd> pointBfs(pdd cur, pdd tar, std::pair<pdd, pdd> minMax);
void removeVisited(pdd point);
bool isVisited(pdd point);
void addVisited(pdd pt);
void addToVisit(pdd point);
void removeToVisit(pdd point);
bool isInToVisit(pdd point);
bool isFollowingBfs();
void clearBfsResult();
pdd chooseMove(RobotInstance *rb, double currentRotation);
const std::set<pdd>& getVisited();
const std::deque<pdd>& getToVisit();

#endif