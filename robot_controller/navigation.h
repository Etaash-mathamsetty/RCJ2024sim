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
bool isTraversableOpt(const pdd& pos, double rad);
bool canSee(pdd cur, pdd tar);
pdd pointTo(pdd point, double dir);
pdd pointTo(pdd point, double dir, double dist);
bool midpoint_check(pdd a, pdd b);
std::pair<pdd, pdd> getMinMax(const std::vector<pdd>& list);
pdd nearestTraversable(pdd point, pdd cur, std::pair<pdd, pdd> minMax);
std::stack<pdd> pointBfs(pdd cur, pdd tar, std::pair<pdd, pdd> minMax);
void removeVisited(pdd point);
bool isVisited(pdd point);
void addVisited(pdd pt);
void addToVisit(pdd point);
void removeToVisit(pdd point);
bool isInToVisit(pdd point);
bool isFollowingBfs();
void clearBfsResult();
void moveToPoint(RobotInstance *rb, pdd point);
pdd chooseMove(RobotInstance *rb, double currentRotation);
const std::set<pdd>& getVisited();
const std::deque<pdd>& getToVisit();
const std::stack<pdd>& getBfsPath();

#endif