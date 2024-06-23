#include <stack>
#include <vector>
#include <unordered_set>
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
bool compPts(pdd a, pdd b);
pdd pointTo(pdd point, double dir);
pdd pointTo(pdd point, double dir, double dist);
bool midpoint_check(pdd a, pdd b);
std::pair<pdd, pdd> getMinMax(const std::vector<pdd>& list);
pdd nearestTraversable(pdd point, pdd cur, std::pair<pdd, pdd> minMax);
std::stack<pdd> pointBfs(pdd cur, pdd tar, std::pair<pdd, pdd> minMax, bool isBlind, bool wall=false);
bool isOnWall(pdd node, double rad = TRAVERSABLE_RADIUS);
bool checkNearbyVisited(pdd point);
void removeVisited(pdd point);
bool isVisited(const pdd& point);
void addVisited(pdd pt);
bool isPseudoVisited(const pdd& point);
void addPseudoVisited(pdd point);
void removePseudoVisited(pdd point);
void addOnWall(pdd point);
void removeOnWall(pdd point);
void clearOnWall();
void bfsAddOnWall(pdd cur, double radius);
void bfsRemoveOnWall(pdd cur, double radius);
bool isFollowingBfs();
bool isAllDone();
void clearBfsResult();
void moveToPoint(RobotInstance *rb, pdd point, bool wall = false);
pdd chooseMove(RobotInstance *rb, double currentRotation);
const std::unordered_set<pdd, pair_hash_combiner<double>>& getVisited();
const std::unordered_set<pdd, pair_hash_combiner<double>>& getOnWall();
const std::stack<pdd>& getBfsPath();
bool isTraversable(const pdd& pos, const std::vector<pdd>& points, double robotRadius);

#endif