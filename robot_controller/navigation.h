#include <stack>
#include <vector>
#include "constants.h"

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

bool onRoute(std::stack<pdd> pts, pdd point);
bool isTraversable(pdd pos, std::vector<pdd> points, double robotRadius);
std::pair<pdd, pdd> getMinMax(std::vector<pdd> list);
std::stack<pdd> pointBfs(pdd cur, pdd tar, std::pair<pdd, pdd> minMax);

#endif