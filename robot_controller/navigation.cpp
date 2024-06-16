#include <iostream>
#include <math.h>
#include <utility>
#include <vector>
#include <queue>
#include <stack>
#include <map>
#include <string>
#include <climits>
#include <set>
#include <algorithm>
#include <webots/GPS.hpp>
#include <webots/Lidar.hpp>
#include "map.h"
#include "constants.h"
#include "RobotInstance.hpp"

using namespace std;
using namespace webots;

#define f first
#define s second

pdd getXY(double angle, double distance)
{
    return { distance * sin(angle), distance * cos(angle) };
}

double getDist2(const pdd& pt1, const pdd& pt2)
{
    return pow(pt2.f - pt1.f, 2) + pow(pt2.s - pt1.s, 2);
}

double getDist(const pdd&  pt1, const pdd& pt2)
{
    return hypot(pt2.f - pt1.f, pt2.s - pt1.s);
}

bool compPts(pdd pt1, pdd pt2, double thresh)
{
    return (abs(pt1.f - pt2.f) < thresh && abs(pt1.s - pt2.s) < thresh);
}

bool compPts(pdd pt1, pdd pt2)
{
    return (abs(pt1.f - pt2.f) < 0.005 && abs(pt1.s - pt2.s) < 0.005);
}

pair<pdd, pdd> getMinMax(const vector<pdd>& list)
{
    double minx = 10000;
    double maxx = -10000;
    double miny = 10000;
    double maxy = -10000;
    for (size_t i = 0; i < list.size(); i++)
    {
        if (list[i].f < minx) minx = list[i].f;
        if (list[i].s < miny) miny = list[i].s;
        if (list[i].f > maxx) maxx = list[i].f;
        if (list[i].s > maxy) maxy = list[i].s;
    }
    return make_pair(pdd(minx, miny), pdd(maxx, maxy));
}

void getPlot(vector<double> datalist, pdd  pos, vector<pdd> *pList, double angle)
{
    vector<pdd> &PLRef = *pList;
    for (size_t i = 0; i < datalist.size(); i++)
    {
        if (datalist[i] != -1)
        {
            pdd coords = getXY(2 * M_PI * i / 512 + angle, datalist[i]);
            PLRef.push_back({ coords.f + pos.f, coords.s - pos.s });
        }
    }
}

bool onRoute(stack<pdd> pts, pdd point)
{
    while (!pts.empty())
    {
        if (compPts(pts.top(), point)) return 1;
        pts.pop();
    }
    return 0;
}

void printStack(stack<pdd> pts)
{
    if (pts.empty()) cout << "empty stack" << endl;
    while (!pts.empty())
    {
        cout << pts.top().f << " " << pts.top().s << endl;
        pts.pop();
    }
}

bool isTraversable(const pdd& pos, const vector<pdd>& points)
{
    for (const pdd& pt : points)
    {
        if (getDist(pos, pt) < 0.043)
            return false;
    }
    return true;
}

bool isTraversableOpt(const pdd& pos, double rad)
{
    for (const auto& r : get_neighboring_regions(pos))
    {
        if(r)
        {
            for (const auto &pair: r->points) {
                if (pair.second.wall && getDist(pos, pair.first) <= rad)
                    return false;
            }
        }
    }

    return true;
}

bool isTraversableOpt(const pdd& pos)
{
    return isTraversableOpt(pos, 0.043);
}

double minDist(pdd a, pdd b, pdd p)
{
    double l2 = getDist2(a, b);
    if(l2 == 0)
    {
        return getDist(a, p);
    }
    double dot = (p.f - a.f) * (b.f - a.f) + (p.s - a.s) * (b.s - a.s);
    double t = max(0.0, min(1.0, dot / l2));
    double projx = a.f + t * (b.f - a.f);
    double projy = a.s + t * (b.s - a.s);
    return getDist(p, pdd(projx, projy));
}

bool canSee(pdd cur, pdd tar)
{
    auto vec = get_neighboring_regions(tar);
    for (const auto& r : vec)
    {
        if(r)
        {
            for(const auto& pair: r->points)
            {
                if(minDist(cur, tar, pair.first) < 0.01 && pair.second.wall)
                {
                    return false;
                }
            }
        }
    }
    vec = get_neighboring_regions(cur);
    for (const auto& r : vec)
    {
        if(r)
        {
            for(const auto& pair: r->points)
            {
                if(minDist(cur, tar, pair.first) < 0.01 && pair.second.wall)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

bool midpoint_check(pdd a, pdd b)
{
    if(!isTraversableOpt(a) || !isTraversableOpt(b))
    {
        return false;
    }

    pdd mid = midpoint(a, b);

    if(!isTraversableOpt(mid))
    {
        return false;
    }

    if(r2d(a) == r2d(b) || r2d(mid) == r2d(a) || r2d(mid) == r2d(b))
    {
        return true;
    }

    return midpoint_check(a, mid) && midpoint_check(mid, b);

}

unordered_map<pdd, pdd, pair_hash_combiner<double>> parent;
bool isWallTracing = false;

stack<pdd> optimizeRoute(stack<pdd> route)
{
    if (route.empty())
    {
        return route;
    }
    stack<pdd> ret;

    while (!route.empty())
    {
        pdd cur = r2d(route.top());
        route.pop();
        ret.push(cur);
    }

    pdd last_pt = ret.top();

    //flip the path again (cuz this stack class sucks)
    stack<pdd> rev_ret;
    while (!ret.empty())
    {
        pdd cur = ret.top();
        ret.pop();
        if (rev_ret.size() > 0 && midpoint_check(rev_ret.top(), cur))
        {
            last_pt = cur;
            continue;
        }
        rev_ret.push(last_pt);
        last_pt = cur;
    }

    //ensure the last point is added
    if(rev_ret.size() == 0 || rev_ret.top() != last_pt)
    {
        rev_ret.push(last_pt);
    }

    return rev_ret;
}

pdd nearestTraversable(pdd point, pdd cur, pair<pdd, pdd> minMax)
{
    pdd min = r2d(minMax.f), max = r2d(minMax.s);
    queue<pdd> q;
    set<pdd> visited;
    point = r2d(point);
    cur = r2d(cur);
    q.push(point);
    while (!q.empty())
    {
        pdd node = r2d(q.front());
        q.pop();
        if (visited.count(node) > 0 || node.f < min.f || node.f > max.f || node.s < min.s || node.s > max.s)
        {
            continue;
        }
        visited.insert(node);
        if (isTraversableOpt(node, 0.043) && canSee(cur, node))
        {
            // cout << "nearest traversable found" << endl;
            return node;
        }
        else
        {
            pdd adjacentNodes[8] = {
                r2d(pdd(node.f, node.s - 0.01)),
                r2d(pdd(node.f, node.s + 0.01)),
                r2d(pdd(node.f + 0.01, node.s)),
                r2d(pdd(node.f - 0.01, node.s)),
                r2d(pdd(node.f - 0.01, node.s - 0.01)),
                r2d(pdd(node.f + 0.01, node.s + 0.01)),
                r2d(pdd(node.f + 0.01, node.s - 0.01)),
                r2d(pdd(node.f - 0.01, node.s + 0.01))
            };
            for (const pdd& adjacent : adjacentNodes)
            {
                if (!visited.count(adjacent))
                {
                    q.push(adjacent);
                }
            }
        }
    }
    return cur;
}

stack<pdd> pointBfs(pdd cur, pdd tar, pair<pdd, pdd> minMax, bool isBlind)
{
    pdd min = r2d(minMax.f), max = r2d(minMax.s);
    parent.clear();
    parent.reserve(10000);
    set<pdd> visited;
    queue<pdd> q;
    cur = r2d(cur);
    tar = r2d(tar);
    q.push(cur);
    parent[cur] = pdd(DBL_MAX, DBL_MAX);
    bool targetFound = false;
    while (!q.empty())
    {
        pdd node = r2d(q.front());
        q.pop();
        if (visited.count(node) > 0 || node.f < min.f || node.f > max.f || node.s < min.s || node.s > max.s)
        {
            continue;
        }
        if (!isTraversableOpt(node) || !isTraversable(node, getLidarPoints()))
        {
            if (node != cur)
            {
                continue;
            }
        }
        visited.insert(node);
        if (!compPts(node, tar) || (isBlind && isVisited(node)))
        {
            //north: +y, east: +x, south: -y, west: -x; 
            pdd adjacentNodes[8] = {
                r2d(pdd(node.f, node.s - 0.01)),
                r2d(pdd(node.f, node.s + 0.01)),
                r2d(pdd(node.f + 0.01, node.s)),
                r2d(pdd(node.f - 0.01, node.s)),
                r2d(pdd(node.f - 0.01, node.s - 0.01)),
                r2d(pdd(node.f + 0.01, node.s + 0.01)),
                r2d(pdd(node.f + 0.01, node.s - 0.01)),
                r2d(pdd(node.f - 0.01, node.s + 0.01))
            };

            // pdd adjacentNodes[4] = {
            //     r2d(pdd(node.f, node.s + 0.01)),
            //     r2d(pdd(node.f + 0.01, node.s)),
            //     r2d(pdd(node.f, node.s - 0.01)),
            //     r2d(pdd(node.f - 0.01, node.s))
            // };

            for (const pdd& adjacent : adjacentNodes)
            {
                if (!visited.count(adjacent) && isTraversableOpt(adjacent))
                {
                    q.push(adjacent);
                    parent[adjacent] = node;
                }
            }
        }
        else
        {
            targetFound = true;
            tar = node;
            break;
        }
    }
    if (targetFound)
    {
        pdd pindex = tar;
        stack<pdd> route;
        while (pindex != cur)
        {
            route.push(pindex);
            pindex = r2d(parent[pindex]);
        }
        cout << "!!!!! successful bfs route length: " << route.size() << endl;
        route.push(cur);
        route = optimizeRoute(route);
        route.pop();
        cout << "!!!!! optimized bfs route length: " << route.size() << endl;
        return route;
    }

    if(getToVisit().size() == 0)
    {
        removeVisited(RobotInstance::getInstance()->getStartPos());
        addToVisit(RobotInstance::getInstance()->getStartPos());
    }
    removeOnWall(tar);
    removeToVisit(tar);
    removeVisited(tar);
    cout << "Route not found" << endl;
    return stack<pdd>();
}

deque<pdd> toVisit;
set<pdd> visitedPoints;
stack<pdd> bfsResult = {};
set<pdd> onWall;

pdd getClosestHeuristic(set<pdd> points, pdd cur, pdd start)
{
    pdd closest = cur;
    double minDist = DBL_MAX;
    for (const pdd& point : points)
    {
        double dist = getDist(cur, point)/* + cbrt(getDist(start, point))*/;
        if (dist < minDist)
        {
            minDist = dist;
            closest = point;
        }
    }
    return closest;
}

bool isOnWall(pdd node)
{
    if (onWall.find(node) != onWall.end())
    {
        return true;
    }
    if (!isTraversableOpt(node) || !isTraversable(node, getLidarPoints()))
    {
        return false;
    }

    // pdd adjacents[8] = {
    //     r2d(pdd(node.f, node.s - 0.01)),
    //     r2d(pdd(node.f, node.s + 0.01)),
    //     r2d(pdd(node.f + 0.01, node.s)),
    //     r2d(pdd(node.f - 0.01, node.s)),
    //     r2d(pdd(node.f - 0.01, node.s - 0.01)),
    //     r2d(pdd(node.f + 0.01, node.s + 0.01)),
    //     r2d(pdd(node.f + 0.01, node.s - 0.01)),
    //     r2d(pdd(node.f - 0.01, node.s + 0.01))
    // };
    pdd adjacents[4] = {
        r2d(pdd(node.f, node.s - 0.01)),
        r2d(pdd(node.f, node.s + 0.01)),
        r2d(pdd(node.f + 0.01, node.s)),
        r2d(pdd(node.f - 0.01, node.s))
    };

    for (const pdd& adjacent : adjacents)
    {
        if (!isTraversableOpt(adjacent) || !isTraversable(adjacent, getLidarPoints()))
        {
            return true;
        }
    }
    return false;
}

// rotation already multiplied by -1
pdd nearestIsOnWall(pdd cur, pair<pdd, pdd> minMax, double rotation, pdd start)
{
    // return getClosestHeuristic(onWall, cur, start);
    pdd min = r2d(minMax.f), max = r2d(minMax.s);
    rotation = clampAngle(round(rotation / (M_PI / 2)) * M_PI / 2);
    queue<pdd> q;
    set<pdd> visited;
    cur = r2d(cur);
    q.push(cur);
    while (!q.empty())
    {
        pdd node = r2d(q.front());
        q.pop();
        if (visited.count(node) > 0 || node.f < min.f || node.f > max.f || node.s < min.s || node.s > max.s)
        {
            continue;
        }
        if (!isTraversableOpt(node) || !isTraversable(node, getLidarPoints()))
        {
            if (node != cur)
            {
                continue;
            }
        }
        visited.insert(node);
        if (isOnWall(node) && node != cur && !isVisited(node))
        {
            return node;
        }
        else
        {
            // prioritizing forward
            pdd adjacentNodes[8] = {
                pointTo(node, rotation),
                pointTo(node, rotation + M_PI / 4),
                pointTo(node, rotation + M_PI / 2),
                pointTo(node, rotation + 3 * M_PI / 4),
                pointTo(node, rotation - M_PI / 4),
                pointTo(node, rotation - M_PI / 2),
                pointTo(node, rotation - 3 * M_PI / 4),
                pointTo(node, rotation + M_PI)
            };
            // pdd adjacentNodes[8] = {
            //     r2d(pdd(node.f, node.s - 0.01)),
            //     r2d(pdd(node.f, node.s + 0.01)),
            //     r2d(pdd(node.f + 0.01, node.s)),
            //     r2d(pdd(node.f - 0.01, node.s)),
            //     r2d(pdd(node.f - 0.01, node.s - 0.01)),
            //     r2d(pdd(node.f + 0.01, node.s + 0.01)),
            //     r2d(pdd(node.f + 0.01, node.s - 0.01)),
            //     r2d(pdd(node.f - 0.01, node.s + 0.01))
            // };
            for (const pdd& adjacent : adjacentNodes)
            {
                if (!visited.count(adjacent))
                {
                    q.push(adjacent);
                }
            }
        }
    }
    return cur;
}

struct wallNode
{
    pdd point;
    double direction; // direction from previous point to current
};

const double wtRadius = 0.05;

pdd bfsWallTrace(RobotInstance* rb, pdd cur)
{
    isWallTracing = true;
    cur = r2d(cur);
    if (!isOnWall(cur))
    {
        pdd temp = nearestIsOnWall(cur, getMinMax(getLidarPoints()), rb->getYaw() * -1, cur);
        if (temp == cur)
        {
            return cur;
        }
        cur = temp;
    }
    const float* rangeImage = rb->getLidar()->getRangeImage() + 512;
    int hr = rb->getLidar()->getHorizontalResolution();
    double offset = rangeImage[hr * 3 / 4] < rangeImage[hr / 4] ? M_PI / 2 : -M_PI / 2;
    cout << (rangeImage[hr * 3 / 4] < rangeImage[hr / 4] ? "left" : "right") << endl;
    pdd min = r2d({cur.f - wtRadius, cur.s - wtRadius}), max = r2d({cur.f + wtRadius, cur.s + wtRadius});
    queue<wallNode> q;
    set<pdd> visited;
    q.push({cur, 0});
    while (!q.empty())
    {
        wallNode node = q.front();
        pdd point = node.point;
        double rotation = node.direction;
        q.pop();
        if (isOnWall(point) && (point.f < min.f || point.f > max.f || point.s < min.s || point.s > max.s))
        {
            return point;
        }
        if (visited.count(point) > 0 || !isOnWall(point) || (isVisited(point) && point != cur) || point.f < min.f || point.f > max.f || point.s < min.s || point.s > max.s)
        {
            continue;
        }
        visited.insert(point);
        double directions[8] = {
            rotation + offset,
            rotation + offset / 2,
            rotation,
            rotation + offset * 3 / 2,
            rotation + M_PI,
            rotation - offset,
            rotation - offset / 2,
            rotation - offset * 3 / 2
        };
        for (const double& direction : directions)
        {
            q.push({pointTo(point, clampAngle(direction)), clampAngle(direction)});
        }
    }
    cout << "no traceable wall found" << endl;
    return nearestIsOnWall(cur, getMinMax(getLidarPoints()), rb->getYaw() * -1, cur);
}

bool isVisited(pdd point)
{
    return visitedPoints.count(point) > 0;
}

void removeToVisit(pdd point)
{
    auto it = find(toVisit.begin(), toVisit.end(), point);
    if(it != toVisit.end())
    {
        toVisit.erase(it);
    }
}

void removeVisited(pdd point)
{
    visitedPoints.erase(point);
    removeToVisit(point);
}

void addVisited(pdd point)
{
    if (isTraversableOpt(point))
    {
        visitedPoints.insert(point);
    }
    auto it = find(toVisit.begin(), toVisit.end(), point);
    if (it != toVisit.end())
    {
        toVisit.erase(it);
    }
}

const set<pdd>& getVisited()
{
    return visitedPoints;
}

void addToVisit(pdd point)
{
    if(isInToVisit(point))
    {
        return;
    }
    toVisit.push_back(point);
}

bool isInToVisit(pdd point)
{
    auto it = find(toVisit.begin(), toVisit.end(), point);
    return it != toVisit.end();
}

void addOnWall(pdd point)
{
    onWall.insert(point);
}

void removeOnWall(pdd point)
{
    onWall.erase(point);
}

void bfsAddOnWall(pdd cur, double radius)
{
    pdd min = {cur.f - radius, cur.s - radius};
    pdd max = {cur.f + radius, cur.s + radius};
    queue<pdd> q;
    set<pdd> visited;
    cur = r2d(cur);
    q.push(cur);
    while (!q.empty())
    {
        pdd node = r2d(q.front());
        q.pop();
        if (visited.count(node) > 0 || !isTraversableOpt(node) || !canSee(cur, node) || node.f < min.f || node.f > max.f || node.s < min.s || node.s > max.s)
        {
            continue;
        }
        visited.insert(node);
        if (isOnWall(node) && !isVisited(node))
        {
            addOnWall(node);
        }
        else
        {
            removeOnWall(node);
        }
        pdd adjacentNodes[8] = {
            r2d(pdd(node.f, node.s - 0.01)),
            r2d(pdd(node.f, node.s + 0.01)),
            r2d(pdd(node.f + 0.01, node.s)),
            r2d(pdd(node.f - 0.01, node.s)),
            r2d(pdd(node.f - 0.01, node.s - 0.01)),
            r2d(pdd(node.f + 0.01, node.s + 0.01)),
            r2d(pdd(node.f + 0.01, node.s - 0.01)),
            r2d(pdd(node.f - 0.01, node.s + 0.01))
        };
        for (const pdd& adjacent : adjacentNodes)
        {
            if (!visited.count(adjacent) || !isTraversableOpt(adjacent) || !canSee(cur, adjacent))
            {
                q.push(adjacent);
            }
        }
    }
}

pdd pointTo(pdd point, double dir)
{
    dir = clampAngle(dir);
    return pdd(r2d(point.f + 0.01 * sin(dir)), r2d(point.s + 0.01 * cos(dir)));
}

pdd pointTo(pdd point, double dir, double dist)
{
    dir = clampAngle(dir);
    return pdd(r2d(point.f + dist * sin(dir)), r2d(point.s + dist * cos(dir)));
}

void printToVisit()
{
    auto it = toVisit.begin();
    while(it != toVisit.end())
    {
        cout << it->first << " " << it->second << endl;
        it++;
    }
    cout << endl;
}

const std::deque<pdd>& getToVisit()
{
    return toVisit;
}

const std::set<pdd>& getOnWall()
{
    return onWall;
}

stack<pdd> toVisitBfs(pdd current, pair<pdd, pdd> minMax)
{
    if(!isTraversableOpt(current))
    {
        return stack<pdd>();
    }

    double radius = 0.15;
    pair<pdd, pdd> localMinMax = make_pair(
        pdd(current.first - radius, current.second - radius),
        pdd(current.first + radius, current.second + radius));

    
    stack<pdd> res = pointBfs(current, current, localMinMax, true);
    if (!res.empty())
    {
        return res;
    }

    if (toVisit.empty())
    {
        res = pointBfs(current, current, minMax, true);
        return res;
    }

    res = {};
    for(; !toVisit.empty() && res.empty();)
    {
        res = pointBfs(current, toVisit.back(), minMax, false);
        if(res.empty())
        {
            toVisit.pop_back();
        }
    }
    return res;
}

bool isBfs = false;
bool allDone = false;

bool isFollowingBfs()
{
    return isBfs;
}

bool isAllDone()
{
    return allDone;
}

void clearBfsResult()
{
    bfsResult = stack<pdd>();
}

const stack<pdd>& getBfsPath()
{
    return bfsResult;
}

void moveToPoint(RobotInstance *rb, pdd point)
{
    point = r2d(point);
    bfsResult = pointBfs(rb->getCurrentGPSPosition(), point, getMinMax(getLidarPoints()), false);
    // cout << "done" << endl;
    while(!bfsResult.empty())
    {
        pdd next = bfsResult.top();
        // if(!canSee(point, next))
        // {
        //     clearBfsResult();
        //     return;
        // }
        rb->moveToPos(next);
        if (!bfsResult.empty() && compPts(rb->getRawGPSPosition(), next))
        {
            bfsResult.pop();
        }
        point = next;
    }
}

pdd chooseMove(RobotInstance *rb, double rotation)
{
    rotation *= -1;
    pdd cur = rb->getCurrentGPSPosition();
    // pdd nearestOnWall = nearestIsOnWall(cur, getMinMax(getLidarPoints()), rotation, rb->getStartPos());
    pdd bfsTarget = bfsWallTrace(rb, cur);
    // pdd nearestUnseen = getClosestHeuristic(getCameraToVisit(), cur, rb->getStartPos());
    if (bfsTarget == cur)
    {
        cout << "no move found" << endl;
        // allDone = true;
        // moveToPoint(rb, rb->getStartPos());
    }
    return bfsTarget;
}

// pdd chooseMove(RobotInstance *rb, double rotation)
// {
//     pdd currentPoint = rb->getCurrentGPSPosition();
//     // webots rotation is flipped sin(-t) = x and cos(-t) = y
//     rotation *= -1;

//     if(!isTraversableOpt(currentPoint))
//     {
//         // std::cout << "current point is not traversable!" << std::endl;
//         clearBfsResult();
//         //look for a nearby traversable point
//         pdd nextPoint = currentPoint;
//         for(int i = 1; i <= 7; i++)
//         {
//             pdd ret, farRet;
//             double angle;
//             switch(i)
//             {
//                 case 4: angle = M_PI / 4; break;
//                 case 1: angle = M_PI / 2; break;
//                 case 5: angle = M_PI * 3 / 4; break;
//                 case 7: angle = M_PI; break;
//                 case 6: angle = -M_PI * 3 / 4; break;
//                 case 2: angle = -M_PI / 2; break;
//                 case 3: angle = -M_PI / 4; break;
//                 case 0:
//                 default: angle = 0; break;
//             }
//             if (angle == 0)
//             {
//                 continue;
//             }
//             for(double l = 0.05; l >= 0.01; l-=0.01)
//             {
//                 ret = pointTo(currentPoint, rotation + angle, l);
//                 ret = r2d(ret);
//                 if (isTraversableOpt(ret))
//                 {
//                     printPoint(ret);
//                     nextPoint = ret;
//                     // cout << "turning to " << i << endl;
//                     i = 8;
//                     break;
//                 }
//             }
//         }
//         if(nextPoint != currentPoint)
//         {
//             isBfs = false;
//             return nextPoint;
//         }
//     }

//     if(!toVisit.empty() && isVisited(toVisit.back()))
//     {
//         toVisit.pop_back();
//     }
//     if (!bfsResult.empty())
//     {
//         pdd nextPoint = r2d(bfsResult.top());
//         if(currentPoint == nextPoint && midpoint_check(currentPoint, nextPoint))
//         {
//             bfsResult.pop();
//             if(bfsResult.empty())
//             {
//                 isBfs = false;
//                 return currentPoint;
//             }
//             nextPoint = r2d(bfsResult.top());
//         }
//         else if(!midpoint_check(currentPoint, nextPoint) && !toVisit.empty())
//         {
//             bfsResult = toVisitBfs(currentPoint, getMinMax(getLidarPoints()));
//             if(!bfsResult.empty())
//             {
//                 nextPoint = r2d(bfsResult.top());
//                 if(!toVisit.empty() && !isTraversableOpt(nextPoint))
//                 {
//                     toVisit.pop_back();
//                     while(!bfsResult.empty())
//                     {
//                         bfsResult.pop();
//                     }
//                     isBfs = false;
//                     return currentPoint;
//                 }
//             }
//         }
//         isBfs = true;
//         // cout << pointToString(rb->getRawGPSPosition()) << " --> " << pointToString(nextPoint) << endl;
//         return nextPoint;
//     }
//     for (int i = 0; i <= 2; i++)
//     {
//         if (i == 2 && !toVisit.empty())
//         {
//             bfsResult = toVisitBfs(currentPoint, getMinMax(getLidarPoints()));
//             if(!bfsResult.empty())
//             {
//                 pdd nextPoint = r2d(bfsResult.top());
//                 if(currentPoint == nextPoint)
//                 {
//                     bfsResult.pop();
//                     if(bfsResult.empty())
//                     {
//                         isBfs = false;
//                         return currentPoint;
//                     }
//                     nextPoint = r2d(bfsResult.top());
//                 }
//                 isBfs = true;
//                 return nextPoint;
//             }
//         }
//         if (i == 0)
//         {
//             for(int i = 0; i <= 2; i++)
//             {
//                 pdd target = r2d(pointTo(currentPoint, rotation, 0.05 - i * 0.02));
//                 if ((!isVisited(target)
//                             && canSee(currentPoint, target))
//                     && isTraversableOpt(target))
//                 {
//                     printPoint(target);
//                     // std::cout << "visited" << isVisited(target) << std::endl;
//                     isBfs = false;
//                     return target;
//                 }
//             }
//         }
//         if (i == 1)
//         {
//             pdd nextPoint = currentPoint;
//             for(int i = 1; i <= 7; i++)
//             {
//                 pdd ret, farRet;
//                 double angle;
//                 switch(i)
//                 {
//                     //case 4: angle = M_PI / 4; ret = pointTo(currentPoint, rotation + M_PI / 4); farRet = pointTo(currentPoint, rotation + M_PI / 4, 0.07); break;
//                     case 1: angle = M_PI / 2; ret = pointTo(currentPoint, rotation + M_PI / 2); farRet = pointTo(currentPoint, rotation + M_PI / 2, 0.07); break;
//                    // case 5: angle = M_PI * 3 / 4; ret = pointTo(currentPoint, rotation + M_PI * 3 / 4); farRet = pointTo(currentPoint, rotation + M_PI * 3 / 4, 0.07); break;
//                     case 7: angle = M_PI; ret = pointTo(currentPoint, rotation + M_PI); farRet = pointTo(currentPoint, rotation + M_PI, 0.07); break;
//                     //case 6: angle = -M_PI * 3 / 4; ret = pointTo(currentPoint, rotation - M_PI * 3 / 4); farRet = pointTo(currentPoint, rotation - M_PI * 3 / 4, 0.07); break;
//                     case 2: angle = -M_PI / 2; ret = pointTo(currentPoint, rotation - M_PI / 2); farRet = pointTo(currentPoint, rotation - M_PI / 2, 0.07); break;
//                     //case 3: angle = -M_PI / 4; ret = pointTo(currentPoint, rotation - M_PI / 4); farRet = pointTo(currentPoint, rotation - M_PI / 4, 0.07); break;
//                     case 0:
//                     default: angle = 0; break;
//                 }
//                 if (angle == 0)
//                 {
//                     continue;
//                 }
//                 ret = r2d(ret);
//                 if ((!isVisited(ret)
//                             && canSee(currentPoint, ret))
//                     && isTraversableOpt(ret))
//                 {
//                     printPoint(ret);
//                     nextPoint = ret;
//                     // cout << "turning to " << i << endl;
//                     i = 8;
//                 }
//                 farRet = r2d(farRet);
//                 if ((!isVisited(farRet)
//                             && canSee(currentPoint, farRet))
//                     && midpoint_check(currentPoint, farRet))
//                 {
//                     printPoint(farRet);
//                     nextPoint = farRet;
//                     // cout << "turning to " << i << endl;
//                     i = 8;
//                 }
//             }
//             if(nextPoint != currentPoint)
//             {
//                 isBfs = false;
//                 return nextPoint;
//             }
//         }
//     }
//     if (toVisit.empty())
//     {
//         printPoint(currentPoint);
//         bfsResult = pointBfs(currentPoint, rb->getStartPos(), getMinMax(getLidarPoints()), true);
//         if (!bfsResult.empty())
//         {
//             pdd nextPoint = bfsResult.top();
//             bfsResult.pop();
//             return nextPoint;
//         }
//     }
//     cout << "no move found" << endl;
//     return currentPoint;
// }

bool isTraversable(const pdd& pos, const vector<pdd>& points, double robotRadius)
{
    for (size_t i = 0; i < points.size(); i++)
    {
        if (getDist(pos, points[i]) < robotRadius)
            return 0;
    }
    return 1;
}