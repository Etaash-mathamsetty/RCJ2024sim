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
#include <unordered_set>
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

inline bool compPts(const pdd& pt1, const pdd& pt2, double thresh)
{
    return (abs(pt1.f - pt2.f) < thresh && abs(pt1.s - pt2.s) < thresh);
}

bool compPts(const pdd& pt1, const pdd& pt2)
{
    return compPts(pt1, pt2, 0.005);
}

pair<pdd, pdd> getMinMax(const vector<pdd>& list)
{
    double minx = 100000;
    double maxx = -100000;
    double miny = 100000;
    double maxy = -100000;
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
        if (getDist(pos, pt) < TRAVERSABLE_RADIUS)
            return false;
    }
    return true;
}

bool isTraversableOpt(const pdd& pos, double rad)
{
    for (const auto& r : get_neighboring_regions(pos, rad))
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
    return isTraversableOpt(pos, TRAVERSABLE_RADIUS);
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
    for (const auto& r : get_neighboring_regions(midpoint(tar, cur), getDist(cur, tar)/2.0))
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
        pdd cur = route.top();
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

bool isNearWall(pdd pt)
{
    for(REGION* r : get_neighboring_regions(pt, 0.1))
    {
        if(r)
        {
            for(const auto& p : r->points)
            {
                if(getDist(pt, p.first) <= 0.053)
                    return true;
            }
        }
    }

    return false;
}

stack<pdd> optimizeRouteOnWall(stack<pdd> route)
{
    if (route.empty())
    {
        return route;
    }
    stack<pdd> ret;
    cout << route.size();

    while (!route.empty())
    {
        pdd cur = route.top();
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
        if (!ret.empty() && !rev_ret.empty())
        {
            vector<pdd> toSkip;
            pdd next = ret.top();
            last_pt = cur;
            while (midpoint_check(next, cur))
            {
                double maxDist = 0;
                for (const pdd& p : toSkip)
                {
                    maxDist = max(maxDist, minDist(cur, next, p));
                }
                if (maxDist < 0.01)
                {
                    last_pt = next;
                    ret.pop();
                    if (ret.empty())
                    {
                        break;
                    }
                    toSkip.push_back(next);
                    next = ret.top();
                }
                else
                {
                    break;
                }
            }
            rev_ret.push(last_pt);
        }
        else
        {
            rev_ret.push(cur);
        }
    }

    //ensure the last point is added
    if(rev_ret.size() == 0 || rev_ret.top() != last_pt)
    {
        rev_ret.push(last_pt);
    }

    cout << " --> " << rev_ret.size() << endl;

    return rev_ret;
}

pdd nearestTraversable(pdd point, pdd cur, pair<pdd, pdd> minMax)
{
    pdd min = r2d(minMax.f), max = r2d(minMax.s);
    queue<pdd> q;
    unordered_set<pdd, pair_hash_combiner<double>> visited;
    point = r2d(point);
    cur = r2d(cur);
    q.push(point);
    while (!q.empty())
    {
        pdd node = q.front();
        q.pop();
        if (visited.count(node) > 0 || node.f < min.f || node.f > max.f || node.s < min.s || node.s > max.s)
        {
            continue;
        }
        visited.insert(node);
        if (isTraversableOpt(node, TRAVERSABLE_RADIUS) && canSee(cur, node))
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

pdd findNearestTraversable(pdd cur, double rad, double inc)
{
    pdd cur2 = cur;
    cur2.first -= rad;
    cur2.second -= rad;

    for( ;cur2.first <= cur.first + rad; cur2.first += inc)
    {
        for(cur2.second = cur.second - rad; cur2.second <= cur.second + rad; cur2.second += inc)
        {
            if(isTraversableOpt(cur2) && cur != cur2)
            {
                return cur2;
            }
        }
    }

    return cur;
}

pdd findNearestTraversable(pdd cur)
{
    // return r2d(findNearestTraversable(cur, 0.03, 0.01));
    double rad = 0.05;
    return nearestTraversable(cur, cur, {pdd(cur.f - rad, cur.s - rad), pdd(cur.f + rad, cur.s + rad)});
}

stack<pdd> pointBfs(pdd cur, pdd tar, pair<pdd, pdd> minMax, bool isBlind, bool wall)
{
    pdd min = r2d(minMax.f), max = r2d(minMax.s);
    unordered_map<pdd, pdd, pair_hash_combiner<double>> parent;
    parent.reserve(10000);
    unordered_set<pdd, pair_hash_combiner<double>> visited;
    queue<pdd> q;
    q.push(cur);
    parent[r3d(cur)] = pdd(DBL_MAX, DBL_MAX);
    bool targetFound = false;

    if(!isTraversableOpt(cur))
    {
        pdd traversable = findNearestTraversable(cur);
        if(compPts(traversable, cur))
        {
            std::cout << "cur is not traversable!" << std::endl;
            return stack<pdd>();
        }

        std::cout << "found traversable pt!!" << std::endl;
        stack<pdd> path;
        path.push(traversable);
        return path;
    }

    if(!isTraversableOpt(tar))
    {
        std::cout << "target is not traversable!" << std::endl;
        std::cout << "target: " << pointToString(tar) << std::endl;
        removeOnWall(tar);
        addVisited(tar);
        return stack<pdd>();
    }

    pdd final_node;
    double angle = -atan2(tar.f - cur.f, tar.s - cur.s);

    while (!q.empty())
    {
        pdd node = q.front();
        q.pop();
        if (visited.count(r3d(node)) > 0 || node.f < min.f || node.f > max.f || node.s < min.s || node.s > max.s)
        {
            continue;
        }
        if (!isTraversableOpt(node))
        {
            if (!compPts(node, cur))
            {
                continue;
            }
        }
        visited.insert(r3d(node));
        if (!midpoint_check(node, tar) || (isBlind && (isVisited(node) || isPseudoVisited(node))))
        {
            //north: +y, east: +x, south: -y, west: -x;
            pdd adjacentNodes[] = {
                pointTo(node, angle, 0.007),
                pointTo(node, angle + M_PI / 2, 0.007),
                pointTo(node, angle - M_PI / 2, 0.007),
                pointTo(node, angle + M_PI, 0.007),
                pointTo(node, angle + M_PI / 4, 0.007),
                pointTo(node, angle - M_PI / 4, 0.007),
                pointTo(node, angle + 3 * M_PI / 4, 0.007),
                pointTo(node, angle - 3 * M_PI / 4, 0.007),
            };

            for (const pdd& adjacent : adjacentNodes)
            {
                if (!visited.count(r3d(adjacent)) && isTraversableOpt(adjacent))
                {
                    q.push(adjacent);
                    parent[r3d(adjacent)] = node;
                }
            }
        }
        else
        {
            targetFound = true;
            final_node = node;
            break;
        }
    }
    if (targetFound)
    {
        pdd pindex = final_node;
        stack<pdd> route;
        if(!compPts(final_node, tar))
            route.push(tar);
        while (!compPts(pindex, cur))
        {
            route.push(pindex);
            pindex = parent[r3d(pindex)];
        }
        // cout << "!!!!! successful bfs route length: " << route.size() << endl;
        route.push(cur);
        if (!wall)
            route = optimizeRoute(route);
        else
            route = optimizeRouteOnWall(route);
        route.pop();
        // cout << "!!!!! optimized bfs route length: " << route.size() << endl;
        return route;
    }

    // removeOnWall(tar);
    // bfsRemoveOnWall(tar, 0.05);
    // addVisited(tar);
    cout << "Route not found:" << endl;
    printPoint(cur);
    printPoint(tar);
    cout << " ---- " << std::endl;

    return stack<pdd>();
}

unordered_set<pdd, pair_hash_combiner<double>> visitedPoints;
unordered_set<pdd, pair_hash_combiner<double>> pseudoVisited;
stack<pdd> bfsResult = {};
unordered_set<pdd, pair_hash_combiner<double>> onWall;

pdd getClosestHeuristic(const unordered_set<pdd, pair_hash_combiner<double>>& points, pdd cur, pdd start)
{
    pdd closest = cur;
    double minDist = DBL_MAX;
    for (const pdd& point : points)
    {
        double dist = getDist(cur, point)/* + cbrt(getDist(start, point))*/;
        if (dist < minDist && isTraversableOpt(point))
        {
            minDist = dist;
            closest = point;
        }
    }
    return closest;
}

bool isOnWall(pdd node, double rad)
{
    node = r2d(node);
    if (!isTraversableOpt(node))
    {
        onWall.erase(node);
        return false;
    }

    if (onWall.find(node) != onWall.end())
    {
        return true;
    }

    pdd adjacents[4] = {
        r2d(pdd(node.f, node.s - 0.01)),
        r2d(pdd(node.f, node.s + 0.01)),
        r2d(pdd(node.f + 0.01, node.s)),
        r2d(pdd(node.f - 0.01, node.s))
    };

    for (const pdd& adjacent : adjacents)
    {
        if (!isTraversableOpt(adjacent, rad))
        {
            return true;
        }
    }
    return false;
}

// check if there is enough visited points nearby to count as visited
bool checkNearbyVisited(pdd point)
{
    point = r2d(point);
    if (!isTraversableOpt(point) || isVisited(point) || isPseudoVisited(point))
    {
        return false;
    }
    pdd adjacents[8] = {
        r2d(pdd(point.f, point.s - 0.01)),
        r2d(pdd(point.f, point.s + 0.01)),
        r2d(pdd(point.f + 0.01, point.s)),
        r2d(pdd(point.f - 0.01, point.s)),
        r2d(pdd(point.f - 0.01, point.s - 0.01)),
        r2d(pdd(point.f + 0.01, point.s + 0.01)),
        r2d(pdd(point.f + 0.01, point.s - 0.01)),
        r2d(pdd(point.f - 0.01, point.s + 0.01))
    };
    int totalTraversable = 0, visited = 0;
    for (const pdd& adjacent : adjacents)
    {
        if (isTraversableOpt(adjacent))
        {
            totalTraversable++;
        }
        if (isVisited(adjacent))
        {
            visited++;
        }
    }
    return ((visited * 1.0) / (totalTraversable * 1.0) >= 0.5);
}

// rotation already multiplied by -1
pdd nearestIsOnWall(pdd cur, pair<pdd, pdd> minMax, double rotation, pdd start)
{
    // return getClosestHeuristic(onWall, cur, start);
    pdd min = r2d(minMax.f), max = r2d(minMax.s);
    rotation = clampAngle(round(rotation / (M_PI / 2)) * M_PI / 2);
    queue<pdd> q;
    unordered_set<pdd, pair_hash_combiner<double>> visited;
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
        if (!isTraversableOpt(node))
        {
            if (!compPts(node, cur))
            {
                continue;
            }
        }
        visited.insert(node);
        if (onWall.count(node) && !compPts(node, cur) && !isVisited(node) && !isPseudoVisited(node) && isTraversableOpt(node))
        {
            cout << "nearest on wall found " << isTraversableOpt(node) << endl;
            return node;
        }
        else
        {
            // prioritizing forward
            pdd adjacentNodes[8] = {
                pointTo(node, rotation),
                pointTo(node, rotation + M_PI / 4, hypot(0.01, 0.01)),
                pointTo(node, rotation + M_PI / 2),
                pointTo(node, rotation + 3 * M_PI / 4, hypot(0.01, 0.01)),
                pointTo(node, rotation - M_PI / 4, hypot(0.01, 0.01)),
                pointTo(node, rotation - M_PI / 2),
                pointTo(node, rotation - 3 * M_PI / 4, hypot(0.01, 0.01)),
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
                if (!visited.count(r2d(adjacent)))
                {
                    q.push(r2d(adjacent));
                }
            }
        }
    }
    if (!onWall.empty())
    {
        cout << "getting closest heuristic" << endl;
        return getClosestHeuristic(onWall, cur, start);
    }
    return start;
}

struct wallNode
{
    pdd point;
    double direction; // direction from previous point to current
};

const double wtRadius = 0.05;
bool isLeft = true;

void checkSide(const float* rangeImage, int hr)
{
    // cout << rangeImage[hr * 3 / 4] << " " << rangeImage[hr / 4] << endl;
    double threshold = 0.05;
    if (isLeft && rangeImage[hr / 4] + threshold < rangeImage[hr * 3 / 4] && !isinf(rangeImage[hr * 3 / 4]))
    {
        isLeft = false;
    }
    else if (!isLeft && rangeImage[hr * 3 / 4] + threshold < rangeImage[hr / 4] && !isinf(rangeImage[hr / 4]))
    {
        isLeft = true;
    }
}

stack<pdd> dfsWallTrace(RobotInstance* rb, pdd cur)
{
    isWallTracing = true;
    cur = r2d(cur);
    if (!isOnWall(cur))
    {
        cur = nearestIsOnWall(cur, get_lidar_minmax_opt(), -rb->getYaw(), rb->getStartPos());
    }
    checkSide(rb->getLidar()->getRangeImage() + 1024, rb->getLidar()->getHorizontalResolution());
    double offset = isLeft ? -M_PI / 2 : M_PI / 2;
    cout << (isLeft ? "left" : "right") << endl;
    pdd min = r2d({cur.f - wtRadius, cur.s - wtRadius}), max = r2d({cur.f + wtRadius, cur.s + wtRadius});
    stack<wallNode> st;
    unordered_map<pdd, pdd, pair_hash_combiner<double>> parent;
    parent.reserve(10000);
    unordered_set<pdd, pair_hash_combiner<double>> visited;
    st.push({cur, 0});
    bool isFound = false;
    pdd tar;
    while (!st.empty() && !isFound)
    {
        wallNode node = st.top();
        pdd point = node.point;
        double rotation = node.direction;
        st.pop();
        if (isOnWall(point) && (point.f < min.f || point.f > max.f || point.s < min.s || point.s > max.s))
        {
            isFound = true;
            tar = point;
            break;
        }
        if (visited.count(point) > 0 || !isOnWall(point) || ((isVisited(point) || isPseudoVisited(point)) && point != cur) || point.f < min.f || point.f > max.f || point.s < min.s || point.s > max.s)
        {
            continue;
        }
        if (abs(rotation) > M_PI / 4 + 0.05)
        {
            isFound = true;
            tar = point;
            break;
        }
        visited.insert(point);
        double directions[8] = { // in order of priority
            rotation + offset,
            rotation + offset / 2,
            rotation,
            rotation + offset * 3 / 2,
            rotation + M_PI,
            rotation - offset,
            rotation - offset / 2,
            rotation - offset * 3 / 2
        };
        for (int i = 7; i >= 0; i--)
        {
            pdd temp = pointTo(point, clampAngle(rb->getYaw() + directions[i]));
            if(!isTraversableOpt(temp))
                continue;
            if (visited.count(temp) == 0)
            {
                st.push({temp, directions[i]});
                parent[temp] = point;
            }
        }
    }
    if (isFound)
    {
        pdd pindex = tar;
        stack<pdd> res;
        while (pindex != cur)
        {
            res.push(pindex);
            pindex = parent[pindex];
        }
        res.push(cur);
        res = optimizeRouteOnWall(res);
        res.pop();
        return res;
    }
    cout << "no traceable wall found" << endl;
    return pointBfs(cur, nearestIsOnWall(cur, get_lidar_minmax_opt(), rb->getYaw(), rb->getStartPos()), get_lidar_minmax_opt(), false);
}

bool isVisited(const pdd& point)
{
    return visitedPoints.count(r2d(point)) > 0;
}

void removeVisited(pdd point)
{
    visitedPoints.erase(point);
}

bool isPseudoVisited(const pdd& point)
{
    return pseudoVisited.count(r2d(point)) > 0;
}

void addVisited(pdd point)
{
    if (isTraversableOpt(point))
    {
        visitedPoints.insert(r2d(point));
    }
}

void addPseudoVisited(pdd point)
{
    if (isTraversableOpt(point))
    {
        pseudoVisited.insert(r2d(point));
    }
}

void removePseudoVisited(pdd point)
{
    pseudoVisited.erase(r2d(point));
}

const unordered_set<pdd, pair_hash_combiner<double>>& getVisited()
{
    return visitedPoints;
}

void addOnWall(pdd point)
{
    onWall.insert(r2d(point));
}

void removeOnWall(pdd point)
{
    onWall.erase(r2d(point));
}

void clearOnWall()
{
    onWall.clear();
}

void bfsAddOnWall(pdd cur, double radius)
{
    pdd min = {cur.f - radius, cur.s - radius};
    pdd max = {cur.f + radius, cur.s + radius};
    queue<pdd> q;
    unordered_set<pdd, pair_hash_combiner<double>> visited;
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
        if (isOnWall(node) && !isVisited(node) && !isPseudoVisited(node))
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

void bfsRemoveOnWall(pdd cur, double radius)
{
    pdd min = {cur.f - radius, cur.s - radius};
    pdd max = {cur.f + radius, cur.s + radius};
    queue<pdd> q;
    unordered_set<pdd, pair_hash_combiner<double>> visited;
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
        if (isOnWall(node))
        {
            removeOnWall(node);
            addVisited(node);
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
            if (!visited.count(adjacent) && isTraversableOpt(adjacent) && canSee(cur, adjacent))
            {
                q.push(adjacent);
            }
        }
    }
}

pdd pointTo(pdd point, double dir)
{
    dir = clampAngle(dir);
    return pdd(point.f + 0.01 * sin(dir), point.s + 0.01 * cos(dir));
}

pdd pointTo(pdd point, double dir, double dist)
{
    dir = clampAngle(dir);
    return pdd(point.f + dist * sin(dir), point.s + dist * cos(dir));
}

const std::unordered_set<pdd, pair_hash_combiner<double>>& getOnWall()
{
    return onWall;
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

void moveToPoint(RobotInstance *rb, pdd point, bool wall)
{
    point = r2d(point);
    bfsResult = pointBfs(rb->getCurrentGPSPosition(), point, get_lidar_minmax_opt(), false, wall);
    if(wall && bfsResult.empty())
    {
        //fallback
        bfsResult = pointBfs(rb->getCurrentGPSPosition(), point, get_lidar_minmax_opt(), false);
    }
    if (bfsResult.empty())
    {
        removeOnWall(point);
        // bfsRemoveOnWall(point, 0.05);
        addVisited(point);
        return;
    }
    // cout << "done" << endl;
    while(!bfsResult.empty())
    {
        pdd next = bfsResult.top();
        // if(!midpoint_check(point, next))
        // {
        //     clearBfsResult();
        //     return;
        // }
        rb->moveToPos(next);
        if (!bfsResult.empty() && compPts(rb->getRawGPSPosition(), next))
        {
            bfsResult.pop();
        }
        else if (!bfsResult.empty() && !isTraversableOpt(next))
        {
            bfsResult.pop();
        }
        point = next;
    }
}

stack<pdd> wallTracePath;

pdd chooseMove(RobotInstance *rb, double rotation)
{
    pdd cur = rb->getRawGPSPosition();
    // pdd nearestOnWall = nearestIsOnWall(cur, getMinMax(getLidarPoints()), rotation, rb->getStartPos());

    // for(auto it = onWall.begin(); it != onWall.end();)
    // {
    //     pdd point = *it;
    //     if(!isTraversableOpt(point) || isVisited(point))
    //     {
    //         it = onWall.erase(it);
    //     }
    //     else
    //     {
    //         it++;
    //     }
    // }

    // pdd nearestUnseen = getClosestHeuristic(getCameraToVisit(), cur, rb->getStartPos());

    if (!wallTracePath.empty())
    {
        if (compPts(rb->getRawGPSPosition(), wallTracePath.top()))
        {
            wallTracePath.pop();
        }
        else if(!isTraversableOpt(wallTracePath.top()))
        {
            wallTracePath.pop();
        }
        else
        {
            pdd temp;
            while (!wallTracePath.empty())
            {
                temp = wallTracePath.top();
                wallTracePath.pop();
            }
            stack<pdd> bfsResult = pointBfs(cur, temp, get_lidar_minmax_opt(), false, false);
            if (bfsResult.empty())
            {
                wallTracePath = stack<pdd>();
            }
        }
        if (!wallTracePath.empty())
        {
            return wallTracePath.top();
        }
    }
    wallTracePath = dfsWallTrace(rb, cur);
    if (wallTracePath.empty())
    {
        cout << "no move found" << endl;

        if(compPts(cur, rb->getStartPos()))
            allDone = true;

        return cur;
    }
    return wallTracePath.top();
}

bool isTraversable(const pdd& pos, const vector<pdd>& points, double robotRadius)
{
    for (size_t i = 0; i < points.size(); i++)
    {
        if (getDist(pos, points[i]) < robotRadius)
            return 0;
    }
    return 1;
}

// int main()
// {
//     vector<pdd> traversable;
//     vector<pdd> untraversable;
//     vector<double> data = { 0.0553168, 0.0553269, 0.0553452, 0.055372, 0.0554071, 0.0554506, 0.0555026, 0.055563, 0.0556319, 0.0557094, 0.0557955, 0.0558921, 0.0559956, 0.0561081, 0.0562294, 0.0563598, 0.0564993, 0.056648, 0.0568061, 0.0569737, 0.0571509, 0.0573378, 0.0575346, 0.0577416, 0.0579587, 0.0581863, 0.0584245, 0.0586735, 0.0589354, 0.0592066, 0.0594894, 0.059784, 0.0600905, 0.0604093, 0.0607442, 0.0610885, 0.061446, 0.0618171, 0.062202, 0.0626033, 0.0630172, 0.0634463, 0.0638929, 0.0643535, 0.0648306, 0.0653268, 0.0658385, 0.06637, 0.0669206, 0.0674885, 0.0680786, 0.0686873, 0.0693197, 0.0699721, 0.0706499, 0.0713516, 0.072078, 0.0728306, 0.0736103, 0.0744181, 0.0752528, 0.0761203, 0.0770197, -1, -1, 0.0804967, 0.0816618, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0855191, 0.0840026, 0.0826001, 0.0823988, 0.0811571, 0.0800119, -1, -1, 0.0765944, 0.0757093, 0.0748558, 0.0740301, 0.0732356, 0.0724689, 0.0717289, 0.0710141, 0.0703244, 0.0696584, 0.0690132, 0.0683922, 0.0677903, 0.0672111, 0.0666496, 0.0661095, 0.0655874, 0.0650818, 0.0645955, 0.0641241, 0.0636691, 0.0632319, 0.0628082, 0.0623995, 0.0620073, 0.0616273, 0.0612612, 0.0609086, 0.060569, 0.0602457, 0.0599315, 0.0596295, 0.0593394, 0.059061, 0.058794, 0.05854, 0.0582952, 0.0580612, 0.0578377, 0.0576246, 0.0574217, 0.0572287, 0.0570457, 0.0568724, 0.0567086, 0.0565544, 0.0564094, 0.0562736, 0.0561469, 0.0560292, 0.0559204, 0.0558204, 0.055731, 0.0556484, 0.0555744, 0.055509, 0.0554521, 0.0554037, 0.0553637, 0.055332, 0.0553087, 0.0552938, 0.0552872, 0.0552872, 0.0552972, 0.0553156, 0.0553423, 0.0553774, 0.0554209, 0.0554728, 0.0555332, 0.0556021, 0.0556796, 0.0557657, 0.0558622, 0.0559657, 0.056078, 0.0561993, 0.0563296, 0.0564691, 0.0566177, 0.0567757, 0.0569432, 0.0571203, 0.0573072, 0.0575039, 0.0577107, 0.0579278, 0.0581552, 0.0583933, 0.0586422, 0.0589039, 0.059175, 0.0594577, 0.0597521, 0.0600585, 0.0603771, 0.0607087, 0.0610528, 0.0614101, 0.0617809, 0.0621657, 0.0625667, 0.0629804, 0.0634092, 0.0638556, 0.0643159, 0.0647928, 0.0652887, 0.0658, 0.0663279, 0.0668782, 0.0674457, 0.0680354, 0.0686437, 0.0692757, 0.0699277, 0.0706051, 0.0713064, 0.0720286, 0.0727808, 0.07356, 0.0743672, 0.0752013, 0.0760683, 0.0769671, -1, -1, 0.0804354, 0.0815982, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0855907, 0.0840711, 0.0826655, 0.0824633, 0.0812189, 0.0800712, -1, -1, 0.0766465, 0.0757609, 0.0749068, 0.0740805, 0.0732856, 0.0725184, 0.0717779, 0.071059, 0.0703688, 0.0697025, 0.0690568, 0.0684354, 0.0678332, 0.0672536, 0.0666918, 0.0661513, 0.0656256, 0.0651196, 0.0646331, 0.0641614, 0.0637061, 0.0632687, 0.0628448, 0.0624358, 0.0620434, 0.0616633, 0.061297, 0.0609441, 0.0606044, 0.0602778, 0.0599634, 0.0596612, 0.059371, 0.0590925, 0.0588254, 0.0585713, 0.0583263, 0.0580922, 0.0578686, 0.0576553, 0.0574523, 0.0572593, 0.0570762, 0.0569028, 0.0567389, 0.0565846, 0.0564395, 0.0563037, 0.0561769, 0.0560592, 0.0559503, 0.0558503, 0.0557608, 0.0556782, 0.0556042, 0.0555388, 0.0554818, 0.0554334, 0.0553933, 0.0553616, 0.0553384, 0.0553234, 0.0553168 },
//         data2 = { 0.0613304, 0.0613416, 0.061362, 0.0613916, 0.0614305, 0.0614788, 0.0615363, 0.0616033, 0.0616797, 0.0617656, 0.0618611, 0.0619682, 0.062083, 0.0622076, 0.0623422, 0.0624867, 0.0626413, 0.0628062, 0.0629815, 0.0631673, 0.0633637, 0.063571, 0.0637892, 0.0640186, 0.0642594, 0.0645117, 0.0647758, 0.0650518, 0.0653421, 0.0656429, 0.0659565, 0.066283, 0.0666228, 0.0669763, 0.0673475, 0.0677293, 0.0681256, 0.068537, 0.0689638, 0.0694087, 0.0698676, 0.0703433, 0.0708384, 0.0713491, 0.0718781, 0.0724282, 0.0729955, 0.0735848, 0.0741953, 0.0748249, 0.0754791, 0.0761539, 0.076855, 0.0775784, 0.0783298, 0.0791078, 0.0799131, 0.080646, 0.0792159, 0.0778932, 0.0777034, 0.0765323, 0.0754523, -1, -1, 0.0722287, 0.0713941, 0.0705892, 0.0698105, 0.0690613, 0.0683383, 0.0676405, 0.0669664, 0.066316, 0.065688, 0.0650795, 0.0644939, 0.0639263, 0.0633801, 0.0628507, 0.0623413, 0.061849, 0.0613721, 0.0609136, 0.060469, 0.0600399, 0.0596277, 0.0592281, 0.0588427, 0.0584728, 0.0581146, 0.0577693, 0.0574367, 0.0571166, 0.0568117, 0.0565153, 0.0562305, 0.055957, 0.0556944, 0.0554427, 0.0552032, 0.0549723, 0.0547516, 0.0545409, 0.0543399, 0.0541485, 0.0539666, 0.053794, 0.0536305, 0.0534761, 0.0533306, 0.0531939, 0.0530659, 0.0529464, 0.0528354, 0.0527328, 0.0526385, 0.0525541, 0.0524763, 0.0524065, 0.0523449, 0.0522912, 0.0522455, 0.0522077, 0.0521779, 0.0521559, 0.0521418, 0.0521356, 0.0521356, 0.0521451, 0.0521624, 0.0521876, 0.0522207, 0.0522617, 0.0523107, 0.0523676, 0.0524326, 0.0525056, 0.0525868, 0.0526778, 0.0527754, 0.0528813, 0.0529957, 0.0531186, 0.0532501, 0.0533902, 0.0535392, 0.0536972, 0.0538642, 0.0540404, 0.0542259, 0.0544209, 0.0546256, 0.0548401, 0.0550645, 0.0552992, 0.055546, 0.0558017, 0.0560683, 0.0563458, 0.0566348, 0.0569352, 0.0572479, 0.0575724, 0.0579093, 0.058259, 0.0586219, 0.059, 0.0593901, 0.0597945, 0.0602154, 0.0606495, 0.0610991, 0.0615667, 0.062049, 0.0625467, 0.0630656, 0.0636008, 0.0641569, 0.0647305, 0.0653264, 0.0659413, 0.06658, 0.0672414, 0.0679224, 0.0686317, 0.0693664, 0.0701276, 0.0709142, 0.0717317, 0.0725792, -1, -1, 0.0758533, 0.0769497, 0.0781385, 0.0783353, 0.0796781, 0.08113, 0.0826998, 0.0831174, 0.0848842, 0.086795, 0.0874021, 0.089555, 0.0902993, 0.0927296, 0.0936294, 0.0963809, 0.0974634, 0.0986118, 0.101878, 0.103247, 0.1047, 0.108614, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.109747, 0.108483, -1, -1, 0.103273, 0.100589, 0.0997699, 0.0973908, 0.0967234, 0.0946111, 0.0926575, 0.092205, 0.0904687, 0.0888624, 0.0873767, 0.0871629, 0.0858476, 0.0846345, -1, -1, 0.0810161, 0.0801454, 0.0813836, 0.0815886, 0.0829871, 0.0844994, 0.0861344, 0.0865695, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.102155, 0.101455, 0.0992396, 0.0971905, 0.096716, 0.0948947, 0.0932099, 0.0916517, 0.0914275, 0.0900479, 0.0887755, -1, -1, 0.0849797, 0.0839978, 0.0830508, 0.0821347, 0.0812533, 0.0804026, 0.0795816, 0.0787845, 0.0780194, 0.0772805, 0.0765647, 0.0758757, 0.075208, 0.0745654, 0.0739425, 0.0733432, 0.0727604, 0.0721994, 0.0716599, 0.071137, 0.0706322, 0.0701472, 0.0696771, 0.0692237, 0.0687886, 0.0683671, 0.067961, 0.0675698, 0.0671931, 0.066831, 0.0664825, 0.0661474, 0.0658256, 0.0655168, 0.0652206, 0.0649389, 0.0646673, 0.0644077, 0.0641598, 0.0639234, 0.0636983, 0.0634843, 0.0632812, 0.063089, 0.0629073, 0.0627362, 0.0625753, 0.0624247, 0.0622842, 0.0621536, 0.0620329, 0.061922, 0.0618227, 0.0617312, 0.0616491, 0.0615766, 0.0615134, 0.0614597, 0.0614153, 0.0613802, 0.0613543, 0.0613378, 0.0613304 },
//         data3 = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0778043, 0.076946, 0.0761203, 0.0753234, 0.0745543, 0.0738114, 0.0730945, 0.0724023, 0.0717316, 0.0710861, 0.0704606, 0.0698586, 0.069275, 0.0687136, 0.068171, 0.0676454, 0.06714, 0.06665, 0.0661771, 0.0657227, 0.0652823, 0.0648575, 0.0644499, 0.064055, 0.0636745, 0.0633079, 0.062955, 0.062619, 0.0622924, 0.0619785, 0.061677, 0.0613876, 0.0611101, 0.0608461, 0.0605917, 0.0603484, 0.0601161, 0.0598946, 0.0596837, 0.0594832, 0.059293, 0.0591128, 0.0589426, 0.0587823, 0.0586316, 0.0584905, 0.0583588, 0.0582365, 0.0581234, 0.0580195, 0.0579265, 0.0578407, 0.0577638, 0.0576959, 0.0576367, 0.0575864, 0.0575448, 0.0575119, 0.0574877, 0.0574722, 0.0574653, 0.0574653, 0.0574757, 0.0574949, 0.0575226, 0.0575591, 0.0576044, 0.0576583, 0.0577211, 0.0577927, 0.0578732, 0.0579627, 0.058063, 0.0581706, 0.0582874, 0.0584135, 0.0585489, 0.0586939, 0.0588484, 0.0590126, 0.0591867, 0.0593708, 0.059565, 0.0597695, 0.0599845, 0.0602101, 0.0604465, 0.060694, 0.0609526, 0.0612247, 0.0615066, 0.0618003, 0.0621063, 0.0624248, 0.062756, 0.0631007, 0.0634583, 0.0638297, 0.0642152, 0.0646151, 0.0650319, 0.065462, 0.0659077, 0.0663716, 0.0668501, 0.0673458, 0.0678612, 0.0683927, 0.0689414, 0.0695134, 0.0701033, 0.0707163, 0.0713485, 0.0720054, 0.0726832, 0.0733872, 0.0741162, 0.0748669, 0.0756487, 0.0764586, 0.0772976, 0.0781646, 0.0790658, 0.08, -1, -1, 0.0836073, 0.0848159, 0.0861263, 0.0863433, 0.0878235, 0.089424, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0921733, -1, -1, -1, 0.0869214, 0.0867089, 0.0870682, 0.088097, -1, -1, 0.0920683, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0809921, 0.0795542, 0.0782243, 0.0780329, 0.0768555, 0.0757696, -1, -1, 0.0773241, 0.0784434, 0.079657, 0.0798585, 0.0812293, 0.0827115, -1, 0.0847357, 0.0865392, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
//         data4 = { 0.0674526, 0.0674656, 0.0674889, 0.0675223, 0.067566, 0.0676199, 0.0676841, 0.0677586, 0.0678434, 0.0679388, 0.0680447, 0.0681641, 0.0682912, 0.0684292, 0.068578, 0.0687379, 0.0689089, 0.0690911, 0.0692848, 0.06949, 0.069707, 0.0699359, 0.0701768, 0.0704301, 0.0706958, 0.0709743, 0.0712657, 0.0715703, 0.0718915, 0.0722234, 0.0725692, 0.0729294, 0.0733042, 0.0736941, 0.0741044, 0.0745253, 0.0749624, 0.075416, 0.0758866, 0.076378, 0.076884, 0.0774084, 0.0779552, 0.0785182, 0.0791013, 0.0797087, 0.080334, 0.0809835, 0.0816574, 0.0823514, 0.0830735, 0.0838173, 0.084591, 0.0853883, 0.0862175, 0.087076, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0731282, 0.0719094, 0.0717353, 0.0706562, 0.069661, -1, -1, 0.0666905, 0.0664541, 0.0674828, 0.0676538, 0.0688158, 0.0700723, 0.0714308, 0.071793, 0.0733221, 0.0749759, 0.0755021, 0.0773656, 0.0780105, 0.0801144, 0.080894, 0.0832762, 0.0842142, 0.0852094, 0.088038, 0.0892236, 0.0904833, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0811395, 0.0804265, 0.0797413, 0.0790761, 0.0784372, 0.0778148, 0.0772158, 0.0766408, 0.0760824, 0.0755435, 0.0750267, 0.0745248, 0.0740408, 0.0735773, 0.0731273, 0.0726938, 0.0722762, 0.0718742, 0.0714887, 0.0711167, 0.0707592, 0.0704159, 0.0700864, 0.0697704, 0.0694708, 0.0691811, 0.0689042, 0.0686398, 0.0683877, 0.0681478, 0.0679197, 0.0677033, 0.0674984, 0.0673049, 0.0671226, 0.0669514, 0.0667911, 0.0666415, 0.0665027, 0.0663744, 0.0662565, 0.066152, 0.0660548, 0.0659678, 0.065891, 0.0658243, 0.0657676, 0.0657209, 0.0656841, 0.0656573, 0.0656404, 0.0656333, 0.0656333, 0.0656461, 0.0656687, 0.0657013, 0.0657438, 0.0657962, 0.0658587, 0.0659312, 0.0660138, 0.0661066, 0.0662096, 0.0663258, 0.0664496, 0.0665838, 0.0667287, 0.0668842, 0.0670506, 0.067228, 0.0674164, 0.0676161, 0.0678273, 0.06805, 0.0682845, 0.0685309, 0.0687895, 0.0690605, 0.0693441, 0.0696405, 0.069953, 0.0702759, 0.0706125, 0.070963, 0.0713277, 0.0717071, 0.0721063, 0.0725159, 0.0729413, 0.0733827, 0.0738406, 0.0743187, 0.0748111, 0.0753214, 0.0758535, 0.0764013, 0.0769687, 0.0775597, 0.0781682, 0.0788003, 0.079456, 0.0801313, 0.080834, 0.0815577, 0.0823106, 0.0830864, 0.0838933, 0.0847288, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0865781, 0.0857394, 0.0849295, 0.0841439, 0.0833887, 0.0826559, 0.0819517, 0.0812682, 0.0806115, 0.0799719, 0.0793563, 0.0787653, 0.0781914, 0.0776375, 0.0771064, 0.0765906, 0.0760932, 0.0756168, 0.0751544, 0.0747088, 0.0742797, 0.0738666, 0.0734703, 0.073088, 0.0727206, 0.0723677, 0.0720291, 0.0717044, 0.0713964, 0.0710987, 0.0708141, 0.0705424, 0.0702834, 0.0700367, 0.0698023, 0.0695799, 0.0693694, 0.0691705, 0.0689832, 0.0688072, 0.0686424, 0.0684887, 0.068346, 0.0682141, 0.0680931, 0.0679856, 0.0678857, 0.0677963, 0.0677174, 0.0676488, 0.0675905, 0.0675425, 0.0675047, 0.0674772 },
//         data5 = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0888318, 0.0873764, 0.085976, 0.0834569, 0.0821931, 0.0809759, 0.0798034, 0.0786737, 0.077585, 0.0765359, 0.0755247, 0.0760868, 0.0769938, 0.0779349, 0.0779349, 0.0789235, 0.0799491, 0.0810134, 0.0821177, 0.0832638, 0.0844534, 0.0856883, 0.0869704, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0826205, 0.0815283, 0.0804755, 0.0795465, 0.080519, 0.080519, 0.0815405, 0.0826004, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
//         data6 = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0871874, 0.0859824, 0.084821, 0.0837017, 0.0826226, 0.0815824, 0.0805797, 0.0805797, 0.0796249, 0.0787047, 0.0778177, 0.0769629, 0.0761393, 0.0753457, 0.0745812, 0.073845, 0.0731361, 0.0724538, 0.0710362, 0.0704193, 0.0698262, 0.0692563, 0.0687089, 0.0681836, 0.0676798, 0.0671969, 0.0667346, 0.0662924, 0.0658698, 0.0654666, 0.0650822, 0.0647165, 0.0643689, 0.0640394, 0.0637276, 0.0628686, 0.0625989, 0.0623459, 0.0621095, 0.0618895, 0.0616857, 0.0609829, 0.0608171, 0.060667, 0.0605327, 0.060414, 0.0598298, 0.059747, 0.0596796, 0.0591667, 0.0591343, 0.0591173, 0.0586727, 0.0586903, 0.0587237, 0.0583451, 0.0584131, 0.0580806, 0.0581835, 0.0578963, 0.0580346, 0.0577924, 0.057576, 0.0577694, 0.0575977, 0.0574515, 0.0573306, 0.0576011, 0.0575257, 0.0574758, -1, -1, 0.0574883, 0.057546, 0.0576294, 0.0573628, 0.057492, 0.0576469, 0.0578275, 0.0576383, 0.0578641, 0.058116, 0.0579824, 0.0582799, 0.0581818, 0.0585253, 0.0584624, 0.0588526, 0.0588248, 0.0588127, 0.0592701, 0.059293, 0.0593316, 0.0598583, 0.0599323, 0.060022, 0.0606215, 0.0607475, 0.0608894, 0.0610472, 0.061221, 0.0619417, 0.0621542, 0.0623831, 0.0626287, 0.0628912, 0.0631707, 0.0640516, 0.0643742, 0.0647149, 0.0650739, 0.0654516, 0.0658483, 0.0662644, 0.0667004, 0.0671564, 0.0676331, 0.0681309, 0.0686501, 0.0691915, 0.0697555, 0.0703427, 0.0709538, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
//         data7 = { 0.0698126, 0.0707035, 0.071628, 0.0725874, 0.073583, 0.0746163, 0.075689, 0.0768026, 0.0771278, 0.0763903, 0.0756804, 0.0742051, 0.0735632, 0.0729461, 0.0723531, 0.0717836, 0.071237, 0.0707129, 0.0702105, 0.0697296, 0.0692696, 0.06883, 0.0684106, 0.0680108, 0.0676304, 0.0672691, 0.0669265, 0.0666023, 0.0657079, 0.0654276, 0.0651648, 0.0649192, 0.0646907, 0.0644792, 0.0637475, 0.0635755, 0.06342, 0.0632809, 0.0631581, 0.0625499, 0.0624645, 0.0623953, 0.0618614, 0.0618286, 0.0618121, 0.0613493, 0.0613688, 0.0614048, 0.061011, 0.0610831, 0.0607373, 0.0608459, 0.0605474, 0.0606929, 0.0604414, 0.0602168, 0.06042, 0.0602421, 0.0600908, 0.0599659, 0.0602496, 0.0601722, 0.0601216, -1, -1, 0.0601406, 0.0602023, 0.0602911, 0.060013, 0.0601498, 0.0603134, 0.060504, 0.060307, 0.060545, 0.0608104, 0.0606715, 0.0609846, 0.060883, 0.0612444, 0.0611796, 0.0615901, 0.0615619, 0.0615504, 0.0620313, 0.0620564, 0.062098, 0.0626517, 0.0627303, 0.0628254, 0.0634555, 0.0635888, 0.0637387, 0.0639052, 0.0640885, 0.064846, 0.0650699, 0.0653111, 0.0655698, 0.0658462, 0.0661405, 0.0670662, 0.0674058, 0.0677643, 0.0681421, 0.0685395, 0.0689569, 0.0693945, 0.0698529, 0.0703324, 0.0708336, 0.0713569, 0.0719029, 0.072472, 0.0730649, 0.0736823, 0.0743247, 0.0749928, 0.0765252, 0.0772647, 0.0780329, 0.0788309, 0.0796596, 0.08052, 0.0814132, 0.0823403, 0.0833025, 0.0843011, 0.0853374, 0.0853374, 0.0854626, 0.0844791, 0.0835312, 0.0826177, 0.0817373, 0.0808891, 0.080072, 0.079285, 0.0785273, 0.077798, 0.0762821, 0.0756226, 0.0749886, 0.0743794, 0.0737942, 0.0732327, 0.0726942, 0.0721781, 0.0716839, 0.0712113, 0.0707597, 0.0703288, 0.0699181, 0.0695273, 0.0691561, 0.0688041, 0.0684711, 0.067552, 0.0672641, 0.0669941, 0.0667419, 0.0665072, 0.06629, 0.0655358, 0.0653592, 0.0651996, 0.0650568, 0.0649308, 0.0643059, 0.0642183, 0.0641473, 0.0635988, 0.0635653, 0.0635484, 0.063073, 0.0630932, 0.0631284, 0.0627239, 0.0627982, 0.0624431, 0.0625549, 0.0622483, 0.0623981, 0.0621398, 0.0619092, 0.0621165, 0.0619339, 0.0617787, 0.0616506, 0.0619424, 0.0618631, 0.0618113, -1, -1, 0.0618301, 0.0618933, 0.0619843, 0.0616983, 0.0618386, 0.0620065, 0.0622022, 0.0620012, 0.0622455, 0.0625181, 0.0623751, 0.0626967, 0.0625921, 0.0629633, 0.0628965, 0.0633181, 0.063291, 0.063279, 0.063773, 0.0637987, 0.0638412, 0.0644101, 0.0644908, 0.0645883, 0.0652357, 0.0653725, 0.0655263, 0.0656973, 0.0658856, 0.0666662, 0.0668961, 0.0671439, 0.0674096, 0.0676936, 0.0679959, 0.0689471, 0.0692959, 0.0696642, 0.0700524, 0.0704606, 0.0708894, 0.071339, 0.0718099, 0.0723026, 0.0728175, 0.0733551, 0.073916, 0.0745008, 0.0751099, 0.0757442, 0.0764042, 0.0770907, 0.0786651, 0.0794248, 0.0802141, 0.0810339, 0.0818853, 0.0827692, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0.0840148, 0.0826448, 0.0813264, 0.0789551, 0.0777649, 0.0766185, 0.0755141, 0.0744498, 0.0734242, 0.0724357, 0.0714829, 0.0705644, 0.0696789, 0.0688252, 0.0688252, 0.0680124, 0.067229, 0.0664739, 0.0657462, 0.0650449, 0.0643692, 0.0637183, 0.0630914, 0.0624878, 0.0619068, 0.0606994, 0.0601741, 0.0596691, 0.0591838, 0.0587177, 0.0582704, 0.0578414, 0.0574302, 0.0570366, 0.0566601, 0.0563004, 0.0559571, 0.0556299, 0.0553186, 0.0550228, 0.0547424, 0.054477, 0.0537451, 0.0535157, 0.0533005, 0.0530995, 0.0529125, 0.0527394, 0.0521423, 0.0520015, 0.0518742, 0.0517602, 0.0516597, 0.0511619, 0.0510919, 0.0510351, 0.0505982, 0.0505713, 0.0505576, 0.0501788, 0.0501947, 0.0502255, 0.0499031, 0.049962, 0.0496789, 0.0497676, 0.0495232, 0.0496422, 0.0494362, 0.0492523, 0.0494197, 0.049274, 0.04915, 0.0490477, 0.0492796, 0.0492161, 0.0491745, -1, -1, 0.0491897, 0.0492404, 0.0493133, 0.049086, 0.0491981, 0.0493322, 0.0494884, 0.0493259, 0.0495208, 0.0497382, 0.0496247, 0.0498811, 0.0497981, 0.050094, 0.0500412, 0.0503772, 0.0503528, 0.0503435, 0.0507371, 0.0507578, 0.050792, 0.0512452, 0.0513097, 0.0513877, 0.0519034, 0.0520126, 0.0521353, 0.0522718, 0.0524219, 0.05304, 0.0532233, 0.0534208, 0.0536326, 0.0538589, 0.0540998, 0.0548574, 0.0551354, 0.0554289, 0.0557381, 0.0560634, 0.056405, 0.0567632, 0.0571384, 0.057531, 0.0579412, 0.0583695, 0.0588164, 0.0592822, 0.0597675, 0.0602728, 0.0607986, 0.0613455, 0.0625997, 0.0632049, 0.0638337, 0.0644869, 0.0651652, 0.0658694, 0.0666005, 0.0673594, 0.068147, 0.0689643, 0.0698126 };
//     getPlot(data, { -0.0600001, -0.06 }, &points, 0);
//     getPlot(data2, { 0.171262, -0.294877 }, &points, 0);
//     getPlot(data3, { 0.0578333, -0.187953 }, &points, 0);
//     getPlot(data4, { 0.0702574, -0.287934 }, &points, 0);
//     getPlot(data5, { 0.0563643, -0.0595838 }, &points, PI / 4);
//     getPlot(data6, { 0.061962, 0.0375973 }, &points, PI / 4);
//     getPlot(data7, { 0.186281, 0.175414 }, &points, 3 * PI / 4);
    
//     pdd cur = { 0.02, 0.06 },
//         tar = { 0.1, -0.05 };
//     pdd curPos = { r2d(getMinMax(points).f.f), r2d(getMinMax(points).s.s)};
//     stack<pdd> route = pointBfs(cur, tar, getMinMax(points));
//     cout << "done" << endl;
//     while (curPos.s >= getMinMax(points).f.s)
//     {
//         while (curPos.f <= getMinMax(points).s.f)
//         {
//             if (isTraversable(curPos, points, rad))
//             {
//                 traversable.push_back(curPos);
//                 if (compPts(curPos, cur))
//                     cout << "O";
//                 else if (compPts(curPos, tar))
//                     cout << "X";
//                 else if (onRoute(route, curPos))
//                     cout << ".";
//                 else
//                     cout << " ";
//             }
//             else
//             {
//                 untraversable.push_back(curPos);
//                 cout << "+";
//             }
           
//             curPos.f += 0.01;
//         }
//         cout << endl;
//         curPos.s -= 0.01;
//         curPos.f = r2d(getMinMax(points).f.f);
//     }
//     cout << curPos.f << " " << curPos.s << endl;
// }