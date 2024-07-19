#include "mapping.h"
#include <webots/Camera.hpp>
#include <webots/Emitter.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include "RobotInstance.hpp"
#include <vector>
#include<utility>
#include "navigation.h"
#include <cmath>
#include <iostream>
#include <string>
#include <cstring>
#include <set>
#include <unordered_set>
#include <map>
#include <queue>

void otherdelay(int ms, webots::Robot* rb)
{
    int timeStep = (int)rb->getBasicTimeStep();
    double init = rb->getTime();
    while (rb->step(timeStep) != -1)
    {
        if ((rb->getTime() - init) * 1000 > ms) break;
    }
}
const std::vector<std::string> colored = { "5", "6", "7", "8", "9", "o", "y", "p", "r", "g", "b" };
std::map<pii, std::string> tilemap;
std::map<pii, int> roommap;
std::unordered_set<pii, pair_hash_combiner<int>> room4;
pii r4passage = pii(-1000, -1000);
int minXrelativetostart = 0, minYrelativetostart = 0, maxXrelativetostart = 0, maxYrelativetostart = 0;
int room = 1;
std::string lastcolor = "0";
//blue 1->2
//yellow 1->3
//green 1->4
//purp 2->3
//orange 2->4
//red 3->4
void setroom(std::string tile, int r1, int r2)
{
    if (room == r1 && lastcolor != tile) { room = r2; }
    else if (room == r2 && lastcolor != tile) { room = r1; }
    lastcolor = tile;
}
int getRoom(const pdd& point, const pdd& startpos)
{
    pii coords_int = pii(int(round((point.first - startpos.first) / TILE_LENGTH)), -int(round((point.second - startpos.second) / TILE_LENGTH)));
    if (roommap.count(coords_int))
    {
        return roommap[coords_int];
    }
    return 0;
}
void send(std::vector<pdd>& pList, webots::Emitter* emitter, const pdd& startpos, webots::Robot* rb)
{
    std::cout << "room: " << room << std::endl;
    double minX = getMinMax(pList).first.first, minY = getMinMax(pList).first.second, maxX = getMinMax(pList).second.first, maxY = getMinMax(pList).second.second;
    double w = maxX - minX, h = maxY - minY;
    int arrW = int(round(w / TILE_LENGTH)) * 4 + 1, arrH = int(round(h / TILE_LENGTH)) * 4 + 1;
    std::vector<std::vector<std::string>>map(arrH, std::vector<std::string>(arrW, "0"));
    double startX = startpos.first - minX - 0.06, startY = maxY - startpos.second - 0.06;
    int startTileX = int(round(startX / TILE_LENGTH)) * 4, startTileY = int(round(startY / TILE_LENGTH)) * 4;
    int worldH = int(round(h / TILE_LENGTH)), worldW = int(round(w / TILE_LENGTH));
    //bounds checking
    if (startTileX < 0) startTileX = 0;
    if (startTileY < 0) startTileY = 0;
    if (startTileX > arrW - 5) startTileX = arrW - 5;
    if (startTileY > arrH - 5) startTileY = arrH - 5;
    std::cout << "min x:" << minX << " " << "min y:" << minY << std::endl;
    std::cout << "max x:" << maxX << " " << "max y:" << maxY << std::endl;
    std::cout << "start x:" << startpos.first << " " << "start y:" << startpos.second << std::endl;
    for (int i = 0; i < worldH; i++)
    {
        for (int j = 0; j < worldW; j++)
        {
            for (int y = 0; y < 5; y++)
            {
                for (int x = 0; x < 5; x++)
                {
                    int yidx = 4 * i + y, xidx = 4 * j + x;
                    double checkX = minX + TILE_LENGTH * j + x * 0.03, checkY = maxY - TILE_LENGTH * i - y * 0.03;
                    if ((x == 1 || x == 3) && (y == 1 || y == 3))
                    {
                        if (!isTraversable(pdd(checkX - 0.01, checkY + 0.01), pList, 0.005)) map[yidx][xidx] = "w";
                        if (!isTraversable(pdd(checkX + 0.01, checkY + 0.01), pList, 0.005)) map[yidx][xidx] = "x";
                        if (!isTraversable(pdd(checkX - 0.01, checkY - 0.01), pList, 0.005)) map[yidx][xidx] = "y";
                        if (!isTraversable(pdd(checkX + 0.01, checkY - 0.01), pList, 0.005)) map[yidx][xidx] = "z";
                    }
                    else
                    {
                        if (!isTraversable(pdd(checkX, checkY), pList, 0.015)) map[yidx][xidx] = "1";
                    }
                }
            }
        }
    }
    for (int i = 0; i < worldH; i++)
    {
        for (int j = 0; j < worldW; j++)
        {
            for (int y = 0; y < 5; y++)
            {
                for (int x = 0; x < 5; x++)
                {
                    int yidx = 4 * i + y, xidx = 4 * j + x;

                    if (map[yidx][xidx] == "w")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx - 1][xidx - 1] = "0";
                    }
                    if (map[yidx][xidx] == "x")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx - 1][xidx + 1] = "0";
                    }
                    if (map[yidx][xidx] == "y")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx + 1][xidx - 1] = "0";
                    }
                    if (map[yidx][xidx] == "z")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx + 1][xidx + 1] = "0";
                    }
                }
            }
        }
    }
    for (const auto& tile : tilemap)
    {
        pii pos = pii(tile.first.first, tile.first.second);
        int y = (pos.second + startTileY / 4) * 4, x = (pos.first + startTileX / 4) * 4;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x > arrW - 5) x = arrW - 5;
        if (y > arrH - 5) y = arrH - 5;
        std::string val = tile.second;
        if (count(colored.begin(), colored.end(), val) || val == "2" || roommap[pos] == 1)
        {
            map[y + 1][x + 2] = "0";
            map[y + 2][x + 1] = "0";
            map[y + 2][x + 2] = "0";
            map[y + 2][x + 3] = "0";
            map[y + 3][x + 2] = "0";
        }
        map[y + 1][x + 1] = val;
        map[y + 1][x + 3] = val;
        map[y + 3][x + 1] = val;
        map[y + 3][x + 3] = val;
    }
    for (const auto& v : getVictims())
    {
        //(std::make_pair(std::make_pair(pdd(position[0], position[2]), side), m_imu->getRollPitchYaw()[2]))
        //first.first.first - x
        //first.first.second - y
        //first.second - side
        //second - angle
        pdd victim_pos = v.first;
        int victimX = int(round(((victim_pos.first - minX) / 0.03))), victimY = int(round(((maxY - victim_pos.second) / 0.03)));
        if (victimY >= arrH) victimY = arrH - 1;
        if (victimX >= arrW) victimX = arrW - 1;
        map[victimY][victimX] = v.second;
    }
    int minXroom4 = 1000, minYroom4 = 1000, maxXroom4 = -1000, maxYroom4 = -1000;
    for (pii tile : room4)
    {
        int y = (tile.second + startTileY / 4) * 4, x = (tile.first + startTileX / 4) * 4;
        minXroom4 = std::min(tile.first, minXroom4);
        maxXroom4 = std::max(tile.first, maxXroom4);
        minYroom4 = std::min(tile.second, minYroom4);
        maxYroom4 = std::max(tile.second, maxYroom4);
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x > arrW - 5) x = arrW - 5;
        if (y > arrH - 5) y = arrH - 5;
        //std::cout << x << " " << y << std::endl;
        if (count(colored.begin(), colored.end(), tilemap[tile]) == 0)
        {
            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    map[y + i][x + j] = "*";
                }
            }
        }

    }

    std::cout << minXroom4 << " " << maxXroom4 << " " << minYroom4 << " " << maxYroom4 << " " << std::endl; 

    if (r4passage.first != -1000 && !room4.empty())
    {
        //flood fill
        pii tile = *room4.begin();
        std::queue<pii> q;
        std::map<pii, bool> bfsvisited;
        q.push(tile);
        bfsvisited[tile] = 1;
        while (!q.empty())
        {
            tile = q.front();
            //std::cout << "front: " << tile.first << " " << tile.second << std::endl;
            q.pop();
            //add stars to map array
            int y = (tile.second + startTileY / 4) * 4, x = (tile.first + startTileX / 4) * 4;
            if (x < 0) x = 0;
            if (y < 0) y = 0;
            if (x > arrW - 5) x = arrW - 5;
            if (y > arrH - 5) y = arrH - 5;
            if (count(colored.begin(), colored.end(), tilemap[tile]) == 0)
            {
                for (int i = 0; i < 5; i++)
                {
                    for (int j = 0; j < 5; j++)
                    {
                        map[y + i][x + j] = "*";
                    }
                }
                //push viable neighbors to queue
            //right
                if (tile.first < maxXrelativetostart && (!roommap.count(pii(tile.first + 1, tile.second)) || (roommap.count(pii(tile.first + 1, tile.second)) && roommap[pii(tile.first + 1, tile.second)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first + 1, tile.second)) && tile.first < maxXroom4)
                    {
                        q.push(pii(tile.first + 1, tile.second));
                        bfsvisited[pii(tile.first + 1, tile.second)] = 1;
                    }
                }
                //up
                if (tile.second > minYrelativetostart && (!roommap.count(pii(tile.first, tile.second - 1)) || (roommap.count(pii(tile.first, tile.second - 1)) && roommap[pii(tile.first, tile.second - 1)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first, tile.second - 1)) && tile.second > minYroom4)
                    {
                        q.push(pii(tile.first, tile.second - 1));
                        bfsvisited[pii(tile.first, tile.second - 1)] = 1;
                    }
                }
                //down
                if (tile.second < maxYrelativetostart && (!roommap.count(pii(tile.first, tile.second + 1)) || (roommap.count(pii(tile.first, tile.second + 1)) && roommap[pii(tile.first, tile.second + 1)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first, tile.second + 1)) && tile.second < maxYroom4)
                    {
                        q.push(pii(tile.first, tile.second + 1));
                        bfsvisited[pii(tile.first, tile.second + 1)] = 1;
                    }
                }
                //left
                if (tile.first > minXrelativetostart && (!roommap.count(pii(tile.first - 1, tile.second)) || (roommap.count(pii(tile.first - 1, tile.second)) && roommap[pii(tile.first - 1, tile.second)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first - 1, tile.second)) && tile.first > minXroom4)
                    {
                        q.push(pii(tile.first - 1, tile.second));
                        bfsvisited[pii(tile.first - 1, tile.second)] = 1;
                    }
                }
            }
        }

    }
    map[startTileY + 1][startTileX + 1] = "5";
    map[startTileY + 1][startTileX + 3] = "5";
    map[startTileY + 3][startTileX + 1] = "5";
    map[startTileY + 3][startTileX + 3] = "5";
    map[startTileY + 1][startTileX + 2] = "0";
    map[startTileY + 2][startTileX + 1] = "0";
    map[startTileY + 2][startTileX + 2] = "0";
    map[startTileY + 2][startTileX + 3] = "0";
    map[startTileY + 3][startTileX + 2] = "0";
    std::cout << "COME AND GET IT" << std::endl;
    std::string flattened = "";
    //int width = map[0].size(),height=map.size();
    for (int i = 0; i < arrH; i++) {
        for (int j = 0; j < arrW; j++) {
            std::cout << map[i][j] + ",";
            flattened += map[i][j] + ","; // Flatten the array with comma separators
        }
        std::cout << std::endl;
    }
    flattened.pop_back(); // Remove the last unnecessary comma
    //std::cout << flattened.c_str() << std::endl;
    char* message = (char*)malloc(flattened.size() + 8);
    memcpy(&message[0], &arrH, sizeof(arrH)); // The first 2 integers in the message array are width, height
    memcpy(&message[4], &arrW, sizeof(arrW));
    memcpy(&message[8], flattened.c_str(), flattened.size()); // Copy in the flattened map afterwards
    emitter->send(message, flattened.size() + 8); // Send map data
    char msg = 'M'; // Send map evaluate request
    emitter->send(&msg, sizeof(msg));
    msg = 'E'; // Send an Exit message to get Map Bonus
    emitter->send(&msg, sizeof(msg));
    rb->step(rb->getBasicTimeStep());
}
void col(webots::Camera* colorsensor, webots::GPS* gps, webots::InertialUnit* imu, const pdd& startpos, int sign)
{
    bool passage = 0;
    const uint8_t* colors = colorsensor->getImage();
    int b = colors[0], g = colors[1], r = colors[2];
    double pos[3];
    memcpy(pos, gps->getValues(), 3 * sizeof(double));
    double angle = imu->getRollPitchYaw()[2];
    //pdd coords = pdd(pos[0]-0.06*sin(angle)-getMinMax(pList).first.first, pos[2] - 0.06 * cos(angle)- getMinMax(pList).first.second);
    //pii coords_int = pii(int(floor(coords.first / TILE_LENGTH)), int(ceil(coords.second / TILE_LENGTH)));
    pdd coords = pdd(pos[0] - 0.0175 * sin(angle) * sign, -pos[2] + 0.0175 * cos(angle) * sign);

    pii coords_int = pii(int(round((coords.first - startpos.first) / TILE_LENGTH)), -int(round((coords.second - startpos.second) / TILE_LENGTH)));
    //std::cout << coords_int.first << " " << coords_int.second << std::endl;
    minXrelativetostart = std::min(coords_int.first, minXrelativetostart);
    minYrelativetostart = std::min(coords_int.second, minYrelativetostart);
    maxXrelativetostart = std::max(coords_int.first, maxXrelativetostart);
    maxYrelativetostart = std::max(coords_int.second, maxYrelativetostart);
    //std::cout << "b: " << b << "g: " << g << "r: " << r << std::endl;
    /*std::cout << "startpos: " << startpos.first << " " << startpos.second << std::endl;
    std::cout << coords.first << " " << coords.second << std::endl;
    std::cout << coords_int.first << " " << coords_int.second << std::endl;*/
    //std::cout << coords.first << " " << coords.second << std::endl;
    //std::cout << room << std::endl;
    //blue 1->2
//yellow 1->3
//green 1->4
//purp 2->3
//orange 2->4
//red 3->4
    if (b <= 55 && g <= 55 && r <= 55)
    {
        //black hole
        //std::cout << "black hole" << std::endl;
        tilemap[coords_int] = "2";
        //lastcolor = "2";
    }
    else if (b >= 250 && g <= 80 && r <= 80)
    {
        //blue 
        //std::cout << "blue" << std::endl;
        tilemap[coords_int] = "b";
        setroom("b", 1, 2);
        passage = 1;
    }
    else if (b <= 80 && g >= 250 && r <= 80)
    {
        //green
        //std::cout << "green" << std::endl;
        tilemap[coords_int] = "g";
        setroom("g", 1, 4);
        passage = 1;
        r4passage = coords_int;
    }
    else if (b <= 80 && g <= 80 && r >= 250)
    {
        //red
        //std::cout << "red" << std::endl;
        tilemap[coords_int] = "r";
        setroom("r", 3, 4);
        passage = 1;
        r4passage = coords_int;
    }
    else if (b <= 80 && g >= 250 && r >= 250)
    {
        //yellow
        //std::cout << "yellow" << std::endl;
        tilemap[coords_int] = "y";
        setroom("y", 1, 3);
        passage = 1;
    }
    else if (b <= 80 && g >= 235 && r >= 250)
    {
        //orange
        //std::cout << "orange" << std::endl;
        tilemap[coords_int] = "o";
        setroom("o", 2, 4);
        passage = 1;
        r4passage = coords_int;
    }
    else if (b >= 235 && g <= 80 && r >= 150)
    {
        //purple
        //std::cout << "purp" << std::endl;
        tilemap[coords_int] = "p";
        setroom("p", 2, 3);
        passage = 1;
    }
    else if (b <= 150 && g >= 190 && r >= 220)
    {
        //swamp
        //std::cout << "swamp" << std::endl;
        tilemap[coords_int] = "3";
        lastcolor = "3";
    }
    else if (b >= 254 && g >= 254 && r >= 254)
    {
        //swamp
        //std::cout << "cp" << std::endl;
        tilemap[coords_int] = "4";
        lastcolor = "4";
    }
    else
    {
        //std::cout << "reg tile" << std::endl;
        tilemap[coords_int] = "0";
        lastcolor = "0";
    }
    if (!passage)
    {
        if (roommap.count(coords_int))
        {
            room = roommap[coords_int];
        }
        else
        {
            roommap[coords_int] = room;
            if (room == 4) room4.insert(coords_int);
        }
    }
    //black: 50 50 50
    //swamp:119 198 229
    //blue: 254 75 75
    //purple: 241 75 167
    //red: 75 75 254
    //green: 40 254 40
    //gold: 78 243 255
    //yellow: 77 255 255
}


void show(std::vector<pdd>& pList, webots::Emitter* emitter, const pdd& startpos, webots::Robot* rb)
{
    std::cout << "room: " << room << std::endl;
    double minX = getMinMax(pList).first.first, minY = getMinMax(pList).first.second, maxX = getMinMax(pList).second.first, maxY = getMinMax(pList).second.second;
    double w = maxX - minX, h = maxY - minY;
    int arrW = int(round(w / TILE_LENGTH)) * 4 + 1, arrH = int(round(h / TILE_LENGTH)) * 4 + 1;
    std::vector<std::vector<std::string>>map(arrH, std::vector<std::string>(arrW, "0"));
    double startX = startpos.first - minX - 0.06, startY = maxY - startpos.second - 0.06;
    int startTileX = int(round(startX / TILE_LENGTH)) * 4, startTileY = int(round(startY / TILE_LENGTH)) * 4;
    int worldH = int(round(h / TILE_LENGTH)), worldW = int(round(w / TILE_LENGTH));
    //bounds checking
    if (startTileX < 0) startTileX = 0;
    if (startTileY < 0) startTileY = 0;
    if (startTileX > arrW - 5) startTileX = arrW - 5;
    if (startTileY > arrH - 5) startTileY = arrH - 5;
    std::cout << "min x:" << minX << " " << "min y:" << minY << std::endl;
    std::cout << "max x:" << maxX << " " << "max y:" << maxY << std::endl;
    std::cout << "start x:" << startpos.first << " " << "start y:" << startpos.second << std::endl;
    for (int i = 0; i < worldH; i++)
    {
        for (int j = 0; j < worldW; j++)
        {
            for (int y = 0; y < 5; y++)
            {
                for (int x = 0; x < 5; x++)
                {
                    int yidx = 4 * i + y, xidx = 4 * j + x;
                    double checkX = minX + TILE_LENGTH * j + x * 0.03, checkY = maxY - TILE_LENGTH * i - y * 0.03;
                    if ((x == 1 || x == 3) && (y == 1 || y == 3))
                    {
                        if (!isTraversable(pdd(checkX - 0.01, checkY + 0.01), pList, 0.005)) map[yidx][xidx] = "w";
                        if (!isTraversable(pdd(checkX + 0.01, checkY + 0.01), pList, 0.005)) map[yidx][xidx] = "x";
                        if (!isTraversable(pdd(checkX - 0.01, checkY - 0.01), pList, 0.005)) map[yidx][xidx] = "y";
                        if (!isTraversable(pdd(checkX + 0.01, checkY - 0.01), pList, 0.005)) map[yidx][xidx] = "z";
                    }
                    else
                    {
                        if (!isTraversable(pdd(checkX, checkY), pList, 0.015)) map[yidx][xidx] = "1";
                    }
                }
            }
        }
    }
    for (int i = 0; i < worldH; i++)
    {
        for (int j = 0; j < worldW; j++)
        {
            for (int y = 0; y < 5; y++)
            {
                for (int x = 0; x < 5; x++)
                {
                    int yidx = 4 * i + y, xidx = 4 * j + x;

                    if (map[yidx][xidx] == "w")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx - 1][xidx - 1] = "0";
                    }
                    if (map[yidx][xidx] == "x")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx - 1][xidx + 1] = "0";
                    }
                    if (map[yidx][xidx] == "y")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx + 1][xidx - 1] = "0";
                    }
                    if (map[yidx][xidx] == "z")
                    {
                        map[yidx][xidx] = "0";
                        map[yidx + 1][xidx + 1] = "0";
                    }
                }
            }
        }
    }
    for (const auto& tile : tilemap)
    {
        pii pos = pii(tile.first.first, tile.first.second);
        int y = (pos.second + startTileY / 4) * 4, x = (pos.first + startTileX / 4) * 4;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x > arrW - 5) x = arrW - 5;
        if (y > arrH - 5) y = arrH - 5;
        std::string val = tile.second;
        if (count(colored.begin(), colored.end(), val) || val == "2")
        {
            map[y + 1][x + 2] = "0";
            map[y + 2][x + 1] = "0";
            map[y + 2][x + 2] = "0";
            map[y + 2][x + 3] = "0";
            map[y + 3][x + 2] = "0";
        }
        map[y + 1][x + 1] = val;
        map[y + 1][x + 3] = val;
        map[y + 3][x + 1] = val;
        map[y + 3][x + 3] = val;
    }
    for (const auto& v : getVictims())
    {
        //(std::make_pair(std::make_pair(pdd(position[0], position[2]), side), m_imu->getRollPitchYaw()[2]))
        //first.first.first - x
        //first.first.second - y
        //first.second - side
        //second - angle
        pdd victim_pos = v.first;
        int victimX = int(round(((victim_pos.first - minX) / 0.03))), victimY = int(round(((maxY - victim_pos.second) / 0.03)));
        if (victimY >= arrH) victimY = arrH - 1;
        if (victimX >= arrW) victimX = arrW - 1;
        map[victimY][victimX] = v.second;
    }
    for (pii tile : room4)
    {
        int y = (tile.second + startTileY / 4) * 4, x = (tile.first + startTileX / 4) * 4;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x > arrW - 5) x = arrW - 5;
        if (y > arrH - 5) y = arrH - 5;
        //std::cout << x << " " << y << std::endl;
        if (count(colored.begin(), colored.end(), tilemap[tile]) == 0)
        {
            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    map[y + i][x + j] = "*";
                }
            }
        }

    }
    if (r4passage.first != -1000 && !room4.empty())
    {
        //flood fill
        pii tile = *room4.begin();
        std::queue<pii> q;
        std::map<pii, bool> bfsvisited;
        q.push(tile);
        bfsvisited[tile] = 1;
        while (!q.empty())
        {
            tile = q.front();
            //std::cout << "front: " << tile.first << " " << tile.second << std::endl;
            q.pop();
            //add stars to map array
            int y = (tile.second + startTileY / 4) * 4, x = (tile.first + startTileX / 4) * 4;
            if (x < 0) x = 0;
            if (y < 0) y = 0;
            if (x > arrW - 5) x = arrW - 5;
            if (y > arrH - 5) y = arrH - 5;
            if (count(colored.begin(), colored.end(), tilemap[tile]) == 0)
            {
                for (int i = 0; i < 5; i++)
                {
                    for (int j = 0; j < 5; j++)
                    {
                        map[y + i][x + j] = "*";
                    }
                }
                //push viable neighbors to queue
            //right
                if (tile.first < maxXrelativetostart && (!roommap.count(pii(tile.first + 1, tile.second)) || (roommap.count(pii(tile.first + 1, tile.second)) && roommap[pii(tile.first + 1, tile.second)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first + 1, tile.second)))
                    {
                        q.push(pii(tile.first + 1, tile.second));
                        bfsvisited[pii(tile.first + 1, tile.second)] = 1;
                    }
                }
                //up
                if (tile.second > minYrelativetostart && (!roommap.count(pii(tile.first, tile.second - 1)) || (roommap.count(pii(tile.first, tile.second - 1)) && roommap[pii(tile.first, tile.second - 1)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first, tile.second - 1)))
                    {
                        q.push(pii(tile.first, tile.second - 1));
                        bfsvisited[pii(tile.first, tile.second - 1)] = 1;
                    }
                }
                //down
                if (tile.second < maxYrelativetostart && (!roommap.count(pii(tile.first, tile.second + 1)) || (roommap.count(pii(tile.first, tile.second + 1)) && roommap[pii(tile.first, tile.second + 1)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first, tile.second + 1)))
                    {
                        q.push(pii(tile.first, tile.second + 1));
                        bfsvisited[pii(tile.first, tile.second + 1)] = 1;
                    }
                }
                //left
                if (tile.first > minXrelativetostart && (!roommap.count(pii(tile.first - 1, tile.second)) || (roommap.count(pii(tile.first - 1, tile.second)) && roommap[pii(tile.first - 1, tile.second)] == 4)))
                {
                    if (!bfsvisited.count(pii(tile.first - 1, tile.second)))
                    {
                        q.push(pii(tile.first - 1, tile.second));
                        bfsvisited[pii(tile.first - 1, tile.second)] = 1;
                    }
                }
            }
        }

    }
    map[startTileY + 1][startTileX + 1] = "5";
    map[startTileY + 1][startTileX + 3] = "5";
    map[startTileY + 3][startTileX + 1] = "5";
    map[startTileY + 3][startTileX + 3] = "5";
    map[startTileY + 1][startTileX + 2] = "0";
    map[startTileY + 2][startTileX + 1] = "0";
    map[startTileY + 2][startTileX + 2] = "0";
    map[startTileY + 2][startTileX + 3] = "0";
    map[startTileY + 3][startTileX + 2] = "0";
    for (int i = 0; i < arrH; i++) {
        for (int j = 0; j < arrW; j++) {
            std::cout << map[i][j] + ",";
        }
        std::cout << std::endl;
    }
}

void insert_tile(std::string type, webots::Camera* colorsensor, webots::GPS* gps, webots::InertialUnit* imu, const pdd& startpos)
{
    double pos[3];
    memcpy(pos, gps->getValues(), 3 * sizeof(double));
    double angle = imu->getRollPitchYaw()[2];
    //pdd coords = pdd(pos[0]-0.06*sin(angle)-getMinMax(pList).first.first, pos[2] - 0.06 * cos(angle)- getMinMax(pList).first.second);
    //pii coords_int = pii(int(floor(coords.first / TILE_LENGTH)), int(ceil(coords.second / TILE_LENGTH)));
    pdd coords = pdd(pos[0] - 0.025 * sin(angle), -pos[2] + 0.025 * cos(angle));

    pii coords_int = pii(int(round((coords.first - startpos.first) / TILE_LENGTH)), -int(round((coords.second - startpos.second) / TILE_LENGTH)));
    tilemap[coords_int] = type;
    roommap[coords_int] = room;
}