#pragma once
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <vector>
#include<utility>
#include "navigation.h"
#include <cmath>
#include <iostream>
#include <string>
#include <cstring>
#define pdd std::pair<double,double>
void col(webots::Camera* colorsensor, webots::GPS* gps, webots::InertialUnit* imu, const pdd& startpos, int sign);
void send(std::vector<pdd>& pList, webots::Emitter* emitter, const pdd& startpos, webots::Robot* rb);
void show(std::vector<pdd>& pList, webots::Emitter* emitter, const pdd& startpos, webots::Robot* rb);