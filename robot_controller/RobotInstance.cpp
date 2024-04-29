#include "RobotInstance.hpp"
#include <math.h>
#include <algorithm>
#include <string.h>
#include <unordered_map>
#include <optional>
#include <stack>
#include <list>
#include <array>
#include "helper.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include "navigation.h"


#include "map.h"
#define _USE_MATH_DEFINES
#include <cmath>

static RobotInstance* instance = NULL;

inline cv::Mat getCv2Mat(webots::Camera *cam)
{
    const uint8_t *img = cam->getImage();
    cv::Mat frame(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)img);

    return frame;
}

cv::Mat RobotInstance::getLeftCameraMat()
{
    return getCv2Mat(m_leftCamera);
}

cv::Mat RobotInstance::getRightCameraMat()
{
    return getCv2Mat(m_rightCamera);
}

RobotInstance::RobotInstance()
{
    m_robot = new webots::Robot();
    m_timestep = m_robot->getBasicTimeStep();

    m_lm = m_robot->getMotor("wheel2 motor");
    m_rm = m_robot->getMotor("wheel1 motor");

    m_lmpos = m_lm->getPositionSensor();
    m_rmpos = m_rm->getPositionSensor();
    m_lmpos->enable(m_timestep);
    m_rmpos->enable(m_timestep);

    m_emitter = m_robot->getEmitter("emitter");

    setPosition(INFINITY);
    forward(0.0);

    m_lidar = m_robot->getLidar("lidar");
    m_lidar->enable(m_timestep);

    m_imu = m_robot->getInertialUnit("inertial_unit");
    m_imu->enable(m_timestep);

    m_color = m_robot->getCamera("colour_sensor");
    m_color->enable(m_timestep);

    m_rightCamera = m_robot->getCamera("camera2");
    m_rightCamera->enable(m_timestep);

    m_leftCamera = m_robot->getCamera("camera3");
    m_leftCamera->enable(m_timestep);

    m_gps = m_robot->getGPS("gps");
    m_gps->enable(m_timestep);

    m_lposoffset = 0;
    m_rposoffset = 0;
    setSpeed(3);

    int res = step();

    if(res == -1)
    {
        std::cout << "failed to step!" << std::endl;
        m_dir = DIR::N;
    }
    else
    {
        m_dir = (DIR)getDirection();
    }

    add_step_callback([this](){
        //navigation update
        this->update_lidar_cloud();
        this->updateVisited();
    });

    m_isFinished = false;
    m_disabledGUI = false;
    m_stopMovement = false;
}

RobotInstance::~RobotInstance()
{
    delete m_robot;
}

const double turn_kp = 0.75;

// turns shortest way to the direction
void RobotInstance::turnTo(double speed, double target_angle)
{
    double current = m_imu->getRollPitchYaw()[2];

    //std::cout << "current: " << current << std::endl;

    double mult = 1.0;

    if(abs(abs(target_angle) - M_PI) < 0.01)
    {
        if(current < 0)
            target_angle = -M_PI;
        else
            target_angle = M_PI;
    }

    while(step() != -1 && abs(current - target_angle) > 0.005)
    {
        current = m_imu->getRollPitchYaw()[2];

        //std::cout << "current: " << current << std::endl;

        if(abs(target_angle - current) > M_PI)
            mult = -1.0;
        else
            mult = 1.0;

        double error = target_angle - current;

        double calc_speed = speed * error * turn_kp * mult;

        //small boost
        if(calc_speed < 0)
            calc_speed -= std::max(0.2, 3.0 * abs(error));
        else
            calc_speed += std::max(0.2, 3.0 * abs(error));

        forward(-calc_speed, calc_speed);
        detectVictims();
    }

    stopMotors();
}

int RobotInstance::step() {
    int ret = m_robot->step(m_timestep);
    if(ret == -1)
        return -1;

    run_callbacks();

    return ret;
}

void RobotInstance::updateTargetPos()
{
    this->m_targetPos = this->calcNextPos();
}

void RobotInstance::turnTo(double speed, DIR dir)
{
    if(dir == m_dir)
        return;

    double angles[4] = { 0.0, -M_PI, M_PI, M_PI };

    //std::cout << "turning to: " << angles[(int)dir] << std::endl;

    turnTo(speed, angles[(int)dir]);

    m_dir = dir;
}

//if this returns -1, then the robot has gone off course
int RobotInstance::getDirection()
{
    double yaw = m_imu->getRollPitchYaw()[2];

    //N E S W
    double errors[4] = { yaw, yaw + M_PI, abs(yaw) - M_PI, yaw - M_PI };

    int result = std::find_if(errors, errors + 4, [](double x) { return abs(x) < 0.1; }) - errors;

    return result == 4 ? -1 : result;
}

bool RobotInstance::alignmentNeeded()
{
    return getDirection() == -1;
}

bool RobotInstance::forwardTicks(double vel, double ticks, pdd target)
{
    double startTime = m_robot->getTime();
    double currentTime = m_robot->getTime();
    pdd start = getRawGPSPosition();
    //TODO: use PID
    double traveled = 0;
    ticks *= 0.97;
    while(traveled <= ticks && step() != -1)
    {
        detectVictims();
        if(blackDetected())
        {
            break;
        }
        forward(vel);
        pdd cur = getRawGPSPosition();
        currentTime = m_robot->getTime();
        if (currentTime > startTime + 5)
        {
            break;
        }
        traveled = hypot(cur.first - start.first, cur.second - start.second);
    }

    stopMotors();

    if(blackDetected())
    {
        std::cout << "black detected" << std::endl;
        pdd cur = getCurrentGPSPosition();
        addLidarPoint(cur);
        addVisited(target);

        while(traveled >= 0.003 && step() != -1)
        {
            forward(-vel * 0.5);
            cur = getRawGPSPosition();
            traveled = hypot(cur.first - start.first, cur.second - start.second);
        }


        stopMotors();

        return false;
    }

    return true;
}

std::vector<std::vector<cv::Point>> RobotInstance::getContours(std::string name, cv::Mat frame)
{
    cv::Mat frame2;
    cv::cvtColor(frame, frame2, cv::COLOR_BGR2GRAY);
    cv::threshold(frame2, frame2, 80, 255, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame2, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat frame3(frame);
    cv::drawContours(frame3, contours, -1, cv::Scalar(255, 0, 0));
    addTexture(name, frame3, SDL_PIXELFORMAT_RGB888);
    return contours;
}

std::vector<std::vector<cv::Point>> RobotInstance::getContours(cv::Mat frame)
{
    cv::Mat frame2;
    cv::cvtColor(frame, frame2, cv::COLOR_BGR2GRAY);
    cv::threshold(frame2, frame2, 80, 255, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame2, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

void RobotInstance::addTexture(std::string name, cv::Mat m, SDL_PixelFormatEnum f)
{
    if(!m_disabledGUI)
        m_tex[name] = getTextureFromMat(renderer, m, f);
}

int RobotInstance::countContours(cv::Mat frame)
{
    return getContours(frame).size();
}

bool RobotInstance::determineLetter(const cv::Mat& roi, std::string side, const double* position) //"l" or "r"
{
    const int rows = roi.rows;
    const int cols = roi.cols;
    if (!(rows >= 30 && rows <= 96 && cols >= 23 && cols <= 74))
    {
        return false;
    }
    const int firstThird = rows / 3;
    const int secondThird = rows * 2 / 3;
    cv::Mat topRoi(roi, cv::Rect(0, 0, cols, firstThird));
    cv::Mat midRoi(roi, cv::Rect(0, firstThird, cols, firstThird));
    cv::Mat bottomRoi(roi, cv::Rect(0, secondThird, cols, firstThird));
    int top = countContours(topRoi);
    int mid = countContours(midRoi);
    int bottom = countContours(bottomRoi);
    int xPos = (int)position[0] * 100;
    int zPos = (int)position[2] * 100;
    char message[9];
    memcpy(&message[0], &xPos, 4);
    memcpy(&message[4], &zPos, 4);
    if (top == 2 && mid == 2 && bottom == 1)
    {
        message[8] = 'U';
    }
    else if (top == 2 && mid == 1 && bottom == 2)
    {
        message[8] = 'H';
    }
    else if (top == 1 && mid == 1 && bottom == 1)
    {
        message[8] = 'S';
    }
    else
    {
        return false;
    }
    std::cout << message[8] << " found" << std::endl;
    m_emitter->send(message, 9);
    return true;
}

void RobotInstance::lookForLetter()
{
    const int horizontalResolution = m_lidar->getHorizontalResolution();
    const float* rangeImage = m_lidar->getRangeImage() + horizontalResolution * 2;
    cv::Rect boundRect;
    cv::Mat frameL = getCv2Mat(m_leftCamera);
    cv::Mat frameR = getCv2Mat(m_rightCamera);
    addTexture("Left Camera", frameL, SDL_PIXELFORMAT_RGB888);
    addTexture("Right Camera", frameR, SDL_PIXELFORMAT_RGB888);
    std::vector<std::vector<cv::Point>> contoursL = getContours("Left Contours", frameL);
    std::vector<std::vector<cv::Point>> contoursR = getContours("Right Contours", frameR);
    if (rangeImage[horizontalResolution * 3 / 4] < MAX_VIC_DETECTION_RANGE)
    {
        for (size_t i = 0; i < contoursL.size(); i++)
        {
            boundRect = boundingRect(contoursL[i]);
            cv::Mat roi(frameL, boundRect);
            addTexture("Left ROI", roi, SDL_PIXELFORMAT_RGB888);
            double rectCenterX = boundRect.x + boundRect.width / 2; //in columns
            double thetaFromStraight;
            if (boundRect.width > 20 && boundRect.height > 20 && boundRect.x != 0 && boundRect.width + boundRect.x < frameL.cols)
            {
                if (determineLetter(roi, "l", m_gps->getValues()))
                {
                    return;
                }
            }
        }
    }
    if (rangeImage[horizontalResolution / 4] < MAX_VIC_DETECTION_RANGE)
    {
        for (size_t i = 0; i < contoursR.size(); i++)
        {
            boundRect = boundingRect(contoursR[i]);
            cv::Mat roi(frameR, boundRect);
            addTexture("Right ROI", roi, SDL_PIXELFORMAT_RGB888);
            if (boundRect.width > 20 && boundRect.height > 20 && boundRect.x != 0 && boundRect.width + boundRect.x < frameR.cols)
            {
                if (determineLetter(roi, "r", m_gps->getValues()))
                {
                    return;
                }
            }
        }
    }
}

void RobotInstance::detectVictims()
{
    try
    {
        lookForLetter();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

}

pdd RobotInstance::calcNextPos()
{
    pdd ret = r2d(chooseMove(this, m_imu->getRollPitchYaw()[2]));
    std::cout << "traversable: " << isTraversable(this, ret, getLidarPoints()) << std::endl;
    return ret;
}

void RobotInstance::moveToNextPos()
{
    pdd nextPos = getTargetPos();
    int xdiff = 100 * r2d(nextPos.first - getCurrentGPSPosition().first);
    int zdiff = 100 * r2d(nextPos.second - getCurrentGPSPosition().second);
    if(zdiff != 0)
    {
        zdiff = copysign(1, zdiff);
    }
    if(xdiff > 0)
    {
        switch(zdiff)
        {
            case 1: turnTo(3, -M_PI / 4); forwardTicks(getSpeed(), 0.95 * hypot(0.01, 0.01), nextPos); break;
            case 0: turnTo(3, -M_PI / 2); forwardTicks(getSpeed(), 0.0095, nextPos); break;
            case -1: turnTo(3, -M_PI * 3 / 4); forwardTicks(getSpeed(), 0.95 * hypot(0.01, 0.01), nextPos); break;
        }
    }
    else if(xdiff == 0)
    {
        switch(zdiff)
        {
            case 1: turnTo(3, 0); forwardTicks(getSpeed(), 0.0095, nextPos); break;
            case -1: turnTo(3, M_PI); forwardTicks(getSpeed(), 0.0095, nextPos); break;
        }
    }
    else
    {
        switch(zdiff)
        {
            case 1: turnTo(3, M_PI / 4); forwardTicks(getSpeed(), 0.95 * hypot(0.01, 0.01), nextPos); break;
            case 0: turnTo(3, M_PI / 2); forwardTicks(getSpeed(), 0.0095, nextPos); break;
            case -1: turnTo(3, M_PI * 3 / 4); forwardTicks(getSpeed(), 0.95 * hypot(0.01, 0.01), nextPos); break;
        }
    }
}

void RobotInstance::updateVisited()
{
    pdd cur = r2d(getCurrentGPSPosition());
    if(!isVisited(cur))
    {
        addVisited(cur);
    }
    if(cur != m_lastPos)
    {
        const double radius = 0.07;

        double x = cur.first - radius, y = cur.second - radius;
        for(; x <= cur.first + radius; x += 0.008)
        {
            for(y = cur.second - radius; y <= cur.second + radius; y += 0.008)
            {
                pdd point = r2d(pdd(x, y));
                if(point == pointTo(cur, m_imu->getRollPitchYaw()[2]))
                {
                    continue;
                }
                if(!isVisited(point) && canSee(cur, point, getLidarPoints()))
                {
                    if(getDist(cur, point) <= 0.05)
                    {
                        addVisited(point);
                    }
                    else if(!isInToVisit(point))
                    {
                        addToVisit(point);
                    }
                }
            }
        }
    }
    m_lastPos = r2d(getCurrentGPSPosition());
}

bool RobotInstance::blackDetected()
{
    const uint8_t* colors = m_color->getImage();

    // std::cout << "R: " << (int)colors[0] << " G: " << (int)colors[1] << " B: " << (int)colors[2] << std::endl;

    if(colors[0] <= 55 && colors[1] <= 55 && colors[2] <= 55)
        return true;

    return false;
}

const uint8_t* RobotInstance::getColor()
{
    return m_color->getImage();
}

void RobotInstance::alignRobot()
{
    std::cout << "alignment todo!" << std::endl;
}

RobotInstance* RobotInstance::getInstance()
{
    if(!instance)
        instance = new RobotInstance();
    return instance;
}

void RobotInstance::destroyInstance()
{
    if(instance)
    {
        delete instance;
    }
    instance = NULL;
}

void RobotInstance::update_lidar_cloud()
{
    update_regions_map(m_gps, m_lidar->getRangeImage() + 512, m_imu->getRollPitchYaw()[2]);
    update_camera_map(m_gps, m_lidar->getRangeImage() + 512, m_leftCamera, m_imu->getRollPitchYaw()[2]);
}

void RobotInstance::run_callbacks()
{
    for(const auto& callback : m_callbacks)
    {
        callback();
    }
}

REGION* RobotInstance::get_current_region()
{
    return get_region(m_gps);
}