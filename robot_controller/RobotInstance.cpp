#include "RobotInstance.hpp"
#include <math.h>
#include <mapping.h>
#include <algorithm>
#include <string.h>
#include <unordered_map>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stack>
#include <list>
#include <array>
#include "helper.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include "navigation.h"


#include "map.h"
#include "mapping.h"
#define _USE_MATH_DEFINES
#include <cmath>

std::map<pdd, char> victimMap;

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

cv::Mat training_data;
std::vector<int> output;

void load_training_data(cv::ml::KNearest *knn)
{
    std::ifstream feature("feature.txt");
    std::ifstream label("label.txt");

    if(!feature.is_open() || !label.is_open())
    {
        std::cerr << "failed to open training data!" << std::endl;
        return;
    }

    for(;;)
    {
        if(feature.eof())
            break;
        std::string line;
        std::getline(feature, line);
        if(line.empty())
            break;

        cv::Mat data;
        std::istringstream iss(line);
        for(;;)
        {
            float val;
            iss >> val;
            if(iss.fail())
                break;
            data.push_back(val);
        }
        data = data.reshape(1, 1);
        if(training_data.rows == 0)
            training_data.push_back(data);
        else
            cv::vconcat(training_data, data, training_data);
    }

    for(;;)
    {
        if(label.eof())
            break;
        float val;
        label >> val;
        if(label.fail())
            break;
        output.push_back(val);
    }

    knn->train(training_data, cv::ml::ROW_SAMPLE, output);
}

void RobotInstance::save_training_data()
{
    std::ofstream feature("feature.txt");
    std::ofstream label("label.txt");

    if(!feature.is_open() || !label.is_open())
    {
        std::cerr << "failed to create training data!" << std::endl;
        return;
    }

    // training_data.forEach<cv::Mat>([&](cv::Mat m){
    //     for(int i = 0; i < m.rows; i++)
    //     {
    //         feature << m.at<float>(i) << " ";
    //     }
    //     feature << std::endl;
    // });

    // for(const auto& val : output)
    // {
    //     label << val << std::endl;
    // }

    // output.forEach<double>([&](double val){
    //     label << val << std::endl;
    // });

    for(int i = 0; i < training_data.rows; i++)
    {
        for(int j = 0; j < training_data.cols; j++)
        {
            feature << training_data.at<float>(i, j) << " ";
        }
        feature << std::endl;
    }

    for(auto &v : output)
    {
        label << v << std::endl;
    }
}

void RobotInstance::add_training_data(std::string side, char classification)
{
    if(classification == 'H' || classification == 'S' || classification == 'U')
    {
        cv::Mat frame;
        if(side == "L")
        {
            frame = getLeftCameraMat();
        }
        else if(side == "R")
        {
            frame = getRightCameraMat();
        }

        auto contour = getContour(frame);
        if(contour.size() == 0)
        {
            // std::cout << "no contour found!" << std::endl;
            return;
        }

        cv::Rect boundRect = boundingRect(contour);
        cv::Mat frame2;
        cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
        cv::inRange(frame, cv::Scalar(0,0,0), cv::Scalar(255,20,80), frame2);
        cv::Mat _roi(frame2, boundRect);
        cv::Mat roi = _roi.clone();
        cv::Mat roi3;
        cv::resize(roi, roi3, cv::Size(10, 10));
        cv::threshold(roi3, roi3, 127, 255, cv::THRESH_BINARY);
        cv::Mat roi4;
        roi3.convertTo(roi4, CV_32FC1);
        cv::Mat roi2 = roi4.reshape(1, 1);
        if(training_data.rows == 0)
            training_data.push_back(roi2);
        else
            cv::vconcat(training_data, roi2, training_data);
        output.push_back((int)classification);
    }
    else
    {
        cv::Mat frame;
        if(side == "L")
        {
            frame = getLeftCameraMat();
        }
        else if(side == "R")
        {
            frame = getRightCameraMat();
        }

        auto contour = getContourHazard("", frame);

        if(contour.size() == 0)
        {
            // std::cout << "no contour found!" << std::endl;
            return;
        }

        cv::Rect boundRect = boundingRect(contour);
        cv::Mat roi2(frame, boundRect);
        cv::Mat roi1;
        cv::cvtColor(roi2, roi1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(roi2, roi2, cv::COLOR_BGR2HSV);

        cv::Mat roi3;
        cv::inRange(roi2, cv::Scalar(0, 0, 0), cv::Scalar(255, 20, 255), roi3);

        cv::bitwise_and(roi1, roi3, roi1);

        cv::threshold(roi1, roi1, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        cv::Mat Roi1D;
        cv::resize(roi1, Roi1D, cv::Size(10, 10));
        cv::threshold(Roi1D, Roi1D, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        addTexture("Hazard threshold " + side, Roi1D.clone(), SDL_PIXELFORMAT_RGB332);
        cv::Mat Roi1D3;
        Roi1D.convertTo(Roi1D3, CV_32F);
        cv::Mat Roi1D2 = Roi1D3.reshape(1, 1);

        if(training_data.rows == 0)
            training_data.push_back(Roi1D2);
        else
            cv::vconcat(training_data, Roi1D2, training_data);
        output.push_back((int)classification);
    }

    m_knn->train(training_data, cv::ml::ROW_SAMPLE, output);
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
    m_receiver = m_robot->getReceiver("receiver");
    m_receiver->enable(m_timestep);

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

    isFollowingVictim = false;
    reporting = false;
    m_disableEmit = false;

    int res = m_robot->step(m_timestep);

    starttime = time(0);

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

        if(getOnWall().empty() && this->getCurrentGPSPosition() == this->m_startPos)
        {
            this->m_isFinished = true;
        }
    });

    m_knn = cv::ml::KNearest::create();

    load_training_data(m_knn);
    //m_knn->setDefaultK(3);

    m_isFinished = false;
    m_disabledGUI = false;
    m_stopMovement = false;

    m_startPos = this->getCurrentGPSPosition();
    m_timeLeft = 800;
    realtime = 0;
}

RobotInstance::~RobotInstance()
{
    delete m_robot;
}

const double turn_kp = 0.75;

double signsqrt(double x)
{
    return x < 0 ? -sqrt(-x) : sqrt(x);
}

// turns shortest way to the direction
void RobotInstance::turnTo(double speed, double target_angle)
{
    target_angle = inputModulus(target_angle, -M_PI, M_PI);
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

    //drive function should take care of it
    if(abs(current - target_angle) <= 0.015)
        return;

    while(step() != -1 && abs(current - target_angle) > 0.01)
    {
        current = m_imu->getRollPitchYaw()[2];

        //std::cout << "current: " << current << std::endl;

        if(abs(target_angle - current) > M_PI)
            mult = -1.0;
        else
            mult = 1.0;

        double error = signsqrt(target_angle - current);

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

    this->m_emitter->send("G", 1);
    while(this->m_receiver->getQueueLength() > 0) { // If receiver queue is not empty
        char *message = (char *)m_receiver->getData(); // Grab data as a string
        if (message[0] == 'L') { // 'L' means a lack of progress occurred
            std::cout << "Detected Lack of Progress!" << std::endl;
            this->m_receiver->nextPacket(); // Discard the current data packet
            this->m_robot->step(this->m_timestep); // update all the senors, for updated gps pos
            clearBfsResult();
            this->updateTargetPos();
        }
        else if(message[0] == 'G')
        {
            char *receivedData = (char *)this->m_receiver->getData(); // Grab data as a string
            if (receivedData[0] == 'G') {
                memcpy(&this->m_score, receivedData + 4, 4); // Score stored in bytes 4 to 7
                memcpy(&this->m_timeLeft, receivedData + 8, 4);  // Remaining time stored in bytes 8 to 11
                this->m_receiver->nextPacket(); // Discard the current data packet
            }
        }
        else
        {
            this->m_receiver->nextPacket();
        }
    }

    realtime = difftime(time(0), starttime);

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

const double drive_kp = 1.2;
const double turn_drive_kp = 1.5;

bool RobotInstance::forwardTicks(double vel, double ticks, pdd target)
{
    double startTime = m_robot->getTime();
    pdd start = getRawGPSPosition();
    double traveled = 0;
    double angle = std::atan2(target.first - start.first, target.second - start.second);
    while(traveled <= ticks && step() != -1)
    {
        /*if (m_lm->getVelocity() < 0 && m_rm->getVelocity() < 0) col(m_color, m_gps, m_imu, m_startPos, -1);
        else col(m_color, m_gps, m_imu, m_startPos, 1);*/
        if (!isTraversable(target, getLidarPoints()))
        {
            // std::cout << "path to target is not traversable!" << std::endl;
            clearBfsResult();
            return false;
        }
        detectVictims();
        if(blackDetected())
        {
            stopMotors();
            break;
        }
        if(ticks - traveled <= 0.01)
        {
            double vel2 = std::max(0.75, vel - pow((traveled - (ticks - 0.01))/0.01, 2) * drive_kp * vel);
            double cur_angle = getYaw();
            double err = angle - cur_angle;
            if(err > M_PI)
                err -= 2 * M_PI;
            else if(err < -M_PI)
                err += 2 * M_PI;
            forward(vel2 + err * turn_drive_kp, vel2 - err * turn_drive_kp);
        }
        else
        {
            double cur_angle = getYaw();
            double err = angle - cur_angle;
            if(err > M_PI)
                err -= 2 * M_PI;
            else if(err < -M_PI)
                err += 2 * M_PI;
            forward(vel + err * turn_drive_kp, vel - err * turn_drive_kp);
        }
        pdd cur = getRawGPSPosition();
        if (m_robot->getTime() > startTime + 3)
        {
            break;
        }
        traveled = hypot(cur.first - start.first, cur.second - start.second);
        angle = std::atan2(target.first - cur.first, target.second - cur.second);
    }

    stopMotors();

    if(blackDetected())
    {
        runCallbacks = false;
        std::cout << "black detected" << std::endl;
        insert_tile("2", m_color, m_gps, m_imu, m_startPos);
        pdd cur = getRawGPSPosition();
        pdd colorSensorLoc = pdd(cur.first + 0.03 * sin(getYaw()), cur.second + 0.03 * cos(getYaw()));
        pdd tileCenter = pdd(std::round((colorSensorLoc.first - m_startPos.first) / TILE_LENGTH) * TILE_LENGTH + m_startPos.first,
            std::round((colorSensorLoc.second - m_startPos.second) / TILE_LENGTH) * TILE_LENGTH + m_startPos.second);
        double rad = 0.045;
        double x = -rad, y = -rad;
        for(; x <= rad; x += 0.005, x = std::round(x / 0.005) * 0.005)
        {
            for(y = -rad; y <= rad; y += 0.005, y = std::round(y / 0.005) * 0.005)
            {
                if (x == -rad || x == rad || y == -rad || y == rad)
                {
                    addLidarPoint(pdd(tileCenter.first + x, tileCenter.second + y));
                }
            }
        }
        updateVisited();
        resetPosition();
        while(traveled >= -2 && step() != -1)
        {
            forward(-vel * 0.5);
            cur = getRawGPSPosition();
            getPosition(&traveled, nullptr);
        }

        clearBfsResult();
        stopMotors();
        runCallbacks = true;
        return false;
    }
    return true;
}

void RobotInstance::delay(double seconds)
{
    double current = m_robot->getTime();
    while(m_robot->step(m_timestep) != -1 && m_robot->getTime() < current + seconds);
}

std::vector<cv::Point> RobotInstance::getContour(std::string name, cv::Mat frame)
{
    cv::Mat frame2;
    cv::Mat hsv;
    cv::cvtColor(frame, frame2, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask1, mask2, mask3;
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 20, 80), mask1);
    cv::inRange(hsv, cv::Scalar(160, 100, 0), cv::Scalar(180, 255, 255), mask2);
    cv::inRange(hsv, cv::Scalar(20, 200, 80), cv::Scalar(40, 255, 255), mask3);

    cv::Mat mask;

    cv::bitwise_or(mask1, mask2, mask);
    cv::bitwise_or(mask, mask3, mask);

    static std::vector<std::vector<cv::Point>> contours;

    contours.clear();

    cv::findContours(mask, contours, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> best_contour;

    addTexture(name + " Threshold", mask.clone(), SDL_PIXELFORMAT_RGB332);

    if(contours.size() == 0)
        return best_contour;

    for(auto it = contours.begin(); it != contours.end(); it++)
    {
        double area = cv::contourArea(*it);
        auto rect = cv::boundingRect(*it);
        double asp_ratio = rect.height / (double)rect.width;
        //std::cout << area << std::endl;
        if(area >= 40 && area <= 2500 && asp_ratio >= 0.5 && asp_ratio <= 2.0)
        {
            if(best_contour.size() == 0 || area > cv::contourArea(best_contour))
                best_contour = *it;
        }
    }

    cv::Mat frame3 = frame.clone();
    if(best_contour.size() > 0)
    {
        cv::drawContours(frame3, std::vector<std::vector<cv::Point>>{best_contour}, -1, cv::Scalar(255, 0, 0));
        if(name.size() > 0)
        {
            addTexture(name, frame3.clone(), SDL_PIXELFORMAT_RGB888);
        }
    }
    return best_contour;
}

std::vector<cv::Point> RobotInstance::getContour(cv::Mat frame)
{
    return getContour("", frame);
}

std::vector<cv::Point> RobotInstance::getContourHazard(std::string name, cv::Mat frame)
{
    cv::Mat frame2, hsv;
    cv::cvtColor(frame, frame2, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;

    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 20, 255), mask);

    static std::vector<std::vector<cv::Point>> contours;

    contours.clear();

    cv::findContours(mask, contours, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> best_contour;

    addTexture(name + " Threshold", mask.clone(), SDL_PIXELFORMAT_RGB332);

    if(contours.size() == 0)
        return best_contour;

    for(auto it = contours.begin(); it != contours.end(); it++)
    {
        double area = cv::contourArea(*it);
        auto rect = cv::boundingRect(*it);
        double asp_ratio = rect.width / (double)rect.height;
        //std::cout << area << std::endl;
        if(area >= 150 && asp_ratio >= 0.5 && asp_ratio <= 2.0)
        {
            if(best_contour.size() == 0 || area > cv::contourArea(best_contour))
                best_contour = *it;
        }
    }

    cv::Mat frame3 = frame.clone();
    if(best_contour.size() > 0)
    {
        cv::drawContours(frame3, std::vector<std::vector<cv::Point>>{best_contour}, -1, cv::Scalar(255, 0, 0));
        if(name.size() > 0)
        {
            addTexture(name, frame3.clone(), SDL_PIXELFORMAT_RGB888);
        }
    }

    return best_contour;
}

std::vector<cv::Point> RobotInstance::getContourColor(std::string name, cv::Mat frame)
{
    cv::Mat frame2, hsv;
    cv::cvtColor(frame, frame2, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask2, mask3;
    cv::inRange(hsv, cv::Scalar(160, 100, 0), cv::Scalar(180, 255, 255), mask2);
    cv::inRange(hsv, cv::Scalar(20, 200, 80), cv::Scalar(40, 255, 255), mask3);

    //there is an orange area between the red and yellow, so blur the mask to make it bigger
    cv::blur(mask2, mask2, cv::Size(5, 5));
    cv::blur(mask3, mask3, cv::Size(5, 5));

    cv::threshold(mask2, mask2, 80, 255, cv::THRESH_BINARY);
    cv::threshold(mask3, mask3, 80, 255, cv::THRESH_BINARY);

    cv::Mat mask;

    cv::bitwise_or(mask3, mask2, mask);

    static std::vector<std::vector<cv::Point>> contours;

    contours.clear();

    cv::findContours(mask, contours, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> best_contour;

    addTexture(name + " Threshold", mask.clone(), SDL_PIXELFORMAT_RGB332);

    if(contours.size() == 0)
        return best_contour;

    for(auto it = contours.begin(); it != contours.end(); it++)
    {
        double area = cv::contourArea(*it);
        auto rect = cv::boundingRect(*it);
        double asp_ratio = rect.width / (double)rect.height;
        //std::cout << area << std::endl;
        if(area >= 50 && area <= 2500 && asp_ratio >= 0.5 && asp_ratio <= 2.0)
        {
            if(best_contour.size() == 0 || area > cv::contourArea(best_contour))
                best_contour = *it;
        }
    }

    cv::Mat frame3 = frame.clone();
    if(best_contour.size() > 0)
    {
        cv::drawContours(frame3, std::vector<std::vector<cv::Point>>{best_contour}, -1, cv::Scalar(255, 0, 0));
        if(name.size() > 0)
        {
            addTexture(name, frame3.clone(), SDL_PIXELFORMAT_RGB888);
        }
    }
    return best_contour;
}

void RobotInstance::addTexture(std::string name, cv::Mat m, SDL_PixelFormatEnum f)
{
    if(!m_disabledGUI)
        m_tex[name] = getTextureFromMat(renderer, m, f);
}

char RobotInstance::checkHsv(cv::Mat roi, std::string side)
{
    if(!m_knn->isTrained())
        return 0;
    cv::Mat roi2 = roi.clone();

    cv::cvtColor(roi2, roi2, cv::COLOR_BGR2HSV);

    addTexture("HSV " + side, roi2.clone(), SDL_PIXELFORMAT_RGB888);

    cv::Mat red;
    cv::Mat orange;

    cv::inRange(roi2, cv::Scalar(160, 100, 0), cv::Scalar(180, 255, 255), red);
    cv::inRange(roi2, cv::Scalar(20, 100, 80), cv::Scalar(40, 255, 255), orange);

    std::vector<std::vector<cv::Point>> red_c;
    std::vector<std::vector<cv::Point>> orange_c;
    cv::findContours(red, red_c, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(orange, orange_c, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if(red_c.size() == 0)
    {
        return 0;
    }

    std::vector<cv::Point> big_orange_c;
    std::vector<cv::Point> big_red_c;

    for(size_t i = 0; i < orange_c.size(); i++)
    {
        const double area = cv::contourArea(orange_c[i]);
        auto rect = cv::boundingRect(orange_c[i]);
        double asp_ratio = rect.width / (double)rect.height;
        if(asp_ratio <= 0.8 || asp_ratio >= 3.0)
            continue;
        if(area >= 50 && area <= 3500 && (big_orange_c.size() == 0 || area > cv::contourArea(big_orange_c)))
            big_orange_c = orange_c[i];
    }

    for(size_t i = 0; i < red_c.size(); i++)
    {
        const double area = cv::contourArea(red_c[i]);
        auto rect = cv::boundingRect(red_c[i]);
        double asp_ratio = rect.width / (double)rect.height;
        if(asp_ratio <= 0.8 || asp_ratio >= 3.0)
            continue;
        if(area >= 50 && area <= 3500 && (big_red_c.size() == 0 || area > cv::contourArea(big_red_c)))
            big_red_c = red_c[i];
    }

    if(big_orange_c.size() > 0 && big_red_c.size() > 0)
    {
        double ratio = cv::contourArea(big_orange_c) / cv::contourArea(big_red_c);
        if(ratio < 0.8 || ratio > 1.2)
        {
            std::cout << "ratio O: " << ratio << std::endl;
            return 0;
        }
        std::cout << "found O" << std::endl;
        return 'O';
    }
    else if(big_red_c.size() > 0)
    {
        double ratio = cv::contourArea(big_red_c) / roi.size().area();

        if(ratio >= 0.7)
        {
            std::cout << "ratio F: " <<  ratio << std::endl;
            return 0;
        }

        std::cout << "found F" << std::endl;
        return 'F';
    }

    return 0;
}

char RobotInstance::checkHazard(cv::Mat roi, std::string side)
{
    if(!m_knn->isTrained())
        return 0;

    cv::Mat roi2 = roi.clone();
    cv::Mat roi1;
    cv::cvtColor(roi2, roi1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(roi2, roi2, cv::COLOR_BGR2HSV);

    cv::Mat roi3;
    cv::inRange(roi2, cv::Scalar(0, 0, 0), cv::Scalar(255, 20, 255), roi3);

    cv::bitwise_and(roi1, roi3, roi1);

    cv::Mat mask_black, mask_white;

    cv::inRange(roi2, cv::Scalar(0, 0, 0), cv::Scalar(255, 20, 120), mask_black);
    cv::inRange(roi2, cv::Scalar(0, 0, 200), cv::Scalar(255, 20, 255), mask_white);

    double white_sum = cv::sum(mask_white)[0];
    double black_sum = cv::sum(mask_black)[0];

    if(black_sum <= 1000 || white_sum <= 1000)
        return 0;

    cv::threshold(roi1, roi1, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::Mat Roi1D;
    cv::resize(roi1, Roi1D, cv::Size(10, 10));
    cv::threshold(Roi1D, Roi1D, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    addTexture("Hazard threshold " + side, Roi1D.clone(), SDL_PIXELFORMAT_RGB332);
    cv::Mat Roi1D3;
    Roi1D.convertTo(Roi1D3, CV_32F);
    cv::Mat Roi1D2 = Roi1D3.reshape(1, 1);

    std::vector<float> dists;
    char ret = (char)m_knn->findNearest(Roi1D2, 3, cv::noArray(), cv::noArray(), dists);

    if(ret == 'H' || ret == 'S' || ret == 'U')
        return 0;

    if(dists[0] > 450000)
        return 0;


    std::cout << "ret: " << ret << " dist: " << dists[0] << std::endl;

    return ret;
}

char RobotInstance::determineLetter(cv::Mat roi, std::string side) //"l" or "r"
{
    if(!m_knn->isTrained())
    {
        return 0;
    }

    cv::Mat roi2 = roi.clone();
    cv::cvtColor(roi2, roi2, cv::COLOR_BGR2HSV);
    addTexture("HSV " + side, roi2.clone(), SDL_PIXELFORMAT_RGB888);
    cv::Mat roi3;
    cv::inRange(roi2, cv::Scalar(0, 0, 0), cv::Scalar(255, 20, 80), roi3);
    cv::Mat Roi1D;
    cv::resize(roi3, Roi1D, cv::Size(10, 10));
    cv::threshold(Roi1D, Roi1D, 127, 255, cv::THRESH_BINARY);
    cv::Mat Roi1D3;
    Roi1D.convertTo(Roi1D3, CV_32F);
    cv::Mat Roi1D2 = Roi1D3.reshape(1, 1);
    std::vector<float> dists;
    float ret = m_knn->findNearest(Roi1D2, 3, cv::noArray(), cv::noArray(), dists);
    char ch = (char)ret;
    float dist = dists[0];

    // std::cout << "ret: " << ch << " dist: " << dist << std::endl;

    if(ch == 'P' || ch == 'C')
        ch = 0;

    if(dist > 600000)
    {
        return 0;
    }

    std::cout << "ret: " << ch << " dist: " << dist << std::endl;

    // std::cout << std::string(message) << std::endl;
    return ch;
}

void RobotInstance::stopAndEmit(void* message)
{
    if(m_disableEmit)
        return;

    stopMotors();
    delay(VICTIM_DELAY_TIME);
    m_emitter->send(message, 9);
    m_robot->step(m_timestep);

    std::cout << "emitted" << std::endl;
}

pdd RobotInstance::victimToPoint(int rectCenterX, int frameCols, std::string side)
{
    double offset = rectCenterX - frameCols / 2;
    double percent = offset / (frameCols / 2);
    double thetaFromCenter = clampAngle(std::atan(percent * std::tan(0.5)));
    double thetaFromRobot = (side == "L" ? -M_PI : M_PI) / 2 + thetaFromCenter;
    if (thetaFromRobot < 0)
    {
        thetaFromRobot += 2 * M_PI;
    }
    int rangeImgIdx = std::round(thetaFromRobot / (2 * M_PI / 512.0));
    return lidarToPoint(getRawGPSPosition(), m_lidar->getRangeImage()[rangeImgIdx + 1024], clampAngle(thetaFromRobot + getYaw()));
}

void RobotInstance::followVictim(pdd point, std::string side)
{
    if(m_disableEmit) return;
    pdd nearest = nearestTraversable(point, getCurrentGPSPosition(), get_lidar_minmax_opt());
    if (isTraversableOpt(nearest) && getDist(nearest, point) <= MAX_VIC_IDENTIFICATION_RANGE)
    {
        std::cout << "following victim" << std::endl;
        isFollowingVictim = true;
        moveToPoint(this, nearest);
        pdd cur = getRawGPSPosition();
        turnTo(MAX_VELOCITY / 2, -std::atan2(point.first - cur.first, point.second - cur.second) + (side == "L" ? -M_PI : M_PI) / 2);
        stopMotors();
        isFollowingVictim = false;
    }
    else
    {
        std::cout << "Could not follow victim" << std::endl;
        stopMotors();
    }
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
    struct {
        int32_t xpos;
        int32_t zpos;
        char letter;
    } message;
    pdd cur = getRawGPSPosition();
    message.xpos = (int32_t)(cur.first * 100);
    message.zpos = (int32_t)(cur.second * -100);
    message.letter = 0;
    if (rangeImage[horizontalResolution * 3 / 4] <= MAX_VIC_DETECTION_RANGE)
    {
        auto contour = getContour("Left Contour", frameL);
        auto hazard = getContourHazard("Left HazardC", frameL);
        auto color = getContourColor("Left ColorH", frameL);

        auto boundRect = cv::boundingRect(contour);
        if(boundRect.area() >= 200)
        {
            cv::Mat roi(frameL, boundRect);
            message.letter = determineLetter(roi, "L");
        }
        if(!message.letter)
        {
            boundRect = cv::boundingRect(color);
            if(boundRect.area() >= 200)
            {
                cv::Mat roi(frameL, boundRect);
                message.letter = checkHsv(roi, "L");
            }
        }
        if(!message.letter)
        {
            boundRect = cv::boundingRect(hazard);
            if(boundRect.area() >= 200)
            {
                cv::Mat roi(frameL, boundRect);
                message.letter = checkHazard(roi, "L");
            }
        }

        if(message.letter)
        {
            pdd point = victimToPoint(boundRect.x + boundRect.width / 2, frameL.cols, "L");
            if(getDist(cur, point) <= MAX_VIC_IDENTIFICATION_RANGE)
            {
                addVictim(point);
                victimMap[point] = message.letter;
                stopAndEmit((void*)&message);
            }
            else if(getDist(cur, point) <= MAX_VIC_DETECTION_RANGE && !isFollowingVictim)
            {
                std::cout << "victim dist: " << getDist(cur, point) << std::endl;
                addVictim(point);
                followVictim(point, "L");
            }
            return;
        }
    }
    if (rangeImage[horizontalResolution / 4] <= MAX_VIC_DETECTION_RANGE)
    {
        auto contour = getContour("Right Contour", frameR);
        auto hazard = getContourHazard("Right HazardC", frameR);
        auto color = getContourColor("Right ColorH", frameR);

        auto boundRect = cv::boundingRect(contour);
        if(boundRect.area() >= 200)
        {
            cv::Mat roi(frameR, boundRect);
            message.letter = determineLetter(roi, "R");
        }
        if(!message.letter)
        {
            boundRect = cv::boundingRect(color);
            if(boundRect.area() >= 200)
            {
                cv::Mat roi(frameR, boundRect);
                message.letter = checkHsv(roi, "R");
            }
        }
        if(!message.letter)
        {
            boundRect = cv::boundingRect(hazard);
            if(boundRect.area() >= 200)
            {
                cv::Mat roi(frameR, boundRect);
                message.letter = checkHazard(roi, "R");
            }
        }

        if(message.letter)
        {
            pdd point = victimToPoint(boundRect.x + boundRect.width / 2, frameL.cols, "R");
            if(getDist(cur, point) <= MAX_VIC_IDENTIFICATION_RANGE)
            {
                addVictim(point);
                victimMap[point] = message.letter;
                stopAndEmit((void*)&message);
            }
            else if(getDist(cur, point) <= MAX_VIC_DETECTION_RANGE && !isFollowingVictim)
            {
                std::cout << "victim dist: " << getDist(cur, point) << std::endl;
                addVictim(point);
                followVictim(point, "R");
            }
            return;
        }
    }
}

void RobotInstance::detectVictims()
{
    lookForLetter();
}

pdd RobotInstance::calcNextPos()
{
    pdd ret = r2d(chooseMove(this, getYaw()));
    // std::cout << "traversable: " << isTraversableOpt(ret) << std::endl;
    return ret;
}

void RobotInstance::moveToPos(pdd pos)
{
    pdd curPos = getRawGPSPosition();

    if(compPts(curPos, pos))
    {
        std::cout << "already at position!" << std::endl;
        return;
    }

    double dist = getDist(curPos, pos);
    double angle = -std::atan2(pos.first - curPos.first, pos.second - curPos.second);

    turnTo(MAX_VELOCITY, angle);
    forwardTicks(MAX_VELOCITY, dist, pos);
    // if (!isFollowingBfs() && dist > 0.012)
    // {
    //     double rounded = clampAngle(std::round(angle / (M_PI / 2)) * M_PI / 2);
    //     turnTo(4, rounded);
    // }
}

void RobotInstance::moveToNextPos()
{
    if (getTimeLeft() <= 20 || getRealTime() >= 580)
    {
        std::cout << "time almost up" << std::endl;
        auto path = pointBfs(getCurrentGPSPosition(), m_startPos, get_lidar_minmax_opt(), false);
        path.push(getRawGPSPosition());
        double pathLen = 0;
        pdd last = path.top();
        path.pop();
        while(!path.empty())
        {
            pdd cur = path.top();
            pathLen += getDist(cur, last);
            last = cur;
            path.pop();
        }
        if (getTimeLeft() < 30 * pathLen || getRealTime() >= (600 - 20 * pathLen))
        {
            moveToPoint(this, m_startPos, false);
            clearOnWall();
            m_isFinished = true;
            return;
        }
    }

    if(!isTraversableOpt(getTargetPos()))
    {
        std::cout << "recomputing target pos!" << std::endl;
        updateTargetPos();
    }

    moveToPoint(this, getTargetPos(), false);
}

void RobotInstance::updateVisited()
{
    std::pair<pdd, pdd> minMax = get_lidar_minmax_opt();
    m_mazeW = minMax.second.first - minMax.first.first;
    m_mazeH = minMax.second.second - minMax.first.second;
    pdd cur = getCurrentGPSPosition();
    double rotation = getYaw();
    addVisited(cur);
    addVisited(pointTo(cur, rotation + M_PI / 2));
    addVisited(pointTo(cur, rotation - M_PI / 2));
    addVisited(pointTo(cur, rotation + M_PI / 2, 0.02));
    addVisited(pointTo(cur, rotation - M_PI / 2, 0.02));
    if(cur != m_lastPos)
    {
        if (m_lm->getVelocity() < 0 && m_rm->getVelocity() < 0) col(m_color, m_gps, m_imu, m_startPos, -1);
        else col(m_color, m_gps, m_imu, m_startPos, 1);
        bfsAddOnWall(cur, 0.08);
        const double radius = 0.08;

        double x = cur.first - radius, y = cur.second - radius;
        for(; x <= cur.first + radius; x += 0.008)
        {
            for(y = cur.second - radius; y <= cur.second + radius; y += 0.008)
            {
                pdd point = r2d(pdd(x, y));
                bool traversable = isTraversableOpt(point);
                if(!traversable)
                {
                    removeOnWall(point);
                    addVisited(point);
                }
                if (checkNearbyVisited(point))
                {
                    addPseudoVisited(point);
                    removeOnWall(point);
                }
            }
        }
    }
    m_lastPos = getCurrentGPSPosition();
}

std::vector<std::pair<char, SDL_Texture*>> RobotInstance::get_training_images()
{
    std::vector<std::pair<char, SDL_Texture*>> ret;

    for(size_t i = 0; i < output.size(); i++)
    {
        cv::Mat img = training_data.row(i).clone();
        cv::Mat img2;
        img.convertTo(img2, CV_8U);
        img2 = img2.reshape(1, 10);
        SDL_Texture *tex = getTextureFromMat(renderer, img2.clone(), SDL_PIXELFORMAT_RGB332);
        ret.push_back(std::make_pair((char)output[i], tex));
    }

    return ret;
}

bool RobotInstance::blackDetected()
{
    const uint8_t* colors = getColor();

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
    if(m_lastPos != getCurrentGPSPosition())
    {
        update_regions_map(this, m_lidar->getRangeImage() + 1024, m_imu->getRollPitchYaw()[2]);
    }
    //update_camera_map(m_gps, m_lidar->getRangeImage() + 1024, m_leftCamera, m_imu->getRollPitchYaw()[2]);
}

void RobotInstance::run_callbacks()
{
    if (!runCallbacks)
    {
        return;
    }
    for(const auto& callback : m_callbacks)
    {
        callback();
    }
}

REGION* RobotInstance::get_current_region()
{
    return get_region(m_gps);
}

std::map<pdd, char>& getVictims()
{
    return victimMap;
}