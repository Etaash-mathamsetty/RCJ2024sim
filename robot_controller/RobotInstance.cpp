#include "RobotInstance.hpp"
#include <math.h>
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
#define _USE_MATH_DEFINES
#include <cmath>

std::map<std::pair<std::pair<pdd, std::string>, double>, char> victims;

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
    if(side == "L")
    {
        cv::Mat frame = getLeftCameraMat();
        auto contour = getContour(frame);
        if(contour.size() == 0)
        {
            std::cout << "no contour found!" << std::endl;
            return;
        }

        cv::Rect boundRect = boundingRect(contour);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        cv::adaptiveThreshold(frame, frame, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2.0);
        cv::Mat _roi(frame, boundRect);
        cv::Mat roi = _roi.clone();
        cv::Mat roi3;
        cv::resize(roi, roi3, cv::Size(20, 20));
        cv::Mat roi4;
        roi3.convertTo(roi4, CV_32FC1);
        cv::Mat roi2 = roi4.reshape(1, 1);
        if(training_data.rows == 0)
            training_data.push_back(roi2);
        else
            cv::vconcat(training_data, roi2, training_data);
        output.push_back((int)classification);
    }
    else if(side == "R")
    {
        cv::Mat frame = getRightCameraMat();
        auto contour = getContour(frame);
        if(contour.size() == 0)
        {
            std::cout << "no contour found!" << std::endl;
            return;
        }

        cv::Rect boundRect = boundingRect(contour);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        cv::adaptiveThreshold(frame, frame, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2.0);
        cv::Mat _roi(frame, boundRect);
        cv::Mat roi = _roi.clone();
        cv::Mat roi3;
        cv::resize(roi, roi3, cv::Size(20, 20));
        cv::Mat roi4;
        roi3.convertTo(roi4, CV_32FC1);
        cv::Mat roi2 = roi4.reshape(1, 1);
        if(training_data.rows == 0)
            training_data.push_back(roi2);
        else
            cv::vconcat(training_data, roi2, training_data);
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

    int res = m_robot->step(m_timestep);

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

        if(getToVisit().size() == 0 && this->getCurrentGPSPosition() == this->m_startPos)
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

    while(step() != -1 && abs(current - target_angle) > 0.005)
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

bool RobotInstance::forwardTicks(double vel, double ticks, pdd target)
{
    double startTime = m_robot->getTime();
    pdd start = getRawGPSPosition();
    double traveled = 0;
    double angle = std::atan2(target.first - start.first, target.second - start.second);
    while(traveled <= ticks && step() != -1)
    {
        detectVictims();
        if(blackDetected())
        {
            break;
        }
        if(ticks - traveled <= 0.01)
        {
            forward(std::max(0.75, vel - pow((traveled - (ticks - 0.01))/0.01, 2) * drive_kp * vel));
        }
        else
        {
            double cur_angle = getYaw();
            double err = angle - cur_angle;
            if(err > M_PI)
                err -= 2 * M_PI;
            else if(err < -M_PI)
                err += 2 * M_PI;
            forward(vel + err * turn_kp, vel - err * turn_kp);
        }
        pdd cur = getRawGPSPosition();
        if (m_robot->getTime() > startTime + 5)
        {
            break;
        }
        traveled = hypot(cur.first - start.first, cur.second - start.second);
        angle = std::atan2(target.first - cur.first, target.second - cur.second);

        if (!isTraversableOpt(target))
        {
            std::cout << "path to target is not traversable!" << std::endl;
            clearBfsResult();
            return false;
        }
    }

    stopMotors();

    if(blackDetected())
    {
        std::cout << "black detected" << std::endl;
        pdd cur = getRawGPSPosition();
        addLidarPoint(r2d(pointTo(cur, this->getYaw(), 0.03)));
        addLidarPoint(r2d(target));
        pdd adjacents[4] = {
            pdd(target.first - 0.01, target.second),
            pdd(target.first + 0.01, target.second),
            pdd(target.first, target.second - 0.01),
            pdd(target.first, target.second + 0.01)
        };
        for (const pdd& adjacent : adjacents)
        {
            addLidarPoint(r2d(adjacent));
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
    cv::cvtColor(frame, frame2, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold(frame2, frame2, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2.0);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame2, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> best_contour;

    addTexture(name + " Threshold", frame2, SDL_PIXELFORMAT_RGB332);

    if(contours.size() == 0)
        return best_contour;

    for(auto it = contours.begin(); it != contours.end(); it++)
    {
        double area = cv::contourArea(*it);
        if(area >= 400 && area <= 2500)
        {
            if(best_contour.size() == 0 || cv::contourArea(*it) > cv::contourArea(best_contour))
                best_contour = *it;
        }
    }

    cv::Mat frame3(frame);
    if(best_contour.size() > 0)
    {
        cv::drawContours(frame3, std::vector<std::vector<cv::Point>>{best_contour}, -1, cv::Scalar(255, 0, 0));
        if(name.size() > 0)
        {
            addTexture(name, frame3, SDL_PIXELFORMAT_RGB888);
        }
    }
    return best_contour;
}

std::vector<cv::Point> RobotInstance::getContour(cv::Mat frame)
{
    return getContour("", frame);
}

void RobotInstance::addTexture(std::string name, cv::Mat m, SDL_PixelFormatEnum f)
{
    if(!m_disabledGUI)
        m_tex[name] = getTextureFromMat(renderer, m, f);
}

char message[9];

bool RobotInstance::determineLetter(const cv::Mat& roi, std::string side, const double* position) //"l" or "r"
{
    if(!m_knn->isTrained())
    {
        return false;
    }
    cv::Mat roi2 = roi.clone();
    cv::cvtColor(roi2, roi2, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold(roi2, roi2, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2.0);
    cv::Mat Roi1D;
    cv::resize(roi2, Roi1D, cv::Size(20, 20));
    cv::Mat Roi1D3;
    Roi1D.convertTo(Roi1D3, CV_32F);
    cv::Mat Roi1D2 = Roi1D3.reshape(1, 1);
    std::vector<float> dists;
    float ret = m_knn->findNearest(Roi1D2, 3, cv::noArray(), cv::noArray(), dists);
    char ch = (char)ret;
    float dist = dists[0];

    std::cout << "ret: " << ret << " dist: " << dist << std::endl;

    if(dist > 4500000)
    {
        return false;
    }

    int xPos = (int)(position[0] * 100);
    int zPos = (int)(position[2] * 100);
    memcpy(&message[0], &xPos, 4);
    memcpy(&message[4], &zPos, 4);
    message[8] = ch;
    if (!victims.count(std::make_pair(std::make_pair(pdd(position[0], position[2]), side), m_imu->getRollPitchYaw()[2])))
    {
        delay(1);
        victims[(std::make_pair(std::make_pair(pdd(position[0], position[2]), side), m_imu->getRollPitchYaw()[2]))] = message[8];
        std::cout << message[8] << " found @" << xPos << " " << zPos << getVictims()[(std::make_pair(std::make_pair(pdd(position[0], position[2]), side), m_imu->getRollPitchYaw()[2]))] << std::endl;
        m_emitter->send(message, 9);
    }
    std::cout << message[8] << " found on side " << side << std::endl;
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
    auto contourL = getContour("Left Contour", frameL);
    auto contourR = getContour("Right Contour", frameR);
    if (rangeImage[horizontalResolution * 3 / 4] <= MAX_VIC_DETECTION_RANGE && contourL.size() > 0)
    {
        boundRect = boundingRect(contourL);
        cv::Mat roi(frameL, boundRect);
        addTexture("Left ROI", roi.clone(), SDL_PIXELFORMAT_RGB888);
        double rectCenterX = boundRect.x + boundRect.width / 2; //in columns
        if (boundRect.width * boundRect.height > 400)
        {
            if (determineLetter(roi, "l", m_gps->getValues()))
            {
                double offset = rectCenterX - frameL.cols / 2;
                double percent = offset / (frameL.cols / 2);
                double thetaFromCenter = clampAngle(std::atan(percent * std::tan(0.5)));
                double thetaFromRobot = -M_PI / 2 + thetaFromCenter;
                if (thetaFromRobot < 0)
                {
                    double offset = rectCenterX - frameL.cols / 2;
                    double percent = offset / (frameL.cols / 2);
                    double thetaFromCenter = clampAngle(std::atan(percent * std::tan(0.5)));
                    double thetaFromRobot = -M_PI / 2 + thetaFromCenter;
                    if (thetaFromRobot < 0)
                    {
                        thetaFromRobot += 2 * M_PI;
                    }
                    int rangeImgIdx = std::round(thetaFromRobot / (2 * M_PI / 512.0));
                    pdd point = lidarToPoint(m_gps, rangeImage[rangeImgIdx], clampAngle(thetaFromRobot - m_imu->getRollPitchYaw()[2])).first;
                    bool ret = addVictim(point);
                    if ((ret && getDist(getRawGPSPosition(), point) <= MAX_VIC_IDENTIFICATION_RANGE) || reporting)
                    {
                        stopMotors();
                        std::cout << "emitting" << std::endl;
                        delay(1);
                        m_emitter->send(message, sizeof(message));
                        isFollowingVictim = false;
                        reporting = false;
                    }
                    // else if(ret && !reporting)
                    // {
                    //     if (!isFollowingVictim)
                    //     {
                    //         std::cout << "following victim" << std::endl;
                    //         stopMotors();
                    //         pdd nearest = nearestTraversable(point, getCurrentGPSPosition(), getMinMax(getLidarPoints()));
                    //         isFollowingVictim = true;
                    //         moveToPoint(this, nearest);
                    //         pdd cur = getRawGPSPosition();
                    //         turnTo(4, -std::atan2(point.first - cur.first, point.second - cur.second) + M_PI / 2);
                    //         isFollowingVictim = false;
                    //         reporting = true;
                    //         std::cout << "done following victim" << std::endl;
                    //     }
                    // }
                    return;
                }
                int rangeImgIdx = std::round(thetaFromRobot / (2 * M_PI / 512.0));
                pdd point = lidarToPoint(m_gps, rangeImage[rangeImgIdx], clampAngle(thetaFromRobot - m_imu->getRollPitchYaw()[2])).first;
                bool ret = addVictim(point);
                if ((ret && getDist(getRawGPSPosition(), point) <= MAX_VIC_IDENTIFICATION_RANGE) || reporting)
                {
                    stopMotors();
                    std::cout << "emitting" << std::endl;
                    delay(1);
                    m_emitter->send(message, sizeof(message));
                    step();
                    isFollowingVictim = false;
                    reporting = false;
                }
                else
                {
                    if (!isFollowingVictim)
                    {
                        std::cout << "following victim" << std::endl;
                        stopMotors();
                        pdd nearest = nearestTraversable(point, getCurrentGPSPosition(), getMinMax(getLidarPoints()));
                        isFollowingVictim = true;
                        moveToPoint(this, nearest);
                        pdd cur = getRawGPSPosition();
                        turnTo(4, -std::atan2(point.first - cur.first, point.second - cur.second) + M_PI / 2);
                        isFollowingVictim = false;
                        reporting = true;
                        std::cout << "done following victim" << std::endl;
                    }
                }
                return;
            }
        }
    }
    if (rangeImage[horizontalResolution / 4] <= MAX_VIC_DETECTION_RANGE && contourR.size() > 0)
    {
        boundRect = boundingRect(contourR);
        cv::Mat roi(frameR, boundRect);
        addTexture("Right ROI", roi.clone(), SDL_PIXELFORMAT_RGB888);
        double rectCenterX = boundRect.x + boundRect.width / 2; //in columns
        if (boundRect.width * boundRect.height > 400)
        {
            if (determineLetter(roi, "r", m_gps->getValues()))
            {
                double offset = rectCenterX - frameR.cols / 2;
                double percent = offset / (frameR.cols / 2);
                double thetaFromCenter = clampAngle(std::atan(percent * std::tan(0.5)));
                double thetaFromRobot = M_PI / 2 + thetaFromCenter;
                if (thetaFromRobot < 0)
                {
                    double offset = rectCenterX - frameR.cols / 2;
                    double percent = offset / (frameR.cols / 2);
                    double thetaFromCenter = clampAngle(std::atan(percent * std::tan(0.5)));
                    double thetaFromRobot = M_PI / 2 + thetaFromCenter;
                    if (thetaFromRobot < 0)
                    {
                        thetaFromRobot += 2 * M_PI;
                    }
                    int rangeImgIdx = std::round(thetaFromRobot / (2 * M_PI / 512.0));
                    pdd point = lidarToPoint(m_gps, rangeImage[rangeImgIdx], clampAngle(thetaFromRobot - m_imu->getRollPitchYaw()[2])).first;
                    bool ret = addVictim(point);
                    // std::cout << getCurrentGPSPosition().first <<" " << getCurrentGPSPosition().second << " " << point.first << " " << point.second << std::endl;
                    if ((ret && getDist(getRawGPSPosition(), point) <= MAX_VIC_IDENTIFICATION_RANGE) || reporting)
                    {
                        stopMotors();
                        std::cout << "emitting" << std::endl;
                        delay(1);
                        m_emitter->send(message, sizeof(message));
                        step();
                        isFollowingVictim = false;
                        reporting = false;
                    }
                    // else if(ret && !reporting)
                    // {
                    //     if (!isFollowingVictim)
                    //     {
                    //         std::cout << "following victim" << std::endl;
                    //         stopMotors();
                    //         pdd nearest = nearestTraversable(point, getCurrentGPSPosition(), getMinMax(getLidarPoints()));
                    //         isFollowingVictim = true;
                    //         moveToPoint(this, nearest);
                    //         pdd cur = getRawGPSPosition();
                    //         turnTo(4, -std::atan2(point.first - cur.first, point.second - cur.second) - M_PI / 2);
                    //         isFollowingVictim = false;
                    //         reporting = true;
                    //         std::cout << "done following victim" << std::endl;
                    //     }
                    // }
                    return;
                }
                int rangeImgIdx = std::round(thetaFromRobot / (2 * M_PI / 512.0));
                pdd point = lidarToPoint(m_gps, rangeImage[rangeImgIdx], clampAngle(thetaFromRobot - m_imu->getRollPitchYaw()[2])).first;
                bool ret = addVictim(point);
                // std::cout << getCurrentGPSPosition().first <<" " << getCurrentGPSPosition().second << " " << point.first << " " << point.second << std::endl;
                if ((ret && getDist(getRawGPSPosition(), point) <= MAX_VIC_IDENTIFICATION_RANGE) || reporting)
                {
                    stopMotors();
                    std::cout << "emitting" << std::endl;
                    delay(1);
                    m_emitter->send(message, sizeof(message));
                    isFollowingVictim = false;
                    reporting = false;
                }
                else
                {
                    if (!isFollowingVictim)
                    {
                        std::cout << "following victim" << std::endl;
                        stopMotors();
                        pdd nearest = nearestTraversable(point, getCurrentGPSPosition(), getMinMax(getLidarPoints()));
                        isFollowingVictim = true;
                        moveToPoint(this, nearest);
                        pdd cur = getRawGPSPosition();
                        turnTo(4, -std::atan2(point.first - cur.first, point.second - cur.second) - M_PI / 2);
                        isFollowingVictim = false;
                        reporting = true;
                        std::cout << "done following victim" << std::endl;
                    }
                }
                return;
            }
        }
    }
}

void RobotInstance::detectVictims()
{
    if (isFollowingVictim)
    {
        return;
    }
    // try
    // {
        lookForLetter();
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << "caught exception:" << std::endl;
    //     std::cerr << e.what() << '\n';
    // }

}

pdd RobotInstance::calcNextPos()
{
    pdd ret = r2d(chooseMove(this, m_imu->getRollPitchYaw()[2]));
    std::cout << "traversable: " << isTraversableOpt(ret) << std::endl;
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

    double dist = getDist(curPos, pos) * 0.95; //prevent overshooting
    double angle = -std::atan2(pos.first - curPos.first, pos.second - curPos.second);
    
    turnTo(4, angle);
    forwardTicks(5, dist, pos);
    // if (!isFollowingBfs() && dist > 0.012)
    // {
    //     double rounded = clampAngle(std::round(angle / (M_PI / 2)) * M_PI / 2);
    //     turnTo(4, rounded);
    // }
}

void RobotInstance::moveToNextPos()
{
    moveToPos(getTargetPos());
    
}

void RobotInstance::updateVisited()
{
    pdd cur = getCurrentGPSPosition();
    if(!isVisited(cur))
    {
        addVisited(cur);
    }
    if(cur != m_lastPos)
    {
        const double radius = 0.08;

        double x = cur.first - radius, y = cur.second - radius;
        for(; x <= cur.first + radius; x += 0.008)
        {
            for(y = cur.second - radius; y <= cur.second + radius; y += 0.008)
            {
                pdd point = r2d(pdd(x, y));
//                if(point == pointTo(cur, m_imu->getRollPitchYaw()[2]))
//                {
//                    continue;
//                }
                bool visited = isVisited(point);
                bool traversable = isTraversableOpt(point);
                if(!visited && traversable && canSee(cur, point))
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
                // else if(visited && !traversable)
                // {
                //     removeVisited(point);
                // }
                else if(!traversable)
                {
                    removeToVisit(point);
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

std::map<std::pair<std::pair<pdd, std::string>, double>, char>& getVictims()
{
    return victims;
}