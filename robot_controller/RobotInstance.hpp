#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <stack>
#include <array>
#include "navigation.h"
#include <cstdint>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include "helper.hpp"
#include "map.h"
#include <opencv2/opencv.hpp>

//FIXME left and right directions might be flipped (in lidar)

#ifndef _ROBOTINSTANCE_H_
#define _ROBOTINSTANCE_H_
class RobotInstance
{
public:
    static RobotInstance* getInstance();
    static void destroyInstance();

    int step();

    void forward(double l_vel, double r_vel)
    {
        if(m_stopMovement)
        {
            m_lm->setVelocity(0.0);
            m_rm->setVelocity(0.0);
            return;
        }
        l_vel = std::clamp(l_vel, -MAX_VELOCITY, MAX_VELOCITY);
        r_vel = std::clamp(r_vel, -MAX_VELOCITY, MAX_VELOCITY);
        m_lm->setVelocity(l_vel);
        m_rm->setVelocity(r_vel);
    }

    void forward(double vel)
    {
        forward(vel, vel);
    }

    void stopMotors()
    {
        forward(0.0);
    }

    void resetPosition()
    {
        m_lposoffset = m_lmpos->getValue();
        m_rposoffset = m_rmpos->getValue();
    }

    void getPosition(double *lpos, double *rpos)
    {
        if(lpos)
            *lpos = m_lmpos->getValue() - m_lposoffset;
        if(rpos)
            *rpos = m_rmpos->getValue() - m_rposoffset;
    }

    bool isFinished()
    {
        return m_isFinished;
    }

    void endSimulation()
    {
        m_emitter->send((const void*)"E", 1);
        step();
    }

    bool forwardTicks(double vel, double ticks, pdd target);

    int getDirection();

    bool alignmentNeeded();

    void turnTo(double speed, DIR dir);

    bool forwardTile(double speed);

    bool blackDetected();

    void writeTileData();

    void detectVictims();

    cv::Mat getLeftCameraMat();

    cv::Mat getRightCameraMat();

    const uint8_t* getColor();

    bool& getStopMovement() { return m_stopMovement; };

    void alignRobot();

    void update_lidar_cloud();

    void add_step_callback(const std::function<void()>& f) {
        m_callbacks.push_back(f);
    }

    pdd getTargetPos() { return m_targetPos; }

    void run_callbacks();

    pdd getCurrentGPSPosition()
    {
        return r2d(getRawGPSPosition());
    }

    pdd getRawGPSPosition()
    {
        return pdd(m_gps->getValues()[0], -m_gps->getValues()[2]);
    }

    void updateTargetPos();
    void moveToPos(pdd pos);
    void moveToNextPos();

    void updateVisited();

    void setDisableGUI(bool disable)
    {
        m_disabledGUI = disable;
    }

    bool getDisableGUI(){ return m_disabledGUI; }

    REGION* get_current_region();

    webots::GPS *getGPS() { return m_gps; }

    webots::Lidar *getLidar() { return m_lidar; }

    webots::InertialUnit *getIMU() { return m_imu; }
    webots::Robot* getRB() { return m_robot; };
    webots::Emitter* getEmitter() { return m_emitter; };
    webots::Camera* getColorSensor() { return m_color; };
    webots::Motor* getRM() { return m_rm; };
    webots::Motor* getLM() { return m_lm; };

    std::map<std::string, SDL_Texture*>& getTextures() { return m_tex; }

    void addTexture(std::string name, cv::Mat, SDL_PixelFormatEnum);

    float getScore() { return m_score; }

    int getTimeLeft() { return m_timeLeft; }

    void sendLackOP() { m_emitter->send("L", 1); }

    cv::ml::KNearest* getKNN() { return m_knn; }

    double getYaw() { return -m_imu->getRollPitchYaw()[2]; }

    const pdd& getStartPos() { return m_startPos; }

    void save_training_data();

    void add_training_data(std::string side, char classification);

    std::vector<std::pair<char, SDL_Texture*>> get_training_images();

    bool& getDisableEmit() { return m_disableEmit; }

    time_t getRealTime() { return realtime; }

private:

    //DOES NOT AFFECT POSITION SENSOR
    void setPosition(double lpos, double rpos)
    {
        m_lm->setPosition(lpos);
        m_rm->setPosition(rpos);
    }

    //DOES NOT AFFECT POSITION SENSOR
    void setPosition(double pos)
    {
        setPosition(pos, pos);
    }

    bool turnTo(double speed, double target_angle);

    void delay(double seconds);

    char determineLetter(cv::Mat roi, std::string side);

    char checkHsv(cv::Mat roi, std::string side);

    char checkHazard(cv::Mat roi, std::string side);

    std::vector<cv::Point> getContourHazard(std::string name, cv::Mat frame);
    std::vector<cv::Point> getContourColor(std::string name, cv::Mat frame);

    void stopAndEmit(void *message);
    pdd victimToPoint(int rectCenterX, int frameCols, std::string side);
    void followVictim(pdd victim, std::string side);

    bool isFollowingVictim;
    bool reporting;
    void lookForLetter();

    RobotInstance();

    ~RobotInstance();

    pdd calcNextPos();

    std::vector<cv::Point> getContour(std::string, cv::Mat);
    std::vector<cv::Point> getContour(cv::Mat);

    void black_detection_callback();

    webots::Robot *m_robot;
    webots::Motor *m_lm;
    webots::Motor *m_rm;
    webots::PositionSensor *m_lmpos;
    webots::PositionSensor *m_rmpos;
    webots::Emitter *m_emitter;
    webots::Receiver *m_receiver;
    webots::Camera *m_color;
    webots::Camera *m_rightCamera;
    webots::Camera *m_leftCamera;
    webots::GPS *m_gps;
    webots::Lidar *m_lidar;
    webots::InertialUnit *m_imu;

    double m_lposoffset;
    double m_rposoffset;
    int m_timestep;
    bool m_isFinished;
    bool m_stopMovement;
    DIR m_dir;

    float m_score;
    int m_timeLeft;

    double m_mazeW = DBL_MAX;
    double m_mazeH = DBL_MAX;

    pdd m_targetPos;
    pdd m_lastPos;
    pdd m_startPos;

    bool m_disabledGUI;
    bool m_disableEmit;

    time_t starttime;
    time_t realtime;

    bool runCallbacks = true;
    std::vector<std::function<void()>> m_callbacks;
    std::map<std::string, SDL_Texture*> m_tex;

    cv::Ptr<cv::ml::KNearest> m_knn;

};

std::map<pdd, char>& getVictims();
#endif
