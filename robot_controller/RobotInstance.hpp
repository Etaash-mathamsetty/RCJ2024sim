#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Camera.hpp>
#include <webots/GPS.hpp>
#include <list>
#include <array>
#include "constants.h"
#include <cstdint>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include "helper.hpp"
#include "map.h"
#include <opencv2/opencv.hpp>

struct TILE
{
    bool N : 1;
    bool S : 1;
    bool E : 1;
    bool W : 1;
    bool bot : 1;
    bool vis : 1;
    bool checkpoint: 1;

    TILE()
    {
        N = false;
        S = false;
        E = false;
        W = false;
        bot = false;
        vis = false;
        checkpoint = false;
    }

    void print()
    {
        std::cout << "N: " << N << " S: " << S << " E: " << E << " W: " << W << " bot: " << bot << " vis: " << vis << std::endl;
    }

private:
    uint8_t filler : 1;
};

//FIXME left and right directions might be flipped (in lidar)

class RobotInstance
{
public:
    static RobotInstance* getInstance();
    static void destroyInstance();

    int step() {
        return m_robot->step(m_timestep);
    }

    void forward(double l_vel, double r_vel)
    {
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
    }

    void getPosition(double *lpos, double *rpos)
    {
        if(lpos)
            *lpos = m_lmpos->getValue() - m_lposoffset;
        if(rpos)
            *rpos = m_rmpos->getValue() - m_rposoffset;
    }

    bool getQuitable()
    {
        return quitable;
    }

    std::pair<int, int> getIndex()
    {
        return m_index;
    }

    int getFloor()
    {
        return m_floor;
    }

    void endSimulation()
    {
        m_emitter->send((const void*)"E", 1);
    }

    bool forwardTicks(double vel, double ticks);

    int getDirection();

    bool alignmentNeeded();

    void turnTo(double speed, DIR dir);

    bool forwardTile(double speed);

    bool blackDetected();

    std::list<std::pair<int, int>> BFS();

    void writeTileData();

    void detectVictims();

    void alignRobot();

    void update_lidar_cloud();

    REGION* get_current_region();

    webots::GPS *getGPS();

    webots::Lidar *m_lidar;
    webots::InertialUnit *m_imu;
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

    std::array<std::optional<std::pair<int, int>>, 4> getNeighbors(std::pair<int, int> index);

    void turnTo(double speed, double target_angle);

    void detectVictims(cv::Mat& img, bool left);

    RobotInstance();

    ~RobotInstance();

    webots::Robot *m_robot;
    webots::Motor *m_lm;
    webots::Motor *m_rm;
    webots::PositionSensor *m_lmpos;
    webots::PositionSensor *m_rmpos;
    webots::Emitter *m_emitter;
    webots::Camera *m_color;
    webots::Camera *m_rightCamera;
    webots::Camera *m_leftCamera;
    webots::GPS *m_gps;
    double m_lposoffset;
    double m_rposoffset;
    int m_timestep;
    std::pair<int, int> m_index;
    int m_floor;
    bool quitable;
    DIR m_dir;

    std::unordered_map<int, std::pair<int, int>> start_tile_floor;
    std::unordered_map<int, std::unordered_map<std::pair<int, int>, TILE, pair_hash_combiner>> floors;

    // TILE **floors;
    std::unordered_map<std::pair<int, int>, TILE, pair_hash_combiner>* map;

    cv::Ptr<cv::ml::KNearest> knn;
};