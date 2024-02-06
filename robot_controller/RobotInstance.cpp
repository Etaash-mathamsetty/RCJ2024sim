#include "RobotInstance.hpp"
#include <math.h>
#include <algorithm>
#include <string.h>
#include <unordered_map>
#include <optional>
#include <list>
#include "helper.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>

#include "map.h"


static RobotInstance* instance = NULL;

inline cv::Mat getCv2Mat(webots::Camera *cam)
{
    const uint8_t *img = cam->getImage();
    cv::Mat frame(cam->getHeight(), cam->getWidth(), CV_8UC4, (void*)img);

    return frame;
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

    m_index = std::make_pair(0, 0);
    m_floor = 0;
    start_tile_floor[m_floor] = m_index;
    floors[m_floor][m_index] = TILE();
    quitable = false;

    map = &floors[m_floor];
    (*map)[m_index].bot = true;

    m_lposoffset = 0;
    m_rposoffset = 0;

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

    knn = cv::ml::KNearest::create();
    knn->load("knn.yml");
}

RobotInstance::~RobotInstance()
{
    delete m_robot;
}

const double turn_kp = 0.5;

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
            calc_speed -= std::max(0.1, 2.0 * abs(error));
        else
            calc_speed += std::max(0.1, 2.0 * abs(error));

        forward(-calc_speed, calc_speed);
    }

    stopMotors();
}

void RobotInstance::turnTo(double speed, DIR dir)
{
    if(dir == m_dir)
        return;

    double angles[4] = { 0.0, -M_PI_2, M_PI, M_PI_2 };

    //std::cout << "turning to: " << angles[(int)dir] << std::endl;

    turnTo(speed, angles[(int)dir]);

    m_dir = dir;
}

//if this returns -1, then the robot has gone off course
int RobotInstance::getDirection()
{
    double yaw = m_imu->getRollPitchYaw()[2];

    //N E S W
    double errors[4] = { yaw, yaw + M_PI_2, abs(yaw) - M_PI, yaw - M_PI_2 };

    int result = std::find_if(errors, errors + 4, [](double x) { return abs(x) < 0.1; }) - errors;

    return result == 4 ? -1 : result;
}

bool RobotInstance::alignmentNeeded()
{
    return getDirection() == -1;
}

bool RobotInstance::forwardTicks(double vel, double ticks)
{
    resetPosition();
    double pos = 0.0;
    //TODO: use PID
    while(pos <= ticks && step() != -1)
    {
        detectVictims();
        if(blackDetected())
            break;
        forward(vel);
        getPosition(&pos, NULL);
        //std::cout << pos << std::endl;
    }

    stopMotors();

    if(blackDetected())
    {
        std::cout << "black detected" << std::endl;

        while(pos >= 0.0 && step() != -1)
        {
            forward(-vel);
            getPosition(&pos, NULL);
        }

        stopMotors();
        resetPosition();

        return false;
    }

    resetPosition();

    return true;
}

std::array<std::optional<std::pair<int, int>>, 4> RobotInstance::getNeighbors(std::pair<int, int> index)
{
    std::array<std::optional<std::pair<int, int>>, 4> result;
    const int dir = (int)m_dir;

    result[(dir + 3) % 4] = !(*map)[index].E ? std::optional(std::make_pair(index.first + 1, index.second)) : std::nullopt;
    result[(dir + 1) % 4] = !(*map)[index].W ? std::optional(std::make_pair(index.first - 1, index.second)) : std::nullopt;
    result[(dir + 2) % 4] = !(*map)[index].S ? std::optional(std::make_pair(index.first, index.second + 1)) : std::nullopt;
    result[dir] = !(*map)[index].N ? std::optional(std::make_pair(index.first, index.second - 1)) : std::nullopt;

    return result;
}

bool RobotInstance::forwardTile(double vel)
{
    bool ret = forwardTicks(vel, TILE_LENGTH);

    if(!ret)
    {
        auto temp_index = m_index;
        switch(m_dir)
        {
            case DIR::N:
                temp_index.second -= 1;
                break;
            case DIR::E:
                temp_index.first += 1;
                break;
            case DIR::S:
                temp_index.second += 1;
                break;
            case DIR::W:
                temp_index.first -= 1;
                break;
        }

        (*map)[temp_index].E = true;
        (*map)[temp_index].W = true;
        (*map)[temp_index].N = true;
        (*map)[temp_index].S = true;

        temp_index.second += 1;
        (*map)[temp_index].N = true;

        temp_index.second -= 2;
        (*map)[temp_index].S = true;
        temp_index.second += 1;

        temp_index.first += 1;
        (*map)[temp_index].W = true;
        temp_index.first -= 2;
        (*map)[temp_index].E = true;
        temp_index.first += 1;


        return false;
    }

    (*map)[m_index].bot = false;

    switch(m_dir)
    {
        case DIR::N:
            m_index.second -= 1;
            break;
        case DIR::E:
            m_index.first += 1;
            break;
        case DIR::S:
            m_index.second += 1;
            break;
        case DIR::W:
            m_index.first -= 1;
            break;
    }

    (*map)[m_index].bot = true;

    alignRobot();

    return true;
}

void RobotInstance::writeTileData()
{
    if(!(*map)[m_index].vis)
    {
        const int horizontalResolution = m_lidar->getHorizontalResolution();
        const float* rangeImage = m_lidar->getRangeImage() + (horizontalResolution * 2);
        float sides[4] = {0.0f};

        sides[0] = rangeImage[0];
        sides[1] = rangeImage[horizontalResolution / 4];
        sides[2] = rangeImage[horizontalResolution / 2];
        sides[3] = rangeImage[((horizontalResolution * 3) / 4)];


        std::cout << "front: " << sides[0] << std::endl;
        std::cout << "right90: " << sides[1] << std::endl;
        std::cout << "left90: " << sides[3] << std::endl;
        std::cout << "back: " << sides[2] << std::endl;

        std::cout << "DIR: " << (int)m_dir << std::endl;

        (*map)[m_index].vis = true;

        //why is this math not working for every case :(
        if(m_dir == DIR::N || m_dir == DIR::S)
        {
            (*map)[m_index].N |= sides[(int)m_dir] < 0.1;
            (*map)[m_index].E |= sides[((int)m_dir + 1) % 4] < 0.1;
            (*map)[m_index].S |= sides[((int)m_dir + 2) % 4] < 0.1;
            (*map)[m_index].W |= sides[((int)m_dir + 3) % 4] < 0.1;
        }
        else
        {
            (*map)[m_index].S |= sides[(int)m_dir] < 0.1;
            (*map)[m_index].W |= sides[((int)m_dir + 1) % 4] < 0.1;
            (*map)[m_index].N |= sides[((int)m_dir + 2) % 4] < 0.1;
            (*map)[m_index].E |= sides[((int)m_dir + 3) % 4] < 0.1;
        }

        (*map)[m_index].print();

        detectVictims();
    }
}

void RobotInstance::detectVictims()
{
    cv::Mat frame = getCv2Mat(m_rightCamera);
    cv::Mat frame2 = getCv2Mat(m_leftCamera);

    try
    {
        detectVictims(frame, true);
        detectVictims(frame2, false);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

}

void RobotInstance::detectVictims(cv::Mat& img, bool left)
{
    const int lhres = m_lidar->getHorizontalResolution();
    const float* lidar_img = m_lidar->getRangeImage() + (lhres * 2);
    if(left)
    {
        if(lidar_img[lhres/4] > 0.1)
        {
            //cv::destroyAllWindows();
            return;
        }
    }
    else
    {
        if(lidar_img[(lhres * 3) / 4] > 0.1)
        {
            //cv::destroyAllWindows();
            return;
        }
    }
    cv::Mat gray_frame = img.clone();
    cv::cvtColor(img, gray_frame, cv::COLOR_BGR2GRAY);

    //std::cout << "Crash?" << std::endl;

    //cv::imshow(std::string("gray left: ") + std::to_string(left), gray_frame);

    cv::adaptiveThreshold(gray_frame, gray_frame, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::list<std::vector<cv::Point>> contours_cand;

    for(std::vector<cv::Point> cont : contours)
    {
        cv::Rect rect = cv::boundingRect(cont);
        const float aspect_ratio = (float)rect.height/(float)rect.width;
        if(aspect_ratio >= 0.5 && aspect_ratio <= 2.0 && cv::contourArea(cont) >= 420)
        {
            contours_cand.push_back(cont);
        }
    }

    //std::cout << "Crash? 1" << std::endl;

    while(contours_cand.size() > 0)
    {
        std::cout << "Loop enter" << std::endl;

        std::vector<cv::Point> cont = contours_cand.front();
        {
            std::vector<std::vector<cv::Point>> conts = {cont};
            cv::drawContours(gray_frame, conts, 0, 255, cv::FILLED);
        }

        std::cout << "Crash? 1.5" << std::endl;

        {
            cv::RotatedRect rect = cv::minAreaRect(cont);
            cv::warpAffine(gray_frame, gray_frame, cv::getRotationMatrix2D(rect.center, rect.angle, 1.0), {gray_frame.cols * 2, gray_frame.rows * 2});
        }

        std::cout << "Crash? 2" << std::endl;

        std::vector<cv::Point> cont2;
        {
            std::vector<std::vector<cv::Point>> conts2;
            cv::findContours(gray_frame, conts2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            auto cont2_itr = std::max_element(conts2.begin(), conts2.end(),
                                        [](std::vector<cv::Point> a, std::vector<cv::Point> b)
                                        {
                                            return cv::contourArea(b) > cv::contourArea(a);
                                        });
            if(cont2_itr != conts2.end())
                cont2 = *cont2_itr;
            else
            {
                std::cerr << "failed to find biggest element in conts2" << std::endl;
                contours_cand.pop_front();
                continue;
            }
        }

        std::cout << "Crash? 3" << std::endl;

        cv::Mat gray_frame2 = gray_frame.clone();
        cv::Rect rect = cv::boundingRect(cont2);
        cv::Mat letter_frame = cv::Mat(gray_frame2, rect);
        cv::resize(letter_frame, letter_frame, cv::Size(20, 20));
        std::transform(letter_frame.begin<uchar>(), letter_frame.end<uchar>(), letter_frame.begin<uchar>(), [](uchar x) { return x > 125 ? 255 : 0; });
        cv::Mat letter_frame2d = letter_frame.clone().reshape(1, 1);

        const int min_score_const = 3500000;
        int min_score = min_score_const;
        char letter = '\0';

        std::cout << "Crash? 4" << std::endl;

        for(int i = 0; i < 4; i++)
        {
            //std::vector<float> letter_frame_float = {letter_frame2d.begin<uchar>(), letter_frame2d.end<uchar>()};
            std::vector<std::vector<float>> dists;
            std::vector<std::vector<float>> nears;

            //test_samples.type() == CV_32F && test_samples.cols == samples.cols in function 'findNearest'

            letter_frame2d.convertTo(letter_frame2d, CV_32F);

            std::cout << letter_frame2d.cols << " " << letter_frame2d.rows << " " << letter_frame2d.type() << std::endl;

            double result = knn->findNearest(letter_frame2d, 1, nears, dists);

            if(dists[0][0] < min_score && dists[0][0] < min_score_const)
            {
                min_score = dists[0][0];
                letter = (char)result;
            }

            cv::rotate(letter_frame, letter_frame, cv::ROTATE_90_CLOCKWISE);
            letter_frame2d = letter_frame.clone().reshape(1, 1);

            std::cout << "Crash? 5" << std::endl;
        }

        if(letter != '\0')
        {
            std::cout << "letter: " << letter << std::endl;
            std::cout << "score: " << min_score << std::endl;
        }

        contours_cand.pop_front();
    }

    //cv::imshow(std::string("gray (tresh) left: ") + std::to_string(left), gray_frame);
    //cv::waitKey(1);
}

std::list<std::pair<int, int>> RobotInstance::BFS()
{
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash_combiner> parent;
	std::list<std::pair<int, int>> worker;
	std::list<std::pair<int, int>> path;
	std::pair<int, int> cur_index = m_index;
	do
	{
		if(worker.size() > 0)
			worker.pop_front();

		auto quad = getNeighbors(cur_index);
		for(int i = 0; i < 4; i++)
		{
			if(quad[i].has_value() && (parent.count(cur_index) == 0 || parent[cur_index] != *quad[i]) && parent.count(*quad[i]) == 0)
			{
				worker.push_back(*quad[i]);
				parent[*quad[i]] = cur_index;
			}
		}

		if(worker.size() == 0)
		{
			/* return to start */
			std::cout << "finished with the maze!" << std::endl;
			std::cout << "returning to starting..." << std::endl;
			quitable = true;
			(*map)[start_tile_floor[m_floor]].vis = false;
			return BFS();
		}

		cur_index = worker.front();

		// BFS is done
		if(!(*map)[cur_index].vis)
			break;

	} while(worker.size() > 0);

	//backtracking
	path.push_front(cur_index);
	do
	{
		path.push_front(parent[path.front()]);
	} while(path.front() != m_index);
	path.pop_front();

	return path;
}

bool RobotInstance::blackDetected()
{
    const uint8_t* colors = m_color->getImage();

    std::cout << "R: " << (int)colors[0] << " G: " << (int)colors[1] << " B: " << (int)colors[2] << std::endl;

    if(colors[0] <= 55 && colors[1] <= 55 && colors[2] <= 55)
        return true;

    return false;
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
}