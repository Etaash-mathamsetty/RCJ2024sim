
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_
enum class DIR
{
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW
};

#define MAX_VELOCITY 6.28
#define TILE_LENGTH 0.12
#define ADJ_WALL 0.05
#define MAX_VIC_DETECTION_RANGE 0.1
#define MAX_VIC_IDENTIFICATION_RANGE 0.072
//vertical and horizontal
#define CAMERA_FOV 1 //yep that's the actual number
#define pdd std::pair<double, double>
#define pii std::pair<int, int>
#define TRAVERSABLE_RADIUS 0.037

#define LIDAR_TILT_ANGLE_L4 -0.10472 //radians (6 degrees)
#define LIDAR_TILT_ANGLE_L3 -0.0349066
#define LIDAR_TILT_ANGLE_L2 0.0349066
#define LIDAR_TILT_ANGLE_L1 0.10472

const double kTiltAngles[4] = { LIDAR_TILT_ANGLE_L1, LIDAR_TILT_ANGLE_L2, LIDAR_TILT_ANGLE_L3, LIDAR_TILT_ANGLE_L4 };

#define LIDAR_Z_HEIGHT 0.12

#define DISTANCE_TILE_ANGLE 0.17 //radians
#define VICTIM_DELAY_TIME 1.4
#endif

#ifndef M_PI
#define M_PI 3.141592653589793238462643383
#endif
