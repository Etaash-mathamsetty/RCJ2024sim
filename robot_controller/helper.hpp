#include <iostream>
#include <list>

#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <opencv2/opencv.hpp>
#include "constants.h"

#ifndef _HELPER_HPP_
#define _HELPER_HPP_

//thanks stack overflow
template <class T>
inline void hash_combine(std::size_t & s, const T & v)
{
    std::hash<T> h;
    s ^= h(v) + 0x9e3779b9 + (s<< 6) + (s>> 2);
}

template<typename T>
struct pair_hash_combiner
{
    std::size_t operator()(const std::pair<T, T>& p) const
    {
        std::size_t hash = 0;
        hash_combine(hash, p.first);
        hash_combine(hash, p.second);
        return hash;
    }
};

inline SDL_Window *window;
inline SDL_Renderer *renderer;

inline double round_to(double value, const double precision = 0.01)
{
    return std::round(value / precision) * precision;
}

inline pdd round_to(const pdd& point, const double precision = 0.01)
{
    return pdd(round_to(point.first, precision), round_to(point.second, precision));
}

inline double floor_to(double value, const double precision = 0.01)
{
    return std::floor(value / precision) * precision;
}

inline double ceil_to(double value, const double precision = 0.01)
{
    return std::ceil(value / precision) * precision;
}

double r2d(double decimal);
double r3d(double decimal);
pdd midpoint(const pdd& pt1, const pdd& pt2);
double inputModulus(double input, double minimumInput, double maximumInput);
pdd r2d(pdd point);
pdd r3d(pdd point);
void printPoint(const pdd& point);
std::string pointToString(const pdd& point);
SDL_Texture *getTextureFromMat(SDL_Renderer *r, cv::Mat mat, SDL_PixelFormatEnum f);

#endif