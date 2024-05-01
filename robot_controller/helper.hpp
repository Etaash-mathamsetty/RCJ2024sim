#include <iostream>
#include <list>

#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <opencv2/opencv.hpp>

#ifndef _HELPER_HPP_
#define _HELPER_HPP_

struct pair_hash_combiner
{
    std::size_t operator()(const std::pair<int, int>& p) const
    {
        std::string s = std::to_string(p.first) + " " + std::to_string(p.second);
        return std::hash<std::string>()(s);
    }
};

inline SDL_Window *window;
inline SDL_Renderer *renderer;

double r2d(double decimal);
pdd r2d(pdd point);
void printPoint(const pdd& point);
std::string pointToString(const pdd& point);
SDL_Texture *getTextureFromMat(SDL_Renderer *r, const cv::Mat& mat, SDL_PixelFormatEnum f);

#endif