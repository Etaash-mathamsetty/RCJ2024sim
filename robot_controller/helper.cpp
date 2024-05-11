#include "constants.h"

#include <cmath>
#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <opencv2/opencv.hpp>

double r2d(double decimal)
{
    return std::round(decimal * 100) / 100;
}

double r3d(double decimal)
{
    return std::round(decimal * 1000) / 1000.0;
}

pdd midpoint(const pdd& pt1, const pdd& pt2)
{
    return pdd((pt1.first + pt2.first) / 2, (pt1.second + pt2.second) / 2);
}

pdd r2d(pdd point)
{
    return pdd(r2d(point.first), r2d(point.second));
}

void printPoint(const pdd& point)
{
    std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;
}

std::string pointToString(const pdd& point)
{
    return std::string("(") + std::to_string(point.first) + ", " + std::to_string(point.second) + ")";
}

SDL_Texture *  getTextureFromMat(SDL_Renderer *r, const cv::Mat& mat, SDL_PixelFormatEnum f)
{
    int width = mat.size().width;
    int height = mat.size().height;

    SDL_Surface *surf = SDL_CreateRGBSurfaceWithFormatFrom(mat.data, width, height, 8, mat.channels() * width, f);

    SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);

    SDL_FreeSurface(surf);

    return tex;
}