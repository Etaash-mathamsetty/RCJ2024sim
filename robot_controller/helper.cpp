#include "constants.h"

#include <cmath>
#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <opencv2/opencv.hpp>

double r2d(double decimal)
{
    return std::round(decimal * 100) / 100.0;
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

pdd r3d(pdd point)
{
    return pdd(r3d(point.first), r3d(point.second));
}

double inputModulus(double input, double minimumInput, double maximumInput)
{
    double modulus = maximumInput - minimumInput;

    // Wrap input if it's above the maximum input
    int numMax = (int) ((input - minimumInput) / modulus);
    input -= numMax * modulus;

    // Wrap input if it's below the minimum input
    int numMin = (int) ((input - maximumInput) / modulus);
    input -= numMin * modulus;

    return input;
}

void printPoint(const pdd& point)
{
    //std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;
}

std::string pointToString(const pdd& point)
{
    return std::string("(") + std::to_string(point.first) + ", " + std::to_string(point.second) + ")";
}

SDL_Texture *  getTextureFromMat(SDL_Renderer *r, cv::Mat mat, SDL_PixelFormatEnum f)
{
    int width = mat.size().width;
    int height = mat.size().height;

    SDL_Surface *surf = SDL_CreateRGBSurfaceWithFormatFrom(mat.data, width, height, 8, mat.channels() * width, f);

    SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);

    SDL_FreeSurface(surf);

    return tex;
}