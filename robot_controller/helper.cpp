#include "constants.h"

#include <cmath>
#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <opencv2/opencv.hpp>

double r2d(double decimal)
{
    return std::round(decimal * 100) / 100;
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