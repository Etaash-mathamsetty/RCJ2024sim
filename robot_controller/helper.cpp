#include "constants.h"

#include <cmath>
#include <SDL3/SDL.h>
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
    std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;
}

std::string pointToString(const pdd& point)
{
    return std::string("(") + std::to_string(point.first) + ", " + std::to_string(point.second) + ")";
}


SDL_GPUTextureSamplerBinding getTextureFromMat(SDL_GPUDevice *device, cv::Mat mat, SDL_GPUTextureFormat f)
{
    int width = mat.size().width;
    int height = mat.size().height;
    int size = mat.size().area() * mat.channels();
    SDL_GPUTexture *tex;

    //std::cout << mat.channels() << std::endl;

    /*
    SDL_Surface *surf = SDL_CreateRGBSurfaceWithFormatFrom(mat.data, width, height, 8, mat.channels() * width, f);

    SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);

    SDL_FreeSurface(surf);
    */

    {
        SDL_GPUTextureCreateInfo info;

        info.format = f;
        info.height = height;
        info.width = width;
        info.sample_count = SDL_GPU_SAMPLECOUNT_1;
        info.type = SDL_GPU_TEXTURETYPE_2D;
        info.layer_count_or_depth = 1;
        info.num_levels = 1;
        info.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;
        info.props = 0;
        tex = SDL_CreateGPUTexture(device, &info);
    }

    SDL_GPUTransferBufferCreateInfo transfer_info;

    transfer_info.props = 0;
    transfer_info.size = size;
    transfer_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;

    SDL_GPUTransferBuffer* transfer_buf = SDL_CreateGPUTransferBuffer(device, &transfer_info);

    uint8_t* data = (uint8_t*)SDL_MapGPUTransferBuffer(device, transfer_buf, false);
    memcpy(data, mat.data, size);
    SDL_UnmapGPUTransferBuffer(device, transfer_buf);

    SDL_GPUCommandBuffer *buf = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUCopyPass* pass = SDL_BeginGPUCopyPass(buf);

    SDL_GPUTextureRegion region = {0};

    region.texture = tex;
    region.d = 1;
    region.w = width;
    region.h = height;


    SDL_GPUTextureTransferInfo info = {0};

    info.transfer_buffer = transfer_buf;
    info.offset = 0;


    SDL_UploadToGPUTexture(pass,
        &info,
        &region,
        false
    );

    SDL_EndGPUCopyPass(pass);
    SDL_SubmitGPUCommandBuffer(buf);
    SDL_ReleaseGPUTransferBuffer(device, transfer_buf);

    SDL_GPUTextureSamplerBinding binding;

    binding.texture = tex;

    SDL_GPUSamplerCreateInfo sampler_info;
    memset(&sampler_info, 0, sizeof(sampler_info));
    sampler_info.min_filter = SDL_GPU_FILTER_LINEAR;
    sampler_info.mag_filter = SDL_GPU_FILTER_LINEAR;
    sampler_info.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
    sampler_info.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    sampler_info.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    sampler_info.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;

    SDL_GPUSampler *sampler = SDL_CreateGPUSampler(device, &sampler_info);

    binding.sampler = sampler;

    return binding;
}