// File:                    dufus.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
#include "RobotInstance.hpp"
#include "helper.hpp"
#include "map.h"
#include "mapping.h"
#include "navigation.h"
#include <filesystem>
#include <time.h>
#include <SDL3/SDL.h>

#include "imgui/imgui.h"
#include "imgui/implot.h"
#include "imgui/implot3d.h"
#include "imgui/imgui_impl_sdl3.h"
#include "imgui/imgui_impl_sdlgpu3.h"

#ifdef _WIN32
#include "win_imgui/imgui_impl_win32.h"
#endif

#ifdef __linux__
#include <unistd.h>
#endif

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;


void printList(std::list<std::pair<int, int>> list)
{
        for(auto item : list)
        {
                std::cout << "(" << item.first << "," << item.second << ") ";
        }
        std::cout << std::endl;
}

void init_gui(SDL_Window *window, SDL_GPUDevice* device)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    io.ConfigFlags = ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();

    //ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
	//ImGui_ImplSDLRenderer2_Init(renderer);

    ImGui_ImplSDL3_InitForSDLGPU(window);
    ImGui_ImplSDLGPU3_InitInfo info;
    info.ColorTargetFormat = SDL_GetGPUSwapchainTextureFormat(device, window);
    info.GpuDevice = device;
    info.MSAASamples = SDL_GPU_SAMPLECOUNT_1;
    ImGui_ImplSDLGPU3_Init(&info);

#ifdef _WIN32
    // ImGui_ImplWin32_EnableDpiAwareness();
    // ImGui_ImplWin32_GetDpiScaleForHwnd(GetDesktopWindow());
#endif
}

void init_frame(void)
{
    ImGui_ImplSDL3_NewFrame();
    ImGui_ImplSDLGPU3_NewFrame();
    ImGui::NewFrame();
}

std::vector<std::pair<char, SDL_GPUTextureSamplerBinding*>> image_map;

void end_frame(RobotInstance *rb, SDL_GPUDevice *device)
{
    SDL_GPUCommandBuffer* cmdbuf = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUTexture *swapchain_tex;
    SDL_WaitAndAcquireGPUSwapchainTexture(cmdbuf, window, &swapchain_tex, NULL, NULL);
    if(swapchain_tex)
    {
        ImGui::Render();
        SDL_GPUColorTargetInfo info = {0};
        info.clear_color = SDL_FColor {0.0f, 0.0f, 0.0f, 0.0f};
        info.texture = swapchain_tex;
        info.load_op = SDL_GPU_LOADOP_CLEAR;
        info.store_op = SDL_GPU_STOREOP_STORE;
        SDL_GPURenderPass* render_pass = SDL_BeginGPURenderPass(cmdbuf, &info, 1, NULL);
        Imgui_ImplSDLGPU3_PrepareDrawData(ImGui::GetDrawData(), cmdbuf);
        ImGui_ImplSDLGPU3_RenderDrawData(ImGui::GetDrawData(), cmdbuf, render_pass);
        SDL_EndGPURenderPass(render_pass);
    }
    SDL_SubmitGPUCommandBuffer(cmdbuf);
    SDL_WaitForGPUIdle(device);
    SDL_WaitForGPUSwapchain(device, window);

    for(const auto &it : rb->getTextures())
    {
        SDL_ReleaseGPUTexture(device, it.second->texture);
        SDL_ReleaseGPUSampler(device, it.second->sampler);
        delete it.second;
    }

    for(const auto &it : image_map)
    {
        SDL_ReleaseGPUTexture(device, it.second->texture);
        SDL_ReleaseGPUSampler(device, it.second->sampler);
        delete it.second;
    }

    rb->getTextures().clear();
    image_map.clear();
}

void draw_frame(RobotInstance *rb, SDL_Window *window)
{
    int width, height;

    SDL_GetWindowSize(window, &width, &height);

    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    if(ImGui::Begin("window", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove))
    {
        if(ImGui::BeginTabBar("tabs", ImGuiTabBarFlags_Reorderable))
        {
            if(ImGui::BeginTabItem("Map Debug", nullptr))
            {
                ImGui::Text("Lidar points: %ld  Score: %f  Time: %d  RealTime: %ld", getCount(), rb->getScore(), rb->getTimeLeft(), rb->getRealTime());
                ImGui::Text("IsTraversableOpt%s: %d", pointToString(rb->getCurrentGPSPosition()).c_str(),
                                isTraversableOpt(rb->getCurrentGPSPosition()));
                ImGui::Text("IsTraversableOpt%s: %d", pointToString(rb->getRawGPSPosition()).c_str(),
                                isTraversableOpt(rb->getRawGPSPosition()));

                if(ImGui::Button("Clear Point Cloud"))
                    clearPointCloud();
                ImGui::SameLine();
                if(ImGui::Button("Lack of Progress"))
                    rb->sendLackOP();

                plotPoints(rb, width, height);

                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("Nav Debug", nullptr))
            {
                static std::vector<double> xs;
                static std::vector<double> ys;
                static std::vector<double> zs;

                ImGui::Text("points: %lu", xs.size());

                if(ImPlot3D::BeginPlot("Nav Weights", ImVec2(width-50, height-50)))
                {
                    ImPlot3D::SetupAxis(ImAxis3D_X, nullptr, ImPlot3DAxisFlags_AutoFit);
                    ImPlot3D::SetupAxis(ImAxis3D_Y, nullptr, ImPlot3DAxisFlags_AutoFit);
                    ImPlot3D::SetupAxis(ImAxis3D_Z, nullptr, ImPlot3DAxisFlags_AutoFit);
                    {
                        double x, y, z = 0;
                        x = rb->getCurrentGPSPosition().first;
                        y = rb->getCurrentGPSPosition().second;
                        ImPlot3D::PlotScatter("Robot", &x, &y, &z, 1);
                    }

                    if(xs.empty() || ImGui::IsKeyPressed(ImGuiKey_C, false))
                    {
                        xs.reserve(20000);
                        ys.reserve(20000);
                        zs.reserve(20000);
                        xs.clear();
                        ys.clear();
                        zs.clear();

                        std::pair<pdd, pdd> minmax = get_lidar_minmax_opt();

                        const double step_size = 0.01;

                        int x_len = abs(minmax.first.first - minmax.second.first) / step_size;
                        int y_len = abs(minmax.first.second - minmax.second.second) / step_size;

                        for(int i = 0; i < x_len; i++)
                        {
                            for(int l = 0; l < y_len; l++)
                            {
                                pdd cur = {minmax.first.first + i * step_size, minmax.first.second + l * step_size};
                                double value = get_value(cur);
                                if(value > 0)
                                {
                                    xs.push_back(cur.first);
                                    ys.push_back(cur.second);
                                    zs.push_back(value);
                                }
                            }
                        }
                    }

                    ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Asterisk, 0.8f);
                    ImPlot3D::PlotScatter("Potential", xs.data(), ys.data(), zs.data(), xs.size());

                    {
                        std::vector<double> path_xs;
                        std::vector<double> path_ys;
                        std::vector<double> path_zs;

                        pdd cur = rb->getRawGPSPosition();
                        pdd tar = rb->getTargetPos();
                        const double zero = 0;

                        stack<pdd> path = contFFGD(cur, tar, get_lidar_minmax_opt());

                        while(path.size() > 0)
                        {
                            path_xs.push_back(path.top().first);
                            path_ys.push_back(path.top().second);
                            path_zs.push_back(0);
                            path.pop();
                        }

                        ImPlot3D::PlotLine("Path", path_xs.data(), path_ys.data(), path_zs.data(), path_xs.size());
                        ImPlot3D::PlotScatter("Target", &tar.first, &tar.second, &zero, 1);
                    }

                    ImPlot3D::EndPlot();
                }
                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("Sensor Debug"))
            {

                float color[3] = {rb->getColor()[0] / 255.0f, rb->getColor()[1] / 255.0f, rb->getColor()[2] / 255.0f};

                ImGui::Text("Color Sensor: ");
                ImGui::ColorEdit3("", color, ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoInputs);

                std::array<double, 2048> xs = {0};
                std::array<double, 2048> ys = {0};
                std::array<double, 2048> zs = {0};

                for(int l = 0; l < 4; l++)
                {
                    const float *image = rb->getLidar()->getLayerRangeImage(l);

                    for(int i = 0; i < 512; i++)
                    {
                        long double dist = image[i];

                        dist *= std::cos(kTiltAngles[l]);

                        double angle = (double)i * (rb->getLidar()->getFov() / rb->getLidar()->getHorizontalResolution());
                        xs[i + l*512] = dist * std::sin(angle);
                        ys[i + l*512] = dist * std::cos(angle);
                        zs[i + l*512] = dist * std::sin(kTiltAngles[l]) + LIDAR_Z_HEIGHT;
                    }
                }

                if(ImPlot3D::BeginPlot("Lidar", ImVec2(-1, 0)))
                {
                    ImPlot3D::PlotScatter("", xs.data(), ys.data(), zs.data(), xs.size());

                    ImPlot3D::EndPlot();
                }

                for(const auto& pair : rb->getTextures())
                {
                    ImGui::Text("%s", pair.first.c_str());
                    ImGui::Image((intptr_t)pair.second, ImVec2(256, 256));
                }

                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("Debug Controls"))
            {
                ImGui::Checkbox("Stop Movement", &rb->getStopMovement());
                ImGui::Checkbox("Disable Emit", &rb->getDisableEmit());

                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("KNN Trainer"))
            {
                const char classifications[5] = {'H', 'S', 'U', 'P', 'C'};
                const char* classification_names[5] = {"Harmed", "Stable", "Unharmed", "Poision", "Corrosive"};
                static int idx = 0;

                ImGui::Combo("Classification", &idx, classification_names, 5);

                if(ImGui::Button("Add Left Image"))
                {
                    rb->add_training_data("L", classifications[idx]);
                }

                ImGui::SameLine();

                if(ImGui::Button("Add Right Image"))
                {
                    rb->add_training_data("R", classifications[idx]);
                }

                if(ImGui::Button("Save to File"))
                {
                    rb->save_training_data();
                }

                for(const auto& pair : rb->getTextures())
                {
                    ImGui::Text("%s", pair.first.c_str());
                    ImGui::Image((intptr_t)pair.second, ImVec2(256, 256));
                }

                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("Training Image Viewer"))
            {

                image_map = rb->get_training_images();

                int i = 0;
                for(const auto& pair : image_map)
                {
                    ImGui::Text("%c: index %d", pair.first, i++);
                    ImGui::Image((intptr_t)pair.second, ImVec2(256, 256));
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
        ImGui::End();
    }

    ImGuiIO& io = ImGui::GetIO();

    if(io.KeyAlt && io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
    {
        rb->getStopMovement() = !rb->getStopMovement();
    }
}

void poll_events(bool &running)
{
    SDL_Event event;

    while(SDL_PollEvent(&event))
    {
        ImGui_ImplSDL3_ProcessEvent(&event);
        if(event.type == SDL_EVENT_QUIT)
        {
            running = false;
            exit(0);
        }
    }
}

void delete_gui(SDL_Window* window, SDL_GPUDevice *device)
{
    ImGui_ImplSDLGPU3_Shutdown();
	ImGui_ImplSDL3_Shutdown();
	ImGui::DestroyContext();
	SDL_DestroyGPUDevice(device);
	SDL_DestroyWindow(window);
    SDL_Quit();
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {

    //std::filesystem::current_path("/home/etaash/Documents/Erebus-v23_0_5/game/controllers/robot_controller/build");

    //std::cout << std::filesystem::current_path() << std::endl;

#ifdef __linux__
    std::cout << "PID: " << getpid() << std::endl;
#endif

    RobotInstance* rb = RobotInstance::getInstance();

//#define COMPMODE
#ifdef COMPMODE
    rb->setDisableGUI(true);
#endif
    bool running = true;

    std::cout << "ImGui Version: " << ImGui::GetVersion() << std::endl;
    {
        int v = SDL_GetVersion();

        std::cout << "SDL Version: " << SDL_VERSIONNUM_MAJOR(v) << "." << SDL_VERSIONNUM_MINOR(v) << "." << SDL_VERSIONNUM_MICRO(v) << std::endl;

#ifdef __linux__
#ifdef SDL_HINT_VIDEO_DRIVER
        SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "wayland,x11");
#endif
#endif
    }

    if(!rb->getDisableGUI())
    {
        window = SDL_CreateWindow("Simulation Debug Window",
                                            800, 600, SDL_WINDOW_HIGH_PIXEL_DENSITY | SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
        device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXBC, false, NULL);

        SDL_ClaimWindowForGPUDevice(device, window);
        SDL_SetGPUSwapchainParameters(device, window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, SDL_GPU_PRESENTMODE_IMMEDIATE);

        init_gui(window, device);

        rb->add_step_callback(
        [&running, &rb]()
        {
            if(!rb->getDisableGUI())
            {
                poll_events(running);

                init_frame();

                draw_frame(rb, window);

                end_frame(rb, device);
            }
        });
    }

    rb->update_lidar_cloud();


    bool sent = false;
    rb->add_step_callback([&rb, &sent](){
        if (rb->getTimeLeft() <= 2 || rb->getRealTime() >= 598) {
            send(getLidarPoints(), rb->getEmitter(), rb->getStartPos(), rb->getRB());
            sent = true;
        }
    });

    /*rb->add_step_callback([&rb]() {
        if (rb->getLM()->getVelocity() < 0 && rb->getRM()->getVelocity() < 0) col(rb->getColorSensor(), rb->getGPS(), rb->getIMU(), rb->getStartPos(), -1);
        else col(rb->getColorSensor(), rb->getGPS(), rb->getIMU(), rb->getStartPos(), 1);
    });*/

    while (rb->step() != -1 && running && !rb->isFinished()) {
        rb->updateTargetPos();
        rb->moveToNextPos();
        //show(getLidarPoints(), rb->getEmitter(), rb->getStartPos(), rb->getRB());
        if ((isAllDone() || rb->isFinished()) && rb->getCurrentGPSPosition() == rb->getStartPos() && getOnWall().size() == 0)
        {
            send(getLidarPoints(), rb->getEmitter(), rb->getStartPos(), rb->getRB());
            sent = true;
            running = false;
        }

        /*if (seconds >= (realseconds - buffertime))
        {
            send(getLidarPoints(), rb->getEmitter(), rb->getStartPos(), rb->getRB());
            sent = true;
            running = false;
        }*/
    }

    if (!sent)
        send(getLidarPoints(), rb->getEmitter(), rb->getStartPos(), rb->getRB());
    // Enter here exit cleanup code.
    if(!rb->getDisableGUI())
        delete_gui(window, device);
    rb->destroyInstance();
    return 0;
}
