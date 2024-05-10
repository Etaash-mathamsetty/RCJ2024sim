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
#include <filesystem>

#include <SDL.h>

#include "imgui/imgui.h"
#include "imgui/implot.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"
#include "imgui/imgui_internal.h"

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

void init_gui(SDL_Window *window, SDL_Renderer *renderer)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    io.ConfigFlags = ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImPlot::CreateContext();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
	ImGui_ImplSDLRenderer2_Init(renderer);

#ifdef _WIN32
    // ImGui_ImplWin32_EnableDpiAwareness();
    // ImGui_ImplWin32_GetDpiScaleForHwnd(GetDesktopWindow());
#endif
}

void init_frame(SDL_Renderer *renderer)
{
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
}

void end_frame(RobotInstance *rb, SDL_Renderer *renderer)
{
    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
    SDL_RenderPresent(renderer);

    for(const auto& pair : rb->getTextures())
    {
        SDL_DestroyTexture(pair.second);
    }
}

void draw_frame(RobotInstance *rb, SDL_Renderer *r, SDL_Window *window)
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
                ImGui::Text("Lidar points: %ld  Score: %f  Time: %d", getCount(), rb->getScore(), rb->getTimeLeft());
                ImGui::Text("IsTraversable%s: %d", pointToString(rb->getCurrentGPSPosition()).c_str(),
                                isTraversableOpt(rb->getCurrentGPSPosition()));

                if(ImGui::Button("Clear Point Cloud"))
                    clearPointCloud();
                ImGui::SameLine();
                if(ImGui::Button("Lack of Progress"))
                    rb->sendLackOP();

                plotPoints(rb, width, height);

                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("Sensor Debug"))
            {
                cv::Mat m = rb->getLeftCameraMat();
                cv::Mat m2 = rb->getRightCameraMat();

                for(const auto& pair : rb->getTextures())
                {
                    ImGui::Text("%s", pair.first.c_str());
                    ImGui::Image((void*)pair.second, ImVec2(256, 256));
                }

                float color[3] = {rb->getColor()[0] / 255.0f, rb->getColor()[1] / 255.0f, rb->getColor()[2] / 255.0f};

                ImGui::Text("Color Sensor: ");
                ImGui::ColorEdit3("", color, ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoInputs);

                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("Debug Controls"))
            {
                ImGui::Checkbox("Stop Movement", &rb->getStopMovement());

                ImGui::EndTabItem();
            }

            if(ImGui::BeginTabItem("KNN Trainer") && rb->getKNN())
            {
                if(ImGui::Button("Save to File"))
                {
                    rb->getKNN()->save("knn.yml");
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
        ImGui_ImplSDL2_ProcessEvent(&event);
        if(event.type == SDL_QUIT)
        {
            running = false;
            exit(0);
        }
    }
}

void delete_gui(SDL_Window* window, SDL_Renderer *renderer)
{
    ImGui_ImplSDLRenderer2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();
	SDL_DestroyRenderer(renderer);
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

#ifdef COMPMODE
    rb->setDisableGUI(true);
#endif
    bool running = true;

    std::cout << "ImGui Version: " << ImGui::GetVersion() << std::endl;
    {
        SDL_version v;
        SDL_GetVersion(&v);

        std::cout << "SDL Version: " << (int)v.major << "." << (int)v.minor << "." << (int)v.patch << std::endl;

#ifdef __linux__
        SDL_SetHint(SDL_HINT_VIDEODRIVER, "wayland,x11");
#endif
    }

    if(!rb->getDisableGUI())
    {
        window = SDL_CreateWindow("Simulation Debug Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
                                            800, 600, SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

        int rw = 0, rh = 0;
        SDL_GetRendererOutputSize(renderer, &rw, &rh);
        if(rw != 800) {
            float widthScale = (float)rw / (float) 800;
            float heightScale = (float)rh / (float) 600;

            if(widthScale != heightScale) {
                fprintf(stderr, "WARNING: width scale != height scale\n");
            }

            SDL_RenderSetScale(renderer, widthScale, heightScale);
        }

        init_gui(window, renderer);

        rb->add_step_callback(
        [&running, &rb]()
        {
            if(!rb->getDisableGUI())
            {
                poll_events(running);

                init_frame(renderer);

                draw_frame(rb, renderer, window);

                end_frame(rb, renderer);
            }
        });
    }

    rb->update_lidar_cloud();

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (rb->step() != -1 && running && !rb->isFinished()) {
        rb->updateTargetPos();
        rb->moveToNextPos();
    }

    if(!rb->getDisableGUI())
        delete_gui(window, renderer);

    // Enter here exit cleanup code.
    rb->endSimulation();
    rb->destroyInstance();
    return 0;
}
