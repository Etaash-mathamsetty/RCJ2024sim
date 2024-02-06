// File:          dufus.cpp
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

  RobotInstance* rb = RobotInstance::getInstance();

  const int horizontalResolution = rb->m_lidar->getHorizontalResolution();
  const int numberOfLayers = rb->m_lidar->getNumberOfLayers();

  std::cout << horizontalResolution << std::endl;
  std::cout << numberOfLayers << std::endl;

  int start_floor = rb->getFloor();
  std::pair<int, int> start_index = rb->getIndex();


  SDL_Window *window;
  SDL_Renderer *renderer;
  SDL_CreateWindowAndRenderer(800, 600, SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE, &window, &renderer);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  io.ConfigFlags = ImGuiConfigFlags_NavEnableKeyboard;

  ImGui::StyleColorsDark();
  ImPlot::CreateContext();

  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
	ImGui_ImplSDLRenderer2_Init(renderer);

  bool running = true;
  bool plot_regions = true;

  rb->update_lidar_cloud();

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (rb->step() != -1 && running) {

    {
      SDL_Event event;

      while(SDL_PollEvent(&event))
      {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if(event.type == SDL_QUIT)
        {
          running = false;
        }
      }
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
		SDL_RenderClear(renderer);
		ImGui_ImplSDLRenderer2_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

    int width, height;

    SDL_GetWindowSize(window, &width, &height);

    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::Begin("window", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    ImGui::Text("Number of points: %d", getCount());

    if(ImGui::Button("Reset Scale"))
      ImPlot::SetNextAxesLimits(-1, -1, 1, 1, ImPlotCond_Once);
    if(ImGui::Button("Clear Point Cloud"))
      clearPointCloud();

    ImGui::Checkbox("Plot Regions", &plot_regions);

    ImPlot::SetNextMarkerStyle(ImPlotMarker_Asterisk, 1.5);
    plotPoints(width, height, plot_regions);

    // rb->writeTileData();

    // auto path = rb->BFS();

    // std::cout << "index: " << rb->getIndex().first << "," << rb->getIndex().second << std::endl;
    // printList(path);

    // for(std::pair<int, int> index : path)
    // {
    //   rb->detectVictims();
    //   //x case
    //   index.first -= rb->getIndex().first;
    //   //y case
    //   index.second -= rb->getIndex().second;
    //   if(index.first == 0)
    //     switch(index.second)
    //     {
    //       case -1:
    //         rb->turnTo(1, DIR::N);
    //         break;
    //       case 1:
    //         rb->turnTo(1, DIR::S);
    //         break;
    //     }
    //   else if(index.second == 0)
    //     switch(index.first)
    //     {
    //       case -1:
    //         rb->turnTo(1, DIR::W);
    //         break;
    //       case 1:
    //         rb->turnTo(1, DIR::E);
    //         break;
    //     }
    //   else
    //   {
    //     std::cout << "ERROR: path is not valid" << std::endl;
    //     break;
    //   }
    //   rb->forwardTile(4.0);
    //   rb->alignRobot();
    // }

    // if(rb->getQuitable() && rb->getIndex() == start_index && rb->getFloor() == start_floor)
    //   break;

    // rb->detectVictims();
    //cout << instance.m_lidar->getRollPitchYaw()[2] << endl;

    rb->update_lidar_cloud();

    ImGui::End();
		ImGui::Render();
		ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
		SDL_RenderPresent(renderer);
  }

  ImGui_ImplSDLRenderer2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
  SDL_Quit();

  // Enter here exit cleanup code.
  rb->endSimulation();
  rb->destroyInstance();
  return 0;
}
