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

#ifdef _WIN32
#include "win_imgui/imgui_impl_win32.h"
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

void end_frame(SDL_Renderer *renderer)
{
  ImGui::End();
  ImGui::Render();
  ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
  SDL_RenderPresent(renderer);
}

void draw_frame(RobotInstance *rb, SDL_Window *window, bool* plot_regions)
{

  int width, height;

  SDL_GetWindowSize(window, &width, &height);

  ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_Always);
  ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
  ImGui::Begin("window", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

  ImGui::Text("Number of points: %ld", getCount());

  if(ImGui::Button("Clear Point Cloud"))
    clearPointCloud();

  ImGui::Checkbox("Plot Regions", plot_regions);

  ImPlot::SetNextMarkerStyle(ImPlotMarker_Asterisk, 1.5);
  plotPoints(rb->getGPS(), rb->m_imu->getRollPitchYaw()[2], width, height, *plot_regions);
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

bool is_blocked(const float *cloud, bool extra = false)
{
  float val = cloud[0];
  float val2 = cloud[10];
  float val3 = cloud[500];
  float val4 = cloud[20];
  float val5 = cloud[490];
  float val6 = cloud[30];
  float val7 = cloud[480];
  float val8 = cloud[40];
  float val9 = cloud[470];

  const float thresh = 0.06 + 0.04 * extra;

  if(val < thresh)
    return true;
  if(val2 < thresh)
    return true;
  if(val3 < thresh)
    return true;
  if(val4 < thresh)
    return true;
  if(val5 < thresh)
    return true;
  if(val6 < thresh)
    return true;
  if(val7 < thresh)
    return true;
  if(val8 < thresh)
    return true;
  if(val9 < thresh)
    return true;

  return false;
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

  bool running = true;
  bool plot_regions = true;

  rb->update_lidar_cloud();

  rb->add_step_callback(
    [&running, &renderer, &rb, &window, &plot_regions]()
    {
      poll_events(running);

      init_frame(renderer);

      draw_frame(rb, window, &plot_regions);

      end_frame(renderer);
    });

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (rb->step() != -1 && running) {

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

    const float *cloud = rb->m_lidar->getRangeImage() + 512;

    const int horizontalResolution = rb->m_lidar->getHorizontalResolution();


    const float *right = cloud + horizontalResolution/4;
    const float *left = cloud + horizontalResolution * 3 / 4;
    const float *front = cloud;

    const double kp = -125.0;
    const double target = 0.06;

    //printf("left %lf right %lf\n", left, right);

    // if(is_blocked(cloud))
    // {
    //   if(std::isfinite(*left))
    //   {
    //     //turn right
    //     while(is_blocked(cloud, true) && rb->step() != -1)
    //       rb->forward(1.0, -1.0);
    //   }
    //   else if(std::isfinite(*right))
    //   {
    //     while(is_blocked(cloud, true) && rb->step() != -1)
    //       rb->forward(-1.0, 1.0);
    //   }
    //   else
    //   {
    //     //turn right
    //     while(is_blocked(cloud, true) && rb->step() != -1)
    //       rb->forward(1.0, -1.0);
    //   }
    // }
    // else if(std::isfinite(*left) && *right > 0.12 /* || (right > 0.15 && left < target+0.2)*/)
    // {
    //   const double cmp_theta = 10/512.0 * 2 * M_PI;

    //   const float *cmp_left = left + 10;

    //   bool failed = false;

    //   //check few sensors for holes
    //   for(int i = 11; i < 20; i++)
    //   {
    //     const float *check_left = left + i;
    //     if(std::isinf(*check_left)) failed = true;
    //   }

      

    //   double err;

    //   if(std::isfinite(*cmp_left) && !failed)
    //   {

    //     double cmp_left_real_dist = cos(cmp_theta) * *cmp_left;

    //     double diff = cmp_left_real_dist - *left;

    //     err = diff;

    //     err += (*left - target) / 10;
    //     err *= kp;
    //     rb->forward(1.0+err, 1.0-err);

    //     printf("diff %lf\n", diff);
    //   }

    //   //double err = *left - target;

    // }
    // else if(std::isfinite(*right))
    // {
    //   const double cmp_theta = 10/512.0 * 2 * M_PI;

    //   const float *cmp_right = right - 10;

    //   bool failed = false;

    //   //check few sensors for holes
    //   for(int i = 1; i < 11; i++)
    //   {
    //     const float *check_right = right - i;
    //     if(std::isinf(*check_right)) failed = true;
    //   }

    //   double err;

    //   if(std::isfinite(*cmp_right) && !failed)
    //   {

    //     double cmp_left_real_dist = cos(cmp_theta) * *cmp_right;

    //     double diff = cmp_left_real_dist - *right;

    //     err = diff;

    //     err += (*right - target) / 10;
    //     err *= kp;
    //     rb->forward(1.0-err, 1.0+err);

    //     printf("diff %lf\n", diff);
    //   }
    // }
    // // else if(std::isfinite(right))
    // // {
    // //   double err = right - target;
    // //   err *= kp;
    // //   rb->forward(1.0-err, 1.0+err);
    // // }
    // else
    // {
    //   rb->forward(1.0, 1.0);
    // }
    // else
    // {
    //   double err = left - right;
    //   err *= kp;
    //   rb->forward(1.0 - err, 1.0 + err);
    // }
  }

  delete_gui(window, renderer);

  // Enter here exit cleanup code.
  rb->endSimulation();
  rb->destroyInstance();
  return 0;
}
