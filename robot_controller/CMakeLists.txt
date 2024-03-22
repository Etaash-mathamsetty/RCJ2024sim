cmake_minimum_required(VERSION 3.6)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp imgui/*.cpp)
set (CMAKE_CXX_STANDARD 17)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# Set the  Webots home path (change it according to your installation method)
if(NOT DEFINED WEBOTS_HOME)
  set(WEBOTS_HOME "/usr/local/webots")
endif()
#set(WEBOTS_HOME "/snap/webots/current/usr/share/webots")

# Link with the Webots controller library.
link_directories(${WEBOTS_HOME}/lib/controller ${OPENCV_LIBRARY_DIRS} ${SDL2_LIBRARY_DIRS})
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp ${OpenCV_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/imgui ${CMAKE_SOURCE_DIR})

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES} ${SDL2_LIBRARIES} ${OpenCV_LIBS})

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
