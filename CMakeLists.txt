cmake_minimum_required(VERSION 3.11.3)

# Set the C++ standard we will use
set(CMAKE_CXX_STANDARD 17)

# Add the path of the cmake files to the CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

project(route)

# Set library output path to /lib
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

# Locate project prerequisites
find_package(io2d REQUIRED)
find_package(Cairo)
find_package(GraphicsMagick)

# Set IO2D flags
set(IO2D_WITHOUT_SAMPLES 1)
set(IO2D_WITHOUT_TESTS 1)

# Add the pugixml and GoogleTest library subdirectories
add_subdirectory(thirdparty/pugixml)
add_subdirectory(thirdparty/googletest)

# Add project executable
add_executable(route src/main.cpp src/model.cpp src/render.cpp src/route_model.cpp src/route_planner.cpp)

target_link_libraries(route
    PRIVATE io2d::io2d
    PUBLIC pugixml
)

# Set options for Linux or Microsoft Visual C++
if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    target_link_libraries(route PUBLIC pthread)
endif()

if(MSVC)
	target_compile_options(route PUBLIC /D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING /wd4459)
endif()
