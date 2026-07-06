cmake_minimum_required(VERSION 3.24)

project(occupancy_grid_3d VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(dataloader STATIC
    src/dataloader/dataloader.cpp
)

target_compile_features(dataloader PRIVATE cxx_std_17)
target_include_directories(dataloader PUBLIC
    src/dataloader
)
target_link_libraries(dataloader PUBLIC
    Open3D::Open3D
    Eigen3::Eigen
)

add_library(visualizer INTERFACE)
target_include_directories(visualizer INTERFACE
    src/visualizer
)
target_link_libraries(visualizer INTERFACE
    Open3D::Open3D
    Eigen3::Eigen
)

add_library(occupancy_grid STATIC
    src/occupancy_grid/occupancy_grid3D.cpp
)
target_compile_features(occupancy_grid PRIVATE cxx_std_17)
target_include_directories(occupancy_grid PUBLIC
    src/occupancy_grid
)
target_link_libraries(occupancy_grid PUBLIC
    Eigen3::Eigen
    visualizer
    dataloader
)

add_executable(occupancy_grid_3d
    src/main.cpp
)

target_include_directories(occupancy_grid_3d PRIVATE
    include
)

target_link_libraries(occupancy_grid_3d PRIVATE
    dataloader
    visualizer
    occupancy_grid
)


