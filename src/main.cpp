#include <iostream> 
#include "dataloader/dataloader.hpp"
#include "visualizer/visualizer.hpp"
#include "occupancy_grid3D.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>


int main(){

    const dataloader::Dataset dataset = dataloader::Dataset("../data");

    occupancy_grid::OccupancyGrid3D grid;

    occupancy_grid::OccupancyGrid3D::InitializeGridFromPointCloud(grid, dataset, 1.5);
    occupancy_grid::OccupancyGrid3D::GridMappingWithKnownPoses(grid, dataset);
    occupancy_grid::OccupancyGrid3D::VisualizeMap(grid, 0.7);


    return 0;
}