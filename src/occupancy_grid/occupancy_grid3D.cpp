#include "occupancy_grid3D.hpp"
#include "visualizer.hpp"


#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>

namespace occupancy_grid {

void OccupancyGrid3D::InitializeGridFromPointCloud(OccupancyGrid3D& occupancy_grid, const dataloader::Dataset& dataset, double voxel_resolution) {

    Eigen::Vector3d min_bound(std::numeric_limits<double>::max(), 
                              std::numeric_limits<double>::max(), 
                              std::numeric_limits<double>::max());
    Eigen::Vector3d max_bound(std::numeric_limits<double>::lowest(), 
                              std::numeric_limits<double>::lowest(), 
                              std::numeric_limits<double>::lowest());

    auto t0 = std::chrono::high_resolution_clock::now();

    size_t point_counter = 0;

    for (int i=0;i<dataset.size();++i) { //dataset.size()

        auto [pose, cloud] = dataset[i];
        const Eigen::Matrix3d& R = pose.block<3,3>(0,0);
        const Eigen::Vector3d& t = pose.block<3,1>(0,3);

        for (auto& p : cloud) {
            p = R * p + t;

            min_bound = min_bound.cwiseMin(p);
            max_bound = max_bound.cwiseMax(p);
            point_counter++;
        }

    }
    
    size_t grid_dimensions_x = static_cast<size_t>(std::ceil((max_bound.x() - min_bound.x()) / voxel_resolution)) + 1;
    size_t grid_dimensions_y = static_cast<size_t>(std::ceil((max_bound.y() - min_bound.y()) / voxel_resolution)) + 1;
    size_t grid_dimensions_z = static_cast<size_t>(std::ceil((max_bound.z() - min_bound.z()) / voxel_resolution)) + 1;


    occupancy_grid.SetVoxelResolution(voxel_resolution);
    occupancy_grid.SetOrigin(min_bound);
    occupancy_grid.SetGridDimensions({grid_dimensions_x, grid_dimensions_y, grid_dimensions_z});
    occupancy_grid.SetOccupancyGrid(point_counter);


    auto t1 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();


    std::cout << "Grid Initialized: " << grid_dimensions_x << "x" << grid_dimensions_y << "x" <<  grid_dimensions_z
              << " voxels at origin (" << min_bound.transpose() << ") which took " << duration << " ms." << std::endl;

}

Eigen::Vector3i OccupancyGrid3D::PointToVoxel(const Eigen::Vector3d& point) const {
    return {
        static_cast<int>(std::floor((point.x() - origin_.x()) / voxel_resolution_)),
        static_cast<int>(std::floor((point.y() - origin_.y()) / voxel_resolution_)),
        static_cast<int>(std::floor((point.z() - origin_.z()) / voxel_resolution_))
    };
}

Eigen::Vector3d OccupancyGrid3D::VoxelToPoint(const Eigen::Vector3i& voxel) const {
    return {
        origin_.x() + (voxel.x() + 0.5) * voxel_resolution_,
        origin_.y() + (voxel.y() + 0.5) * voxel_resolution_,
        origin_.z() + (voxel.z() + 0.5) * voxel_resolution_
    };
}

const std::unordered_map<Eigen::Vector3i, float, Vector3iHash>&  OccupancyGrid3D::GetVoxels() const {
    return log_odds_;
}

void OccupancyGrid3D::SetVoxelResolution(const double voxel_resolution) {
    voxel_resolution_ = voxel_resolution;
}
void OccupancyGrid3D::SetOrigin(const Eigen::Vector3d& origin) {
    origin_ = origin;
}
void OccupancyGrid3D::SetGridDimensions(const std::array<size_t, 3>& grid_dimensions) {
    grid_dimensions_ = grid_dimensions;
}

void OccupancyGrid3D::SetOccupancyGrid(const size_t& point_count) {
    log_odds_.reserve(point_count);
}

void OccupancyGrid3D::UpdateVoxelLogOdds(const Eigen::Vector3i& key, float delta) {
      auto& cell = log_odds_[key];  
      cell = std::clamp(cell + delta, kLogOddsMin, kLogOddsMax);
}

void OccupancyGrid3D::ComputeRayVoxels(const Eigen::Vector3i& start, const Eigen::Vector3i& end, std::vector<Eigen::Vector3i>& ray)  {

    ray.clear();
    ray.reserve(std::abs(end.x() - start.x()) + 
                std::abs(end.y() - start.y()) + 
                std::abs(end.z() - start.z()));

    if (start == end) {
        ray.push_back(end);
        return;
    }

    int dx = std::abs(end.x() - start.x());
    int dy = std::abs(end.y() - start.y());
    int dz = std::abs(end.z() - start.z());

    int xs = (end.x() > start.x()) ? 1 : -1;
    int ys = (end.y() > start.y()) ? 1 : -1;
    int zs = (end.z() > start.z()) ? 1 : -1;

    int x = start.x(), y = start.y(), z =  start.z();

    if (dx >= dy && dx >=dz) {

        int p1 = 2 * dy - dx;
        int p2 = 2 * dz - dx;

        while (x != end.x()) {

            ray.emplace_back(Eigen::Vector3i{x,y,z});
            x += xs;

            if(p1 >= 0) {
                y += ys;
                p1 -= 2 * dx;
            }

            if(p2 >= 0) {
                z += zs;
                p2 -= 2 * dx;
            }

            p1 += 2 * dy;
            p2 += 2 * dz;

        }

    }
    else if (dy >= dx && dy >= dz) {

        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;

        while (y != end.y()) {

            ray.emplace_back(Eigen::Vector3i{x,y,z});
            y += ys;

            if(p1 >= 0) {
                x += xs;
                p1 -= 2 * dy;
            }

            if(p2 >= 0) {
                z += zs;
                p2 -= 2 * dy;
            }

            p1 += 2 * dx;
            p2 += 2 * dz;

        }

    }
    else {

        int p1 = 2 * dx - dz;
        int p2 = 2 * dy - dz;

        while (z != end.z()) {

            ray.emplace_back(Eigen::Vector3i{x,y,z});
            z += zs;

            if(p1 >= 0) {
                x += xs;
                p1 -= 2 * dz;
            }

            if(p2 >= 0) {
                y += ys;
                p2 -= 2 * dz;
            }

            p1 += 2 * dx;
            p2 += 2 * dy;

        }

    }

    ray.emplace_back(Eigen::Vector3i{x,y,z});

}

void OccupancyGrid3D::GridMappingWithKnownPoses(OccupancyGrid3D& occupancy_grid, const dataloader::Dataset& dataset) {

    std::vector<Eigen::Vector3i> ray_buffer;
    int counter = 0;
    for (int i=0;i<dataset.size();++i) { // dataset.size()

        auto [pose, cloud] = dataset[i];
        const Eigen::Matrix3d& R = pose.block<3,3>(0,0);
        const Eigen::Vector3d& t = pose.block<3,1>(0,3);
        for (auto& p : cloud) {
            p = R * p + t;
        }
        const Eigen::Vector3d origin = pose.block<3,1>(0,3);
        const Eigen::Vector3i start = occupancy_grid.PointToVoxel(origin);
        auto t0 = std::chrono::high_resolution_clock::now();
        for (const auto& point : cloud) {
            const Eigen::Vector3i end = occupancy_grid.PointToVoxel(point);
            occupancy_grid.ComputeRayVoxels(start, end, ray_buffer);

            for (int j = 0; j < ray_buffer.size()-1; ++j) {
                occupancy_grid.UpdateVoxelLogOdds(ray_buffer[j], kLogOddsFree);
            }
            occupancy_grid.UpdateVoxelLogOdds(ray_buffer.back(), kLogOddsOccupied);
            
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        if(counter % 100 == 0) {
            std::cout << "Iteration : " << counter  << " took " << duration << " ms to compute." << std::endl;
        }

        counter++;
    }
}

void OccupancyGrid3D::VisualizeMap(const OccupancyGrid3D& occupancy_grid, const float& probability_threshold) {

    std::vector<Eigen::Vector3d> occupied_points;

    auto prob2log = [](float p) {
        return std::log(p / (1.0 - p));
    };

    float treshold_logodds = prob2log(probability_threshold);

    for (const auto& [key, log_odds] : occupancy_grid.GetVoxels()) {
        if (log_odds > treshold_logodds) { 
            occupied_points.emplace_back(occupancy_grid.VoxelToPoint(key));
        }
    }

    std::cout << "Occupied voxels: " << occupied_points.size() << std::endl;
    visualize(occupied_points);
}


} // namespace occupancy_grid