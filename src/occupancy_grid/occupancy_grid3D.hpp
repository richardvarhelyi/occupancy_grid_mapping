#ifndef FINAL_PROJECT_OCCUPANCY_GRID3D_H_
#define FINAL_PROJECT_OCCUPANCY_GRID3D_H_

#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <open3d/utility/Helper.h>
#include <array>
#include "dataloader.hpp"


namespace occupancy_grid {

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;
using Vector3iHash = open3d::utility::hash_eigen<Eigen::Vector3i>;

constexpr float kLogOddsOccupied = 0.85f;
constexpr float kLogOddsFree = -0.4f;
constexpr float kLogOddsMin = -5.0f;
constexpr float kLogOddsMax = 5.0f;
constexpr float kLogOddsPrior = 0.0f;

class OccupancyGrid3D {
  private:
    std::unordered_map<Eigen::Vector3i, float , Vector3iHash> log_odds_;
    double voxel_resolution_;
    std::array<size_t, 3> grid_dimensions_;
    Eigen::Vector3d origin_;


  public:
    OccupancyGrid3D() = default;
    Eigen::Vector3i PointToVoxel(const Eigen::Vector3d& point) const;
    Eigen::Vector3d VoxelToPoint(const Eigen::Vector3i& voxel) const;
    void ComputeRayVoxels(const Eigen::Vector3i& start, const Eigen::Vector3i& end, std::vector<Eigen::Vector3i>& ray);
    void UpdateVoxelLogOdds(const Eigen::Vector3i& key, float delta); 
    void SetVoxelResolution(const double voxel_resolution);
    void SetOrigin(const Eigen::Vector3d& origin);
    void SetGridDimensions(const std::array<size_t, 3>& grid_dimensions);
    void SetOccupancyGrid(const size_t& point_count);
    const std::unordered_map<Eigen::Vector3i, float, Vector3iHash>& GetVoxels() const;

    static void InitializeGridFromPointCloud(OccupancyGrid3D& occupancy_grid, const dataloader::Dataset& dataset, double voxel_resolution);
    static void GridMappingWithKnownPoses(OccupancyGrid3D& occupancy_grid, const dataloader::Dataset& dataset);
    static void VisualizeMap(const OccupancyGrid3D& occupancy_grid, const float& probability_threshold);
};

}  // namespace occupancy_grid

#endif  // FINAL_PROJECT_OCCUPANCY_GRID3D_H_