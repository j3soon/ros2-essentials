#ifndef COST_TRAVERSABILITY_HPP
#define COST_TRAVERSABILITY_HPP__

#include "sl/Fusion.hpp"

namespace stereolabs
{
namespace cost_traversability
{
/**
\brief Defines robot parameters, this will be used to compute the TRAVERSABILITY_COST
*/
struct RobotParameters
{
  float radius = 0.25f;
  float step_max = 0.1f;
  float slope_max = 20.0f;           // degrees
  float roughness_max = 0.1f;
};

/**
        \brief Defines the traversability parameters, this will be used to compute the TRAVERSABILITY_COST
         */
struct TraversabilityParameters
{
  float occupancy_threshold = 0.5f;
  float slope_weight = 1.f / 3.f;
  float step_weight = 1.f / 3.f;
  float roughness_weight = 1.f / 3.f;
};

constexpr sl::LayerName TRAVERSABILITY_COST =
  static_cast<sl::LayerName>(static_cast<int>(sl::LayerName::LAST) + 1);
constexpr sl::LayerName OCCUPANCY =
  static_cast<sl::LayerName>(static_cast<int>(sl::LayerName::LAST) + 2);
constexpr sl::LayerName TRAVERSABILITY_COST_STEP =
  static_cast<sl::LayerName>(static_cast<int>(sl::LayerName::LAST) + 3);
constexpr sl::LayerName TRAVERSABILITY_COST_SLOPE =
  static_cast<sl::LayerName>(static_cast<int>(sl::LayerName::LAST) + 4);
constexpr sl::LayerName TRAVERSABILITY_COST_ROUGHNESS =
  static_cast<sl::LayerName>(static_cast<int>(sl::LayerName::LAST) + 5);

constexpr float OCCUPIED_CELL = 1.f;
constexpr float FREE_CELL = 0.f;
constexpr float INVALID_CELL_DATA = NAN;
constexpr float UNKNOWN_CELL = NAN;

void initCostTraversibily(
  sl::Terrain & cost_terrain, float resolution, float range,
  float height_threshold);

void computeCost(
  sl::Terrain & elevation_terrain, sl::Terrain & cost_terrain,
  const float grid_resolution, RobotParameters robot_parameters,
  TraversabilityParameters traversability_parameters);

void normalization(sl::Terrain & cost_terrain, sl::LayerName layer, sl::Mat & view);

}     // namespace cost_traversability

} // namespace stereolabs

// SDK internal function
namespace plane
{
void compute_pca(
  std::vector<sl::float3> & points, sl::float3 & normal_vect, sl::float3 & centroid,
  sl::float3 & eigen_values);
}

#endif // COST_TRAVERSABILITY_HPP_
