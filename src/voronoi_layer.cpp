/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  Copyright (c) 2021, Samsung Research America
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *                         stevenmacenski@gmail.com
 *********************************************************************/

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

#include "voronoi_layer/voronoi_layer.hpp"

namespace voronoi_layer
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using rcl_interfaces::msg::ParameterType;

/*****************************************************************************/
VoronoiLayer::VoronoiLayer(void)
/*****************************************************************************/
{

}

/*****************************************************************************/
VoronoiLayer::~VoronoiLayer(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void VoronoiLayer::onInitialize(void)
/*****************************************************************************/
{
  RCLCPP_INFO( logger_, "%s being initialized as VoronoiLayer!", getName().c_str());

  bool track_unknown_space;
  double transform_tolerance, map_save_time;
  int decay_model_int;
  // source names
  auto node = node_.lock();

  // whether to default on
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  RCLCPP_INFO(logger_, "%s created underlying voxel grid.", getName().c_str());

  matchSize();
  current_ = true;

  RCLCPP_INFO(logger_, "Configuring plugin %s ", name_.c_str());
  RCLCPP_INFO(logger_, "%s initialization complete!", getName().c_str());
}


/*****************************************************************************/
void VoronoiLayer::activate(void)
/*****************************************************************************/
{
  // subscribe and place info in buffers from sensor sources
  RCLCPP_INFO(logger_, "%s was activated.", getName().c_str());
  
}

/*****************************************************************************/
void VoronoiLayer::deactivate(void)
/*****************************************************************************/
{
  // unsubscribe from all sensor sources
  RCLCPP_INFO(logger_, "%s was deactivated.", getName().c_str());
}

/*****************************************************************************/
void VoronoiLayer::reset(void)
/*****************************************************************************/
{
  // reset layer
  Costmap2D::resetMaps();
}

/*****************************************************************************/
bool VoronoiLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
/*****************************************************************************/
{
  return false;
}

/*****************************************************************************/
void VoronoiLayer::clearArea(
  int start_x, int start_y, int end_x, int end_y, bool invert_area)
/*****************************************************************************/
{
 CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert_area);
}

void VoronoiLayer::updateBounds(double robot_x, double robot_y,
                                double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y) {
  
  // grabs new max bounds for the costmap
  if (!enabled_) {
    return;
  }

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
 
  useExtraBounds(min_x, min_y, max_x, max_y);
  
}


/*****************************************************************************/
void VoronoiLayer::matchSize(void)
/*****************************************************************************/
{
  // match the master costmap size, volume_grid maintains full w/ expiration.
  CostmapLayer::matchSize();
}

/*****************************************************************************/
void VoronoiLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
    // update costs in master_grid with costmap_
    if (!enabled_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    UpdateDynamicVoronoi(master_grid);
    voronoi_.update();
    voronoi_.visualize();
    voronoi_.prune();
}


void VoronoiLayer::UpdateDynamicVoronoi(const nav2_costmap_2d::Costmap2D& master_grid) {

  Costmap2D::resetMaps();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  if (last_size_x_ != size_x || last_size_y_ != size_y) {
    voronoi_.initializeEmpty(static_cast<int>(size_x), static_cast<int>(size_y));
    last_size_x_ = size_x;
    last_size_y_ = size_y;
  }

  std::vector<IntPoint> new_free_cells;
  std::vector<IntPoint> new_occupied_cells;
  for (int j = 0; j < static_cast<int>(size_y); ++j) {
    for (int i = 0; i < static_cast<int>(size_x); ++i) {
      if (voronoi_.isOccupied(i, j) && master_grid.getCost(i, j) == nav2_costmap_2d::FREE_SPACE) {
             new_free_cells.emplace_back(i, j);
      }
      if (!voronoi_.isOccupied(i, j) && master_grid.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        new_occupied_cells.emplace_back(i, j);
      }
    }
  }

  for (const IntPoint& cell : new_free_cells) {
    voronoi_.clearCell(cell.x, cell.y);
  }

  for (const IntPoint& cell : new_occupied_cells) {
    voronoi_.occupyCell(cell.x, cell.y);
  }
}

}  // namespace voronoi_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(voronoi_layer::VoronoiLayer, nav2_costmap_2d::Layer)
