// Copyright 2026 loik1235
// Licensed under the Apache License, Version 2.0

#include "scene.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

CurveSlowdownModule::CurveSlowdownModule(
  const lanelet::Id module_id, const std::shared_ptr<CurveSlowdownParam> param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  param_(param)
{
}

bool CurveSlowdownModule::modifyPathVelocity(
  experimental::Trajectory & path,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  [[maybe_unused]] const PlannerData & planner_data)
{
  slowdown_pose_.reset();

  const auto & p = *param_;
  if (!p.enable) {
    return false;
  }

  const double length = path.length();
  if (length < p.sample_interval) {
    return false;
  }

  // 경로를 일정 간격으로 훑으며 곡률에서 속도 상한을 만든다.
  // 상한은 곡선 지점이 아니라 그 preview_distance 앞에서부터 적용한다 — 곡선에 들어간 뒤
  // 줄이면 이미 횡가속도를 겪은 뒤다. velocity_smoother 의 횡가속도 필터와 다른 점이 이것이다
  // (그쪽 decel_distance_before_curve 는 3.5 m).
  bool modified = false;
  for (double s = 0.0; s <= length; s += p.sample_interval) {
    const double curvature = std::abs(path.curvature(s));
    if (curvature < p.curvature_threshold) {
      continue;
    }
    const double v_limit =
      std::max(p.min_velocity, std::sqrt(p.max_lateral_accel / curvature));

    const double from = std::max(0.0, s - p.preview_distance);
    const double to = std::min(length, s + p.sample_interval);
    path.longitudinal_velocity_mps().range(from, to).clamp(static_cast<float>(v_limit));

    if (!modified) {
      // 이번 주기에 처음 제한이 걸린 지점을 감속 시작점으로 남긴다
      slowdown_pose_ = path.compute(from).point.pose;
      slowdown_velocity_ = v_limit;
      modified = true;
    }
  }

  if (modified && slowdown_pose_) {
    // 감속 사유를 factor 로 남긴다. 기존 횡가속도 필터(velocity_smoother)는 사유를 남기지
    // 않아 "왜 느려졌는지"가 기록에 안 잡힌다 — 하네스가 귀속할 수 있게 하는 것이 목적.
    // path 는 자차 부근에서 시작하므로 호(arc) 길이를 자차 기준 거리로 쓴다.
    const double distance = autoware_utils_geometry::calc_distance2d(
      planner_data.current_odometry->pose.position, slowdown_pose_->position);
    planning_factor_interface_->add(
      distance, *slowdown_pose_,
      autoware_internal_planning_msgs::msg::PlanningFactor::SLOW_DOWN,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/,
      slowdown_velocity_, 0.0 /*shift_length*/, "curve");
  }

  return modified;
}

visualization_msgs::msg::MarkerArray CurveSlowdownModule::createDebugMarkerArray()
{
  return visualization_msgs::msg::MarkerArray{};
}

autoware::motion_utils::VirtualWalls CurveSlowdownModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls walls;
  if (!slowdown_pose_) {
    return walls;
  }
  autoware::motion_utils::VirtualWall wall;
  wall.text = "curve_slowdown";
  wall.ns = std::to_string(module_id_) + "_";
  wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  wall.pose = *slowdown_pose_;
  walls.push_back(wall);
  return walls;
}

}  // namespace autoware::behavior_velocity_planner
