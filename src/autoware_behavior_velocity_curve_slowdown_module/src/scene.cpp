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
namespace
{
/// 경로 위 s 지점의 곡률을 ±span 떨어진 세 점의 외접원으로 잰다.
///
/// Trajectory::curvature() 를 그대로 쓰면 안 된다 — 그 값은 2 m 간격 경로점을 스플라인으로
/// 보간한 곡선의 해석적 곡률이라 기하 곡률보다 크게 나온다. 실측: 같은 좌회전에서
/// 보간 곡률 0.106 (R=9.4 m) vs 경로점 3점 기하 0.071 (R=14.0 m), 지도 중심선 0.069.
/// 곡률이 1.5배로 부풀면 상한 v=√(a/κ) 가 낮아져 모듈이 필요 이상으로 감속한다 (WORKLOG 18).
double span_curvature(const experimental::Trajectory & path, const double s, const double span)
{
  const double len = path.length();
  const double a_s = std::max(0.0, s - span);
  const double c_s = std::min(len, s + span);
  if (c_s - a_s < 1e-3) {
    return 0.0;
  }
  const auto a = path.compute(a_s).point.pose.position;
  const auto b = path.compute(s).point.pose.position;
  const auto c = path.compute(c_s).point.pose.position;
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(a.x - c.x, a.y - c.y);
  const double denom = ab * bc * ca;
  if (denom < 1e-6) {
    return 0.0;
  }
  const double area2 = std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
  return 2.0 * area2 / denom;   // 4 * (면적) / (세 변의 곱)
}
}  // namespace

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
  slowdown_velocity_ = 0.0;

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
    const double curvature = span_curvature(path, s, p.curvature_span);
    if (curvature < p.curvature_threshold) {
      continue;
    }
    const double v_limit =
      std::max(p.min_velocity, std::sqrt(p.max_lateral_accel / curvature));

    const double from = std::max(0.0, s - p.preview_distance);
    const double to = std::min(length, s + p.sample_interval);
    path.longitudinal_velocity_mps().range(from, to).clamp(static_cast<float>(v_limit));

    if (!modified) {
      // 감속을 시작하는 지점 (처음 제한이 걸린 곳)
      slowdown_pose_ = path.compute(from).point.pose;
      modified = true;
    }
    // 보고할 속도는 "가장 낮은 상한"이다. 처음 걸린 점의 값을 쓰면 실제로 차를 묶는 값과
    // 다르다 — 그 차이 때문에 스윕 분석에서 상한 대비 실제 속도가 65%로 잘못 보였다.
    slowdown_velocity_ = modified && slowdown_velocity_ > 0.0
                           ? std::min(slowdown_velocity_, v_limit)
                           : v_limit;
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
