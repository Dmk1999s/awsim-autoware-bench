// Copyright 2026 loik1235
// Licensed under the Apache License, Version 2.0

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner
{

/// 매 주기 매니저가 노드 파라미터에서 갱신한다 — enable 을 런타임에 끄고 켤 수 있게.
struct CurveSlowdownParam
{
  bool enable{true};
  double max_lateral_accel{0.8};
  double min_velocity{1.5};
  double preview_distance{20.0};
  double curvature_threshold{0.02};
  double sample_interval{1.0};
  double curvature_span{3.0};
};

class CurveSlowdownModule : public experimental::SceneModuleInterface
{
public:
  CurveSlowdownModule(
    const lanelet::Id module_id, const std::shared_ptr<CurveSlowdownParam> param,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(
    experimental::Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  std::shared_ptr<CurveSlowdownParam> param_;

  // 디버그·가상벽용: 이번 주기에 감속을 시작한 지점
  std::optional<geometry_msgs::msg::Pose> slowdown_pose_;
  double slowdown_velocity_{0.0};
};

}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
