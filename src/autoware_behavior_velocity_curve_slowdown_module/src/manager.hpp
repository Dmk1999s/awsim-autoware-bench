// Copyright 2026 loik1235
// Licensed under the Apache License, Version 2.0

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{

class CurveSlowdownModuleManager : public experimental::SceneModuleManagerInterface<>
{
public:
  explicit CurveSlowdownModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "curve_slowdown"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  // 씬 모듈과 공유한다. 매 주기 노드 파라미터에서 갱신하므로 `ros2 param set` 이 바로 먹는다
  std::shared_ptr<CurveSlowdownParam> param_;

  void launchNewModules(
    const experimental::Trajectory & path, const rclcpp::Time & stamp,
    const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
  getModuleExpiredFunction(
    const experimental::Trajectory & path, const PlannerData & planner_data) override;
};

class CurveSlowdownModulePlugin : public experimental::PluginWrapper<CurveSlowdownModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
