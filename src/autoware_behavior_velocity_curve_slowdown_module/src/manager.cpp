// Copyright 2026 loik1235
// Licensed under the Apache License, Version 2.0

#include "manager.hpp"

#include <memory>
#include <string>

namespace autoware::behavior_velocity_planner
{

CurveSlowdownModuleManager::CurveSlowdownModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName()), param_(std::make_shared<CurveSlowdownParam>())
{
  const std::string ns(getModuleName());
  param_->enable = experimental::get_or_declare_parameter<bool>(node, ns + ".enable");
  param_->max_lateral_accel =
    experimental::get_or_declare_parameter<double>(node, ns + ".max_lateral_accel");
  param_->min_velocity = experimental::get_or_declare_parameter<double>(node, ns + ".min_velocity");
  param_->preview_distance =
    experimental::get_or_declare_parameter<double>(node, ns + ".preview_distance");
  param_->curvature_threshold =
    experimental::get_or_declare_parameter<double>(node, ns + ".curvature_threshold");
  param_->sample_interval =
    experimental::get_or_declare_parameter<double>(node, ns + ".sample_interval");
}

void CurveSlowdownModuleManager::launchNewModules(
  [[maybe_unused]] const experimental::Trajectory & path,
  [[maybe_unused]] const rclcpp::Time & stamp, const PlannerData & planner_data)
{
  // 매 주기 파라미터를 다시 읽는다. A/B 를 스택 재기동 없이 하기 위한 것으로,
  // 재기동은 그 자체가 조건 차이를 만들어 비교를 흐린다 (run_ab.sh 와 같은 이유).
  const std::string ns(getModuleName());
  param_->enable = node_.get_parameter(ns + ".enable").as_bool();
  param_->max_lateral_accel = node_.get_parameter(ns + ".max_lateral_accel").as_double();
  param_->min_velocity = node_.get_parameter(ns + ".min_velocity").as_double();
  param_->preview_distance = node_.get_parameter(ns + ".preview_distance").as_double();
  param_->curvature_threshold = node_.get_parameter(ns + ".curvature_threshold").as_double();

  // 지도상 특정 요소에 붙는 모듈이 아니라 경로 전체를 보는 모듈이라 인스턴스는 하나면 된다
  const lanelet::Id module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(
      std::make_shared<CurveSlowdownModule>(
        module_id, param_, logger_.get_child(getModuleName()), clock_, time_keeper_,
        planning_factor_interface_),
      planner_data);
  }
}

std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
CurveSlowdownModuleManager::getModuleExpiredFunction(
  [[maybe_unused]] const experimental::Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  return
    []([[maybe_unused]] const std::shared_ptr<experimental::SceneModuleInterface> & scene_module)
      -> bool { return false; };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::CurveSlowdownModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
