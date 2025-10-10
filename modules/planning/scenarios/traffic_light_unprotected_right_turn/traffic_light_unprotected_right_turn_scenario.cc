#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_creep.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_intersection_cruise.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_stop.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

bool TrafficLightUnprotectedRightTurnScenario::Init(
    std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "初始化场景失败: " << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioTrafficLightUnprotectedRightTurnConfig>(
          &context_.scenario_config)) {
    AERROR << "加载场景配置失败: " << Name();
    return false;
  }
  init_ = true;
  return true;
}

// 判断是否可以切换到红绿灯无保护右转场景
bool TrafficLightUnprotectedRightTurnScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  // 检查是否有车道跟随指令
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  
  // 检查场景和参考线是否有效
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  
  // 查找交通灯overlap，排除停止标志和让行标志
  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      // 如果前方是停止标志或让行标志，不进入红绿灯场景
      return false;
    } else if (overlap.first == ReferenceLineInfo::SIGNAL) {
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    }
  }
  
  // 如果没有找到交通灯overlap，不进入该场景
  if (traffic_sign_overlap == nullptr) {
    return false;
  }
  
  // 获取参考线上所有的交通灯overlap
  const std::vector<hdmap::PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  
  // 获取场景切入的检测距离配置
  const double start_check_distance =
      context_.scenario_config.start_traffic_light_scenario_distance();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  
  // 查找同一组的所有交通灯（距离第一个交通灯2米以内的认为是同一组）
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // 单位：米
  for (const auto& overlap : traffic_light_overlaps) {
    const double dist = overlap.start_s - traffic_sign_overlap->start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      next_traffic_lights.push_back(overlap);
    }
  }
  
  bool traffic_light_scenario = false;
  // 遍历所有同组交通灯，检查是否有红灯/黄灯/未知状态
  for (const auto& overlap : next_traffic_lights) {
    const double adc_distance_to_traffic_light =
        overlap.start_s - adc_front_edge_s;
    AINFO << "交通灯[" << overlap.object_id << "] 起始位置s["
           << overlap.start_s << "] 车辆到交通灯距离["
           << adc_distance_to_traffic_light << "]";

    // 基于距离判断是否进入交通灯场景
    if (adc_distance_to_traffic_light <= 0.0 ||
        adc_distance_to_traffic_light > start_check_distance) {
      continue;  // 距离过近或过远，跳过
    }

    const auto& signal_color = frame.GetSignal(overlap.object_id).color();
    AINFO << "交通灯ID[" << overlap.object_id << "] 起始位置s["
           << overlap.start_s << "] 灯色状态[" << signal_color << "]";

    // 如果不是绿灯且不是黑灯（关闭状态），则需要进入交通灯场景
    if (signal_color != perception::TrafficLight::GREEN &&
        signal_color != perception::TrafficLight::BLACK) {
      traffic_light_scenario = true;
      break;
    }
  }
  
  // 如果所有灯都是绿灯或关闭状态，不需要进入交通灯场景
  if (!traffic_light_scenario) {
    return false;
  }
  
  // 检查当前车道是否为右转车道
  const auto& turn_type =
      reference_line_info.GetPathTurnType(traffic_sign_overlap->start_s);
  if (turn_type != hdmap::Lane::RIGHT_TURN) {
    AINFO<<"不是右转类型，无法进入无保护右转的情况";
    return false;  // 不是右转车道，不进入该场景
  }
  
  // 保存当前交通灯组的所有ID到上下文
  context_.current_traffic_light_overlap_ids.clear();
  for (const auto& overlap : next_traffic_lights) {
    context_.current_traffic_light_overlap_ids.push_back(overlap.object_id);
  }
  
  return true;  // 满足所有条件，可以切换到红绿灯无保护右转场景
}

// 退出场景时清理规划状态
bool TrafficLightUnprotectedRightTurnScenario::Exit(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_traffic_light()
      ->Clear();
  return true;
}

// 进入场景时初始化交通灯状态
bool TrafficLightUnprotectedRightTurnScenario::Enter(Frame* frame) {
  const auto& reference_line_info = frame->reference_line_info().front();
  std::string current_traffic_light_overlap_id;
  
  // 查找第一个遇到的交通灯overlap
  const auto& overlaps = reference_line_info.FirstEncounteredOverlaps();
  for (auto overlap : overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      current_traffic_light_overlap_id = overlap.second.object_id;
      break;
    }
  }

  if (current_traffic_light_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    AERROR << "在参考线中未找到交通灯overlap！";
    return false;
  }

  // 查找同一位置/组的所有交通灯
  const std::vector<apollo::hdmap::PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_itr = std::find_if(
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
      
  if (traffic_light_overlap_itr == traffic_light_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return true;
  }

  // 将同组的所有交通灯添加到规划上下文中
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // 单位：米
  const double current_traffic_light_overlap_start_s =
      traffic_light_overlap_itr->start_s;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_current_traffic_light_overlap_id(
              traffic_light_overlap.object_id);
      AINFO << "更新规划上下文，添加首次遇到的交通灯["
             << traffic_light_overlap.object_id << "] 起始位置s["
             << traffic_light_overlap.start_s << "]";
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo