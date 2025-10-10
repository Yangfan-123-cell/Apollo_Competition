#include <string>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/scenarios/best_parking_space/stage_approaching_parking_spot.h"

namespace apollo {
namespace planning {

// 初始化函数，用于StageBestApproachingParkingSpot阶段的初始化
bool StageBestApproachingParkingSpot::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    // 调用父类Stage的初始化函数
    if (!Stage::Init(config, injector, config_dir, context)) {
        return false;  // 初始化失败则返回false
    }
    // 从上下文中获取配置并复制到scenario_config_
    scenario_config_.CopyFrom(GetContextAs<BestParkingSpaceContext>()->scenario_config);
    return true;
}

// 处理阶段逻辑，进行停车位接近过程中的判断和动作
StageResult StageBestApproachingParkingSpot::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: StageBestApproachingParkingSpot";  // 调试输出阶段信息
    CHECK_NOTNULL(frame);  // 确保frame不为空
    StageResult result;  // 定义阶段结果
    auto scenario_context = GetContextAs<BestParkingSpaceContext>();  // 获取场景上下文

    // 如果目标停车位ID为空，则返回错误状态
    if (scenario_context->target_parking_spot_id.empty()) {
        return result.SetStageStatus(StageStatusType::ERROR);  // 设置状态为错误
    }

    // 设置帧的开放空间信息，填充目标停车位ID和预停车点的信息
    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = scenario_context->target_parking_spot_id;
    frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(scenario_context->pre_stop_rightaway_flag);
    *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point())
            = scenario_context->pre_stop_rightaway_point;

    // 遍历参考线，查找目的障碍物，并忽略该障碍物的决策
    auto* reference_lines = frame->mutable_reference_line_info();
    for (auto& reference_line : *reference_lines) {
        auto* path_decision = reference_line.path_decision();
        if (nullptr == path_decision) {
            continue;
        }
        auto* dest_obstacle = path_decision->Find(FLAGS_destination_obstacle_id);//quanjucanshu
        if (nullptr == dest_obstacle) {
            continue;
        }
        // 创建一个“忽略”决策，并添加到目标障碍物的决策列表中
        ObjectDecisionType decision;
        decision.mutable_ignore();
        dest_obstacle->EraseDecision();
        dest_obstacle->AddLongitudinalDecision("ignore-dest-in-valet-parking", decision);
    }

    // 执行任务并根据结果返回状态
    result = ExecuteTaskOnReferenceLine(planning_init_point, frame);

    // 更新场景上下文中的停车标志和停车点信息
    scenario_context->pre_stop_rightaway_flag = frame->open_space_info().pre_stop_rightaway_flag();
    scenario_context->pre_stop_rightaway_point = frame->open_space_info().pre_stop_rightaway_point();

    // 如果车速小于停止条件，设置下一阶段为停车
    if (CheckADCStop(*frame)) {
        next_stage_ = "BEST_PARKING_PARKING";
        return StageResult(StageStatusType::FINISHED);  // 当前阶段完成，返回完成状态
    }

    // 如果发生错误，输出错误日志并返回错误状态
    if (result.HasError()) {
        AERROR << "StopSignUnprotectedStagePreStop planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    return result.SetStageStatus(StageStatusType::RUNNING);  // 当前阶段运行中
}

// 检查自动驾驶车辆是否已停止，满足停车条件
bool StageBestApproachingParkingSpot::CheckADCStop(const Frame& frame) {
    // 获取参考线信息和车辆速度
    const auto& reference_line_info = frame.reference_line_info().front();
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    const double max_adc_stop_speed
            = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();
    
    // 如果车速大于最大停止速度，则返回false，表示车辆尚未停下
    if (adc_speed > max_adc_stop_speed) {
        ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
        return false;
    }

    // 获取车辆前沿与停车线的距离
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double stop_fence_start_s = frame.open_space_info().open_space_pre_stop_fence_s();
    const double distance_stop_line_to_adc_front_edge = stop_fence_start_s - adc_front_edge_s;

    // 如果车与停车线的距离超过预设的最大有效停止距离，返回false
    if (distance_stop_line_to_adc_front_edge > scenario_config_.max_valid_stop_distance()) {
        ADEBUG << "not a valid stop. too far from stop line.";
        return false;
    }
    return true;  // 满足停止条件，返回true
}

}  // namespace planning
}  // namespace apollo
