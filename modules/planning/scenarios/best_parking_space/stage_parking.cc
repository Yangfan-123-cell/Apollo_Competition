#include "modules/planning/scenarios/best_parking_space/stage_parking.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

// 该函数处理停车阶段的规划逻辑
StageResult StageBestParking::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    AINFO << "1";  // 打印调试信息，标识进入该函数

    // Open space规划与前续阶段不同，不使用来自上游的planning_init_point，因为它们的拼接策略不同
    auto scenario_context = GetContextAs<BestParkingSpaceContext>();
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);  // 设置车辆是否在开阔地带轨迹上
    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = scenario_context->target_parking_spot_id;  // 设置目标停车位ID

    // 执行停车任务，参数为当前的frame
    AINFO << "执行任务ExecuteTaskOnOpenSpace";
    StageResult result = ExecuteTaskOnOpenSpace(frame);

    // 如果执行任务过程中发生错误，设置状态为ERROR并返回
    if (result.HasError()) {
        AERROR << "StageParking planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    // 正常运行中返回RUNNING状态
    return result.SetStageStatus(StageStatusType::RUNNING);
}

// 结束停车阶段，设置为已完成
StageResult StageBestParking::FinishStage() {
    return StageResult(StageStatusType::FINISHED);  // 返回阶段完成状态
}

}  // namespace planning
}  // namespace apollo
