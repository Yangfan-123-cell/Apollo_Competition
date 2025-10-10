#pragma once

#include <memory>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/best_parking_space/best_parking_space.h"

namespace apollo {
namespace planning {

// 定义停车阶段类，继承自 Stage 基类
class StageBestParking : public Stage {
public:
    // 重写基类的 Process 方法，处理停车阶段的规划和执行
    StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

private:
    // 完成停车阶段的私有方法
    StageResult FinishStage();

    // 存储停车场景的配置
    BestParkingSpaceConfig scenario_config_;
};

// 将 StageBestParking 注册为插件
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StageBestParking, Stage)

}  // namespace planning
}  // namespace apollo
