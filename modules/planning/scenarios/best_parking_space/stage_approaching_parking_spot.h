#pragma once

#include <memory>
#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/best_parking_space/best_parking_space.h"

#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

// StageBestApproachingParkingSpot 类继承自 Stage，用于处理自动驾驶车辆接近停车位的逻辑。
class StageBestApproachingParkingSpot : public Stage {
public:
    // 初始化函数，设置该阶段的配置、注入器和上下文等信息
    bool Init(
            const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir,
            void* context);

    // 处理阶段逻辑，在接近停车位时执行相关操作
    StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

private:
    // 检查自动驾驶车辆是否已经停下并满足停车条件
    bool CheckADCStop(const Frame& frame);

    // 场景配置，包括停车场景的相关设置
    BestParkingSpaceConfig scenario_config_;
};

// 将 StageBestApproachingParkingSpot 注册为插件，以便动态加载和管理
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StageBestApproachingParkingSpot, Stage)

}  // namespace planning
}  // namespace apollo
