// best_parking_space.h - 更新后的头文件

#pragma once

// 引入所需的头文件
#include <memory>
#include <string>
#include <vector>
#include <limits> // For std::numeric_limits

#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/planning/scenarios/best_parking_space/proto/best_parking_space.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"
#include "gtest/gtest.h" // While gtest is typically for tests, it was in your original, so keeping it.
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/dependency_injector.h" // Added from your updated snippet
#include "modules/planning/planning_base/common/planning_context.h"   // Added from your updated snippet
#include "modules/common/math/vec2d.h"                               // Added from your updated snippet

namespace apollo {
namespace planning {

// 定义BestParkingSpaceContext结构体，继承自ScenarioContext
// 用于存储场景相关的配置和状态信息
struct BestParkingSpaceContext : public ScenarioContext {
  BestParkingSpaceConfig scenario_config;      // 场景配置
  std::string target_parking_spot_id;          // 目标停车位ID
  bool pre_stop_rightaway_flag = false;        // 是否立即停下的标志
  hdmap::MapPathPoint pre_stop_rightaway_point; // 停车前的路径点
};

// 最优停车位结构体
struct OptimalParkingSpot {
  int scenario_id;                                // 场景ID
  std::vector<apollo::common::math::Vec2d> corners; // 四个角点
  apollo::common::math::Vec2d center_point;         // 中心点
};

// BestParkingSpaceScenario类继承自Scenario，用于定义停车位选择场景
class BestParkingSpaceScenario : public Scenario {
 public:
  // 初始化函数，重写父类的Init函数
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  /**
   * @brief 获取场景上下文。
   * @return 返回场景上下文的指针
   */
  BestParkingSpaceContext* GetContext() override {
    return &context_; // 返回BestParkingSpaceContext对象的地址
  }

  /**
   * @brief 判断是否能转移到另一个场景
   */
  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;

 private:
  bool IsPointInPolygon(const apollo::common::math::Vec2d& point, 
                         const std::vector<apollo::common::math::Vec2d>& polygon);
  std::vector<apollo::common::math::Vec2d> GetObstacleCornerPoints(
        const apollo::common::math::Vec2d& center, 
        double theta_radians);
  // 静态函数：在路径上查找目标停车位
  static bool SearchTargetParkingSpotOnPath(
      const hdmap::Path& nearby_path, const std::string& target_parking_id,
      hdmap::PathOverlap* parking_space_overlap);

  // 静态函数：检查车辆与停车位之间的距离
  static bool CheckDistanceToParkingSpot(
      const Frame& frame, const common::VehicleState& vehicle_state,
      const hdmap::Path& nearby_path, const double parking_start_range,
      const hdmap::PathOverlap& parking_space_overlap);

  /**
   * @brief 初始化预定义的最优停车位坐标
   */
  void InitOptimalParkingSpots();

  /**
   * @brief 计算四个角点的中心点
   * @param corners 四个角点坐标
   * @return 中心点坐标
   */
  apollo::common::math::Vec2d CalculateCenterPoint(
      const std::vector<apollo::common::math::Vec2d>& corners);

  /**
   * @brief 根据场景ID和最优位置查找最近的停车位ID
   * @param scenario_id 场景ID (1, 2, 3, 4)
   * @param search_range 搜索半径（米）
   * @return 最近的停车位ID，如果未找到返回空字符串
   */
  std::string FindParkingSpotIdByOptimalPosition(int scenario_id,
                                                   double search_range = 10.0);


 private:
  bool init_ = false;                             // 标志是否已初始化
  BestParkingSpaceContext context_;               // 场景上下文
  const hdmap::HDMap* hdmap_ = nullptr;           // 地图信息指针

  // 存储所有预定义的最优停车位
  std::vector<OptimalParkingSpot> optimal_parking_spots_;
};

// 注册BestParkingSpaceScenario插件
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::BestParkingSpaceScenario,
                                     Scenario)

} // namespace planning
} // namespace apollo