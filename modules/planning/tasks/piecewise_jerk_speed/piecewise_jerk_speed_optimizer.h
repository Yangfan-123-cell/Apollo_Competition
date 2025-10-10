#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "modules/planning/tasks/piecewise_jerk_speed/proto/piecewise_jerk_speed.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/speed_optimizer.h"

namespace apollo {
namespace planning {

class PiecewiseJerkSpeedOptimizer : public SpeedOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

  virtual ~PiecewiseJerkSpeedOptimizer() = default;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;
  void AdjustInitStatus(
      const std::vector<std::pair<double, double>> s_dot_bound, double delta_t,
      std::array<double, 3>& init_s);
  double GetSpeedOffsetByObstacles();
  PiecewiseJerkSpeedOptimizerConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::PiecewiseJerkSpeedOptimizer, Task)

}  // namespace planning
}  // namespace apollo
