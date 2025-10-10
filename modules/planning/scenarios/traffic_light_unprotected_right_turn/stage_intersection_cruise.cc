#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_intersection_cruise.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/context.h"

namespace apollo {
namespace planning {

StageResult TrafficLightUnprotectedRightTurnStageIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedRightTurnStageIntersectionCruise "
           << "plan error";
  }

  bool stage_done = CheckDone(*frame, injector_->planning_context(), false);
  if (stage_done) {
    return FinishStage();
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult
TrafficLightUnprotectedRightTurnStageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace apollo
