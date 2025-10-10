/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

 #include "modules/planning/scenarios/park_and_go/stage_cruise.h"

 #include "cyber/common/log.h"
 #include "modules/planning/planning_base/common/frame.h"
 #include "modules/planning/planning_base/common/planning_context.h"
 #include "modules/planning/planning_base/common/util/common.h"
 #include "modules/planning/scenarios/park_and_go/context.h"
 
 namespace apollo {
 namespace planning {
 
 using apollo::common::TrajectoryPoint;
 
 StageResult ParkAndGoStageCruise::Process(
     const TrajectoryPoint& planning_init_point, Frame* frame) {
   ADEBUG << "stage: Cruise";
   CHECK_NOTNULL(frame);

   const common::VehicleState& vehicle_state = frame->vehicle_state();
   const double ego_x = vehicle_state.x();
   const double ego_y = vehicle_state.y();
   const double ego_heading = vehicle_state.heading();
   
   int total_obstacles = frame->obstacles().size();
   double min_distance = std::numeric_limits<double>::max();
   common::PathPoint obstacle_point;
   bool obstacle_found = false;
   std::string obstacle_id = "";
   
   for (const auto& obstacle : frame->obstacles()) {
     int type = obstacle->Perception().type();
     AINFO << "障碍物id" << obstacle->Id() << "障碍物类型" << type;
     
     if (type != 5) {
       continue;
     }
   
     const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
     double obstacle_x = obstacle->Perception().position().x();
     double obstacle_y = obstacle->Perception().position().y();
     
     double dx = obstacle_x - ego_x;
     double dy = obstacle_y - ego_y;
     
     double distance = std::hypot(dx, dy);
     if (distance < min_distance) {
       min_distance = distance;
       obstacle_point.set_x(obstacle_x);
       obstacle_point.set_y(obstacle_y);
       obstacle_found = true;
       obstacle_id = obstacle->Id();
     }
   }
   
   if (obstacle_found) {
     AINFO << "obstacle id:" << obstacle_id << " Found type-5 obstacle at distance: " << min_distance
           << " meters, position: (" << obstacle_point.x() << ", " 
           << obstacle_point.y() << ")";
     
     ADEBUG << "Front obstacle detected - ID: " << obstacle_id 
            << ", Distance: " << min_distance
            << ", Position: (" << obstacle_point.x() << ", " << obstacle_point.y() << ")";
     
     if (total_obstacles > 30 && min_distance > 30.0) {
       AINFO << "障碍物数量(" << total_obstacles << ") > 30, 最近障碍物距离(" 
             << min_distance << "m) > 30m, 完成该阶段";
       return FinishStage();
     }
     
   } else {
     AINFO << "No type-5 obstacles found around the vehicle";
   }
   
   StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
   if (result.HasError()) {
     AERROR << "ParkAndGoStageCruise planning error";
   }
 
   const ReferenceLineInfo& reference_line_info =
       frame->reference_line_info().front();
   
   ParkAndGoStatus status =
       CheckADCParkAndGoCruiseCompleted(reference_line_info);
 
   if (status == CRUISE_COMPLETE) {
     return FinishStage();
   }
   return result.SetStageStatus(StageStatusType::RUNNING);
 }
 
 StageResult ParkAndGoStageCruise::FinishStage() { return FinishScenario(); }
 
 ParkAndGoStageCruise::ParkAndGoStatus
 ParkAndGoStageCruise::CheckADCParkAndGoCruiseCompleted(
     const ReferenceLineInfo& reference_line_info) {
   const auto& reference_line = reference_line_info.reference_line();
 
   const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                             injector_->vehicle_state()->y()};
   common::SLPoint adc_position_sl;
   reference_line.XYToSL(adc_position, &adc_position_sl);
 
   const double kLBuffer = 0.0;
   if (std::fabs(adc_position_sl.l()) < kLBuffer) {
     ADEBUG << "cruise completed";
     return CRUISE_COMPLETE;
   }
 
   return CRUISING;
 }
 
 }  // namespace planning
 }  // namespace apollo