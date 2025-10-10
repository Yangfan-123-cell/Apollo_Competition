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

 #include "modules/planning/scenarios/park_and_go/stage_adjust.h"

 #include "cyber/common/log.h"
 #include "modules/planning/planning_base/common/frame.h"
 #include "modules/planning/planning_base/common/util/common.h"
 #include "modules/planning/scenarios/park_and_go/context.h"
 #include "modules/planning/scenarios/park_and_go/util.h"
 
 namespace apollo {
 namespace planning {
 
 using apollo::common::TrajectoryPoint;
 
 StageResult ParkAndGoStageAdjust::Process(
     const TrajectoryPoint& planning_init_point, Frame* frame) {
   ADEBUG << "stage: Adjust";
   CHECK_NOTNULL(frame);
 
   // 遍历障碍物，获取车前方第一个障碍物的位置和距离
   const common::VehicleState& vehicle_state = frame->vehicle_state();
   const double ego_x = vehicle_state.x();
   const double ego_y = vehicle_state.y();
   const double ego_heading = vehicle_state.heading();
   
   double min_distance = std::numeric_limits<double>::max();
   common::PathPoint obstacle_point;
   bool obstacle_found = false;
   std::string obstacle_id = "";
   int zy123=0;
   for (const auto& obstacle : frame->obstacles()) {
     int type = obstacle->Perception().type();
     AINFO<<"障碍物id"<<obstacle->Id()<<"障碍物类型"<<type;
     // 仅考虑类型为5的障碍物
     if (type != 5) {
       continue;
     }
   
     const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
     // 不再判断障碍物是否在车前方，考虑所有障碍物
     double obstacle_x = obstacle->Perception().position().x();
     double obstacle_y = obstacle->Perception().position().y();
     
     // 计算障碍物与车辆的距离
     double dx = obstacle_x - ego_x;
     double dy = obstacle_y - ego_y;
     
     // 计算障碍物与车辆的欧几里得距离
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
     
     // 记录障碍物信息用于调试，但不直接修改planning_context
     // 因为proto文件中可能没有定义相应字段
     ADEBUG << "Front obstacle detected - ID: " << obstacle_id 
            << ", Distance: " << min_distance
            << ", Position: (" << obstacle_point.x() << ", " << obstacle_point.y() << ")";
     
     // 当类型为5的障碍物距离大于30米时，完成当前阶段
     if (min_distance > 35.0) {
       AINFO << "Type-5 obstacle distance > 30 meters, finishing stage directly";
       return FinishStage();
     }
     
   } else {
     AINFO << "No type-5 obstacles found around the vehicle";
   }
 
   frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
   StageResult result = ExecuteTaskOnOpenSpace(frame);
   if (result.HasError()) {
     AERROR << "ParkAndGoStageAdjust planning error";
     return result.SetStageStatus(StageStatusType::ERROR);
   }
   const bool is_ready_to_cruise =
       CheckADCReadyToCruise(injector_->vehicle_state(), frame,
                             GetContextAs<ParkAndGoContext>()->scenario_config);
 
   bool is_end_of_trajectory = false;
   const auto& history_frame = injector_->frame_history()->Latest();
   if (history_frame) {
     const auto& trajectory_points =
         history_frame->current_frame_planned_trajectory().trajectory_point();
     if (!trajectory_points.empty()) {
       is_end_of_trajectory =
           (trajectory_points.rbegin()->relative_time() < 0.0);
     }
   }
 
   if (!is_ready_to_cruise && !is_end_of_trajectory) {
     return result.SetStageStatus(StageStatusType::RUNNING);
   }
   return FinishStage();
 }
 
 StageResult ParkAndGoStageAdjust::FinishStage() {
   const auto vehicle_status = injector_->vehicle_state();
   ADEBUG << vehicle_status->steering_percentage();
   if (std::fabs(vehicle_status->steering_percentage()) <
       GetContextAs<ParkAndGoContext>()
           ->scenario_config.max_steering_percentage_when_cruise()) {
     next_stage_ = "PARK_AND_GO_CRUISE";
   } else {
     ResetInitPostion();
     next_stage_ = "PARK_AND_GO_PRE_CRUISE";
   }
   return StageResult(StageStatusType::FINISHED);
 }
 
 void ParkAndGoStageAdjust::ResetInitPostion() {
   auto* park_and_go_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_park_and_go();
   park_and_go_status->mutable_adc_init_position()->set_x(
       injector_->vehicle_state()->x());
   park_and_go_status->mutable_adc_init_position()->set_y(
       injector_->vehicle_state()->y());
   park_and_go_status->mutable_adc_init_position()->set_z(0.0);
   park_and_go_status->set_adc_init_heading(
       injector_->vehicle_state()->heading());
 }
 
 }  // namespace planning
 }  // namespace apollo