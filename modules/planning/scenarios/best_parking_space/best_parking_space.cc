#include "modules/planning/scenarios/best_parking_space/best_parking_space.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

bool BestParkingSpaceScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<BestParkingSpaceConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    init_ = true;
    return true;
}

bool BestParkingSpaceScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    std::string target_parking_spot_id;
    double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();

    auto parking_command = frame.local_view().planning_command->has_parking_command();
    auto parking_spot_id = frame.local_view().planning_command->parking_command().has_parking_spot_id();

    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;
    }

    if (parking_command && parking_spot_id) {
        target_parking_spot_id = frame.local_view().planning_command->parking_command().parking_spot_id();
    }

    const auto& vehicle_state_info = frame.vehicle_state();
    const auto& reference_line_info = frame.reference_line_info().front();

    if (!parking_command) {
        const auto routing_end = frame.local_view().end_lane_way_point;
        if (routing_end == nullptr) {
            return false;
        }

        common::PointENU end_position;
        end_position.set_x(routing_end->pose().x());
        end_position.set_y(routing_end->pose().y());

        common::SLPoint dest_sl;
        const auto& reference_line = reference_line_info.reference_line();
        reference_line.XYToSL(routing_end->pose(), &dest_sl);

        const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
        const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;

        if (adc_distance_to_dest > parking_spot_range_to_start) {
            return false;
        }

        if (target_parking_spot_id.empty()) {
            std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
            if (hdmap_->GetParkingSpaces(end_position, parking_spot_range_to_start, &parking_spaces) == 0
                && !parking_spaces.empty()) {
                const auto& obstacles = frame.obstacles();

                int right_side_parking_count = 0;
                double vehicle_x = vehicle_state_info.x();
                double vehicle_y = vehicle_state_info.y();
                double vehicle_heading = vehicle_state_info.heading();
                apollo::common::math::Vec2d vehicle_pos(vehicle_x, vehicle_y);
                apollo::common::math::Vec2d vehicle_heading_vec(cos(vehicle_heading), sin(vehicle_heading));

                for (const auto& parking_space : parking_spaces) {
                    const auto& points = parking_space->polygon().points();
                    if (points.size() != 4) {
                        continue;
                    }
                    apollo::common::math::Vec2d center = (points[0] + points[1] + points[2] + points[3]) / 4.0;
                    apollo::common::math::Vec2d vehicle_to_spot = center - vehicle_pos;
                    double cross_product = vehicle_heading_vec.CrossProd(vehicle_to_spot);
                    bool is_right_side = (cross_product <= 0);
                    if (is_right_side) {
                        right_side_parking_count++;
                    }
                }

                bool select_right_side_only = (right_side_parking_count == 5);
                
                if (select_right_side_only) {
                    AINFO << "右边停车位数量为5，仅选择右侧停车位";
                }
                
                std::vector<std::pair<std::string, double>> valid_spaces;

                for (const auto& parking_space : parking_spaces) {
                    const std::string& id = parking_space->id().id();
                    const auto& points = parking_space->polygon().points();

                    if (points.size() != 4) {
                        continue;
                    }

                    apollo::common::math::Vec2d center = (points[0] + points[1] + points[2] + points[3]) / 4.0;
                    apollo::common::math::Vec2d vehicle_to_spot = center - vehicle_pos;
                    double cross_product = vehicle_heading_vec.CrossProd(vehicle_to_spot);
                    bool is_right_side = (cross_product <= 0);
                    std::string position_str = is_right_side ? "右侧" : "左侧";
                    double distance_to_vehicle = center.DistanceTo(vehicle_pos);

                    if (select_right_side_only && !is_right_side) {
                        continue;
                    }

                    bool occupied = false;
                    for (const auto& obstacle : obstacles) {
                        if (!obstacle)
                            continue;
                        const auto& obs_center = obstacle->PerceptionBoundingBox().center();
                        apollo::common::math::Vec2d obs_vec(obs_center.x(), obs_center.y());

                        if (center.DistanceTo(obs_vec) <= 3.0) {
                            occupied = true;
                            break;
                        }
                    }

                    std::string status = occupied ? "占用" : "可用";
                    AINFO << "停车位ID: " << id << ", 距离: " << std::fixed << std::setprecision(2) 
                          << distance_to_vehicle << "m, 位置: " << position_str << ", 状态: " << status;

                    if (!occupied) {
                        valid_spaces.emplace_back(id, distance_to_vehicle);
                    }
                }

                if (!valid_spaces.empty()) {
                    std::sort(valid_spaces.begin(), valid_spaces.end(), [](const auto& a, const auto& b) {
                        return a.second < b.second;
                    });
                    target_parking_spot_id = valid_spaces.front().first;
                    AINFO << "选择目标停车位: " << target_parking_spot_id;
                }
            }
        }
    }
    // 检查车辆与目标停车位的距离
    hdmap::Id parking_id;
    parking_id.set_id(target_parking_spot_id);
    auto target_parking_spot_ptr = hdmap_->GetParkingSpaceById(parking_id);
    double distance_to_parking = 0.0;
    if (target_parking_spot_ptr) {
        const auto& points = target_parking_spot_ptr->polygon().points();
        apollo::common::math::Vec2d center = (points[0] + points[1] + points[2] + points[3]) / 4.0;
        double vehicle_x = vehicle_state_info.x();
        double vehicle_y = vehicle_state_info.y();
        apollo::common::math::Vec2d vehicle_pos(vehicle_x, vehicle_y);
        distance_to_parking = center.DistanceTo(vehicle_pos);
    }
    if (target_parking_spot_id.empty() || distance_to_parking >= 7.0) {
        return false;
    }

    context_.target_parking_spot_id = target_parking_spot_id;
    return true;
}

bool BestParkingSpaceScenario::SearchTargetParkingSpotOnPath(
        const Path& nearby_path,
        const std::string& target_parking_id,
        PathOverlap* parking_space_overlap) {
    const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
    for (const auto& parking_overlap : parking_space_overlaps) {
        if (parking_overlap.object_id == target_parking_id) {
            *parking_space_overlap = parking_overlap;
            return true;
        }
    }
    return false;
}

bool BestParkingSpaceScenario::CheckDistanceToParkingSpot(
        const Frame& frame,
        const VehicleState& vehicle_state,
        const Path& nearby_path,
        const double parking_start_range,
        const PathOverlap& parking_space_overlap) {
    // TODO(Jinyun) parking overlap s are wrong on map, not usable
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    hdmap::Id id;
    double center_point_s, center_point_l;
    id.set_id(parking_space_overlap.object_id);
    ParkingSpaceInfoConstPtr target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
    Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
    Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
    Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
    Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
    Vec2d center_point = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;
    nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
    double vehicle_point_s = 0.0;
    double vehicle_point_l = 0.0;
    Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
    nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
    if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
        return true;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo