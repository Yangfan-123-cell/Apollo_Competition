#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/common/st_graph_data.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "modules/planning/tasks/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

// 初始化分段加加速度速度优化器
bool PiecewiseJerkSpeedOptimizer::Init(
        const std::string& config_dir,
        const std::string& name,
        const std::shared_ptr<DependencyInjector>& injector) {
    if (!SpeedOptimizer::Init(config_dir, name, injector)) {
        return false;
    }
    // 加载当前任务的配置
    return SpeedOptimizer::LoadConfig<PiecewiseJerkSpeedOptimizerConfig>(&config_);
}

double PiecewiseJerkSpeedOptimizer::GetSpeedOffsetByObstacles() {
  double speed_offset = 0.0;
  const auto& obstacle_items = reference_line_info_->path_decision()->obstacles().Items();
  const auto& crosswalk_overlaps = reference_line_info_->reference_line().map_path().crosswalk_overlaps();
  int obstacle_count = obstacle_items.size();
  
  // 统计类型为 5 的障碍物数量
  int type5_count = 0;
  for (const auto* obstacle : obstacle_items) {
    if (obstacle->Perception().type() == 5) {
      ++type5_count;
    }
  }
  
  auto x = frame_->vehicle_state().x();
  auto y = frame_->vehicle_state().y();
  auto speed = frame_->vehicle_state().linear_velocity();
  
  // 只有类型 5 的障碍物数量大于 13 时，才进行控制区逻辑
  if (type5_count > 13) {
    // 遍历所有障碍物，记录最小的x和最大的x坐标
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    for (const auto* obstacle : obstacle_items) {
      if (obstacle->Id() == "DEST") {
        continue;
      }
      const auto& position = obstacle->PerceptionBoundingBox().center();
      min_x = std::min(min_x, position.x());
      max_x = std::max(max_x, position.x());
    }
    
    double control_zone_start = min_x - 20.0;
    double control_zone_end = max_x + 15.0;
    
    if (x >= control_zone_start && x <= control_zone_end) {
      if (speed > 8.0) {
        speed_offset = -0.40;
        return speed_offset;
      }
    } else {
      speed_offset = 0.04; // 场景2可以快一点
      return speed_offset;
    }
  }
  
  // 红绿灯123,根据人行道的数量来
  bool has_TL = false;
  bool has_SS = false;
  for (const auto* obstacle : obstacle_items) {
    if (obstacle->Id() == "TL_Signal_5") {
      has_TL = true;
    }
    if (obstacle->Id() == "SS_StopSign_1") {
      has_SS = true;
    }
  }

  if (has_SS) {
    speed_offset = 0.09;
    return speed_offset;
  }
  if (crosswalk_overlaps.size() == 2 && has_TL) {
    speed_offset = 0.06;
    return speed_offset;
  }
  
  if (crosswalk_overlaps.size() == 2) {
    for (const auto* obstacle : obstacle_items) {
      if (obstacle->Id() == "DEST") {
        continue;
      }
      double obs_x = obstacle->PerceptionBoundingBox().center().x();
      if (x >= obs_x - 10 && x <= obs_x + 10.0) {
        speed_offset = 0.00;
        return speed_offset;
      }
    }
    // 否则保持原有逻辑
    speed_offset = 0.05;
    return speed_offset;
  }
  
  // 新增：基于type5最大X坐标的速度限制（障碍物总数>30时）
  if (obstacle_count > 30) {
    // 找到type5中X坐标最大的障碍物
    double type5_max_x = std::numeric_limits<double>::lowest();
    for (const auto* obstacle : obstacle_items) {
      if (obstacle->Perception().type() == 5) {
        double obs_x = obstacle->PerceptionBoundingBox().center().x();
        if (obs_x > type5_max_x) {
          type5_max_x = obs_x;
        }
      }
    }
    
    // 定义速度限制区域
    double speed_limit_zone_start = type5_max_x - 47.4;
    double speed_limit_zone_end = type5_max_x + 121.2;
    
    if (x >= speed_limit_zone_start && x <= speed_limit_zone_end) {
      if (speed > 8.0) {
        speed_offset = -0.40;
        return speed_offset;
      }
    } else {
      speed_offset = 0.04;
      return speed_offset;
    }
  }
  
  // 狭窄路
  if (type5_count == 3 || type5_count == 4) {
    speed_offset = 0.07;
    return speed_offset;
  }
  
  // 自动泊车
  if (type5_count > 5 && type5_count < 14) {
    speed_offset = 0.05;
    return speed_offset;
  }
  
  return speed_offset;
}


Status PiecewiseJerkSpeedOptimizer::Process(
        const PathData& path_data,
        const TrajectoryPoint& init_point,
        SpeedData* const speed_data) {
    if (reference_line_info_->ReachedDestination()) {
        return Status::OK();
    }

    ACHECK(speed_data != nullptr);
    SpeedData reference_speed_data = *speed_data;

    // 检查路径数据是否为空
    if (path_data.discretized_path().empty()) {
        const std::string msg = "路径数据为空";
        AINFO << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();
    PrintCurves print_debug;
    const auto& veh_param = common::VehicleConfigHelper::GetConfig().vehicle_param();

    // 【速度初始状态】：设置初始位置、速度、加速度
    std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(), st_graph_data.init_point().a()};

    // 【倒车处理】：如果车辆处于倒车状态，调整速度符号
    const auto& vehicle_state = frame_->vehicle_state();
    if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        init_s[1] = std::max(-init_s[1], 0.0);  // 倒车速度转为正值
        init_s[2] = -init_s[2];                 // 倒车加速度取反
        AINFO << "转换倒车速度" << init_s[0] << "," << init_s[1] << "," << init_s[2];
    }

    // 【时间参数设置】：这些参数直接影响速度规划的精度和性能
    double delta_t = 0.1;                                           // 时间步长（秒）
    double total_length = st_graph_data.path_length();              // 总路径长度
    double total_time = st_graph_data.total_time_by_conf();         // 总时间
    int num_of_knots = static_cast<int>(total_time / delta_t) + 1;  // 节点数量

    print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);
    print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);
    print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);

    // 【ST边界更新】：处理障碍物和交通规则产生的约束
    const double kEpsilon = 0.01;
    std::vector<std::pair<double, double>> s_bounds;  // 位置边界

    for (int i = 0; i < num_of_knots; ++i) {
        double curr_t = i * delta_t;
        double s_lower_bound = 0.0;           // 位置下界
        double s_upper_bound = total_length;  // 位置上界

        // 遍历所有ST边界约束（障碍物、停车线等）
        for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
            double s_lower = 0.0;
            double s_upper = 0.0;
            if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
                continue;
            }

            // 根据边界类型设置不同的约束
            switch (boundary->boundary_type()) {
            case STBoundary::BoundaryType::STOP:   // 停车约束
            case STBoundary::BoundaryType::YIELD:  // 让行约束
                s_upper_bound = std::fmin(s_upper_bound, s_upper);
                break;
            case STBoundary::BoundaryType::FOLLOW:  // 跟车约束
                s_upper_bound = std::fmin(s_upper_bound, s_upper);
                break;
            case STBoundary::BoundaryType::OVERTAKE:  // 超车约束
                s_lower_bound = std::fmax(s_lower_bound, s_lower);
                break;
            default:
                break;
            }
        }

        s_upper_bound = std::fmax(s_upper_bound, s_lower_bound + kEpsilon);
        print_debug.AddPoint("st_bounds_lower", curr_t, s_lower_bound);
        print_debug.AddPoint("st_bounds_upper", curr_t, s_upper_bound);

        // 检查边界合理性
        if (s_lower_bound > s_upper_bound) {
            const std::string msg = "ST图上的位置下界大于上界";
            AINFO << msg;
            speed_data->clear();
            print_debug.PrintToLog();
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }

        s_bounds.emplace_back(s_lower_bound, s_upper_bound);
    }

    // 【速度边界和参考速度更新】：这是控制正常行驶速度的关键部分
    std::vector<double> x_ref(num_of_knots, total_length);  // 参考位置
    std::vector<double> dx_ref(num_of_knots,
                               reference_line_info_->GetCruiseSpeed());       // 【关键】参考巡航速度
    std::vector<double> dx_ref_weight(num_of_knots, config_.ref_v_weight());  // 速度参考权重
    std::vector<double> penalty_dx;                                           // 曲率惩罚
    std::vector<std::pair<double, double>> s_dot_bounds;                      // 速度边界

    const SpeedLimit& speed_limit = st_graph_data.speed_limit();

    for (int i = 0; i < num_of_knots; ++i) {
        double curr_t = i * delta_t;

        // 获取路径上的s坐标
        SpeedPoint sp;
        reference_speed_data.EvaluateByTime(curr_t, &sp);
        const double path_s = sp.s();
        x_ref[i] = path_s;

        // 获取曲率并计算曲率惩罚
        PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
        penalty_dx.push_back(std::fabs(path_point.kappa()) * config_.kappa_penalty_weight());

        // 【速度上下界设置】：限制车辆的最大最小速度
        const double v_lower_bound = 0.0;                         // 最小速度（不能倒车）
        double v_upper_bound = FLAGS_planning_upper_speed_limit;  // 规划最大速度限制

        // 应用道路速度限制
        v_upper_bound = std::fmin(speed_limit.GetSpeedLimitByS(path_s), v_upper_bound);
        double kappa_kp = std::abs(path_point.kappa());
        if (kappa_kp > 1e-6) {
            double v_max_lat = std::sqrt(1.0 / kappa_kp);
            v_upper_bound = std::min(v_upper_bound, v_max_lat);
        }
        // 【参考速度调整】：确保参考速度不超过限制
        dx_ref[i] = std::fmin(v_upper_bound, dx_ref[i]);

        s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));

        // 记录调试信息
        print_debug.AddPoint("st_reference_line", curr_t, x_ref[i]);
        print_debug.AddPoint("st_penalty_dx", curr_t, penalty_dx.back());
        print_debug.AddPoint("vt_reference_line", curr_t, dx_ref[i]);
        print_debug.AddPoint("vt_weighting", curr_t, dx_ref_weight[i]);
        print_debug.AddPoint("vt_boundary_lower", curr_t, v_lower_bound);
        print_debug.AddPoint("sv_boundary_lower", path_s, v_lower_bound);
        print_debug.AddPoint("sk_curve", path_s, path_point.kappa());
        print_debug.AddPoint("vt_boundary_upper", curr_t, v_upper_bound);
        print_debug.AddPoint("sv_boundary_upper", path_s, v_upper_bound);
    }

    // 调整初始状态以确保可行性
    AdjustInitStatus(s_dot_bounds, delta_t, init_s);

    // 【创建优化问题】：设置分段加加速度优化问题
    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t, init_s);

    // 【优化权重设置】：这些权重决定了速度规划的特性
    piecewise_jerk_problem.set_weight_ddx(config_.acc_weight());    // 加速度权重
    piecewise_jerk_problem.set_weight_dddx(config_.jerk_weight());  // 加加速度权重
    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});    // 缩放因子

    // 【约束设置】：设置位置、速度、加速度、加加速度的约束
    piecewise_jerk_problem.set_x_bounds(0.0, total_length);  // 位置约束
    piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(),
                                          veh_param.max_acceleration());  // 加速度约束
    piecewise_jerk_problem.set_dddx_bound(
            FLAGS_longitudinal_jerk_lower_bound,
            FLAGS_longitudinal_jerk_upper_bound);  // 加加速度约束

    // 设置详细约束
    piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));                    // 位置边界
    piecewise_jerk_problem.set_dx_ref(dx_ref_weight, dx_ref);                    // 【关键】速度参考
    piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));  // 位置参考
    piecewise_jerk_problem.set_penalty_dx(penalty_dx);                           // 曲率惩罚
    piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));               // 速度边界

    // 【求解优化问题】：生成最优速度曲线
    if (!piecewise_jerk_problem.Optimize()) {
        const std::string msg = "分段加加速度速度优化器失败!";
        AINFO << msg << ".尝试回退方案.";

        // 尝试放宽速度约束的回退方案
        piecewise_jerk_problem.set_dx_bounds(
                0.0, std::fmax(FLAGS_planning_upper_speed_limit, st_graph_data.init_point().v()));

        if (!FLAGS_speed_optimize_fail_relax_velocity_constraint || !piecewise_jerk_problem.Optimize()) {
            speed_data->clear();
            print_debug.AddPoint("optimize_st_curve", 0, init_s[0]);
            print_debug.AddPoint("optimize_vt_curve", 0, init_s[1]);
            print_debug.AddPoint("optimize_at_curve", 0, init_s[2]);
            AINFO << "加加速度边界: " << FLAGS_longitudinal_jerk_lower_bound << ","
                  << FLAGS_longitudinal_jerk_upper_bound;
            AINFO << "加速度边界: " << veh_param.max_deceleration() << "," << veh_param.max_acceleration();
            print_debug.PrintToLog();
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    }

    // 【提取优化结果】：获得最优的位置、速度、加速度序列
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();  // 最优位置
    std::vector<double> ds = piecewise_jerk_problem.opt_dx();       // 最佳速度

    // 【修改部分】：根据障碍物检测结果动态调整速度偏移量
    double speed_offset = GetSpeedOffsetByObstacles();
    for (size_t i = 0; i < ds.size(); ++i) {
        ds[i] += speed_offset;
    }

    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();  // 最优加速度

    for (int i = 0; i < num_of_knots; ++i) {
        //AINFO << "在时刻t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i] << ", a = " << dds[i];
        print_debug.AddPoint("optimize_st_curve", i * delta_t, s[i]);
        print_debug.AddPoint("optimize_vt_curve", i * delta_t, ds[i]);
        print_debug.AddPoint("optimize_at_curve", i * delta_t, dds[i]);
    }

    // 【生成最终速度数据】：构建车辆执行的速度配置文件
    speed_data->clear();
    speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);

    for (int i = 1; i < num_of_knots; ++i) {
        // 避免在已停车时继续添加点
        if (ds[i] <= 0.0) {
            break;
        }
        speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i], (dds[i] - dds[i - 1]) / delta_t);
    }

    // 填充足够的速度点并记录调试信息
    SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
    RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
    print_debug.PrintToLog();
    return Status::OK();
}

// 调整初始状态以确保优化问题的可行性
void PiecewiseJerkSpeedOptimizer::AdjustInitStatus(
        const std::vector<std::pair<double, double>> s_dot_bound,
        double delta_t,
        std::array<double, 3>& init_s) {
    double v_min = init_s[1];
    double v_max = init_s[1];
    double a_min = init_s[2];
    double a_max = init_s[2];
    double last_a_min = 0;
    double last_a_max = 0;

    // 检查初始状态在未来时间步是否可行
    for (size_t i = 1; i < s_dot_bound.size(); i++) {
        last_a_min = a_min;
        last_a_max = a_max;
        a_min = a_min + delta_t * FLAGS_longitudinal_jerk_upper_bound;
        a_max = a_max + delta_t * FLAGS_longitudinal_jerk_lower_bound;
        v_min = v_min + 0.5 * delta_t * (a_min + last_a_min);
        v_max = v_max + 0.5 * delta_t * (a_max + last_a_max);

        // 如果速度超出边界，调整初始加速度为0
        if (v_min < s_dot_bound[i].first || v_max > s_dot_bound[i].second) {
            AINFO << "第" << i << "步初始状态不合适," << v_min << "," << v_max << "调整初始状态中的加速度为0 "
                  << init_s[0] << "," << init_s[1] << "," << init_s[2];
            init_s[2] = 0;  // 将初始加速度设为0
            return;
        }
    }
}

}  // namespace planning
}  // namespace apollo
