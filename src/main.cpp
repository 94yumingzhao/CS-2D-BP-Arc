// =============================================================================
// main.cpp - 程序入口
// 项目: CS-2D-BP-Arc
// 描述: 二维下料问题分支定价求解器, 支持Arc Flow和DP子问题求解
// =============================================================================

#include "2DBP.h"

#include <chrono>

using namespace std;

int main() {
    // 创建输出目录
    filesystem::create_directories("logs");
    filesystem::create_directories("lp");

    // 初始化日志系统 (日志文件放在 logs/ 目录)
    string log_file = "logs/log_2DBP_Arc_" + GetTimestampString();
    Logger logger(log_file);

    LOG("[系统] 日志初始化完成");
    LOG_FMT("[系统] 日志文件: %s\n", logger.GetLogFilePath().c_str());

    // 记录开始时间
    auto start_time = chrono::high_resolution_clock::now();

    // 程序标题
    LOG("============================================================");
    LOG("  二维下料问题分支定价求解器 (CS-2D-BP-Arc)");
    LOG("  2D Cutting Stock Problem - Branch and Price with Arc Flow");
    LOG("============================================================");

    // 初始化数据结构
    ProblemData data;
    ProblemParams params;

    // 设置子问题求解方法 (可选: kCplexIP, kArcFlow, kDP)
    params.sp1_method_ = kCplexIP;  // SP1: 宽度背包
    params.sp2_method_ = kCplexIP;  // SP2: 长度背包

    // 初始化根节点
    BPNode root_node;
    root_node.id_ = 1;

    // 阶段1: 数据读取
    LOG("------------------------------------------------------------");
    LOG("[阶段1] 数据读取与预处理");
    LOG("------------------------------------------------------------");

    auto [status, num_items, num_strips] = LoadInput(params, data);
    if (status != 0) {
        LOG("[错误] 数据读取失败");
        return 1;
    }

    // 如果使用Arc Flow方法, 生成网络
    if (params.sp1_method_ == kArcFlow || params.sp2_method_ == kArcFlow) {
        GenerateAllArcs(data, params);
    }

    // 阶段2: 启发式生成初始解
    LOG("------------------------------------------------------------");
    LOG("[阶段2] 启发式生成初始解");
    LOG("------------------------------------------------------------");

    RunHeuristic(params, data, root_node);

    // 阶段3: 根节点列生成
    LOG("------------------------------------------------------------");
    LOG("[阶段3] 根节点列生成");
    LOG("------------------------------------------------------------");

    SolveRootCG(params, data, root_node);

    // 阶段4: 检查整数性
    LOG("------------------------------------------------------------");
    LOG("[阶段4] 整数性检查");
    LOG("------------------------------------------------------------");

    bool is_integer = IsIntegerSolution(root_node.solution_);

    if (is_integer) {
        LOG("[结果] 根节点解为整数解, 无需分支");
        params.global_best_int_ = root_node.solution_.obj_val_;
        params.global_best_y_cols_ = root_node.solution_.y_columns_;
        params.global_best_x_cols_ = root_node.solution_.x_columns_;
    } else {
        LOG("[结果] 根节点解非整数, 需要分支定价");

        // 阶段5: 分支定价
        LOG("------------------------------------------------------------");
        LOG("[阶段5] 分支定价求解");
        LOG("------------------------------------------------------------");

        RunBranchAndPrice(params, data, &root_node);
    }

    // 计算耗时
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    double elapsed_sec = duration.count() / 1000.0;

    // 输出结果
    LOG("============================================================");
    LOG("  求解结果 (Solution Summary)");
    LOG("============================================================");
    LOG_FMT("  最优目标值 (母板数): %.4f\n", params.global_best_int_);
    LOG_FMT("  根节点下界: %.4f\n", root_node.lower_bound_);
    LOG_FMT("  最优性间隙: %.2f%%\n", params.gap_ * 100);
    LOG_FMT("  分支节点数: %d\n", params.node_counter_);
    LOG_FMT("  总耗时: %.3f 秒\n", elapsed_sec);
    LOG("============================================================");

    // 输出切割方案
    if (params.global_best_int_ < INFINITY) {
        LOG("[最优解] Y列 (母板切割方案):");
        for (int i = 0; i < (int)params.global_best_y_cols_.size(); i++) {
            if (params.global_best_y_cols_[i].value_ > kZeroTolerance) {
                ostringstream oss;
                oss << "  Y" << (i + 1) << " = " << fixed << setprecision(0)
                    << params.global_best_y_cols_[i].value_ << " [";
                for (int j = 0; j < (int)params.global_best_y_cols_[i].pattern_.size(); j++) {
                    if (j > 0) oss << ", ";
                    oss << params.global_best_y_cols_[i].pattern_[j];
                }
                oss << "]";
                LOG(oss.str().c_str());
            }
        }

        LOG("[最优解] X列 (条带切割方案):");
        for (int i = 0; i < (int)params.global_best_x_cols_.size(); i++) {
            if (params.global_best_x_cols_[i].value_ > kZeroTolerance) {
                ostringstream oss;
                oss << "  X" << (i + 1) << " (条带" << params.global_best_x_cols_[i].strip_type_id_ + 1
                    << ") = " << fixed << setprecision(0)
                    << params.global_best_x_cols_[i].value_ << " [";
                for (int j = 0; j < (int)params.global_best_x_cols_[i].pattern_.size(); j++) {
                    if (j > 0) oss << ", ";
                    oss << params.global_best_x_cols_[i].pattern_[j];
                }
                oss << "]";
                LOG(oss.str().c_str());
            }
        }
    }

    LOG("[完成] 程序执行结束");

    return 0;
}
