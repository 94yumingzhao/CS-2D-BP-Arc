// main.cpp - 程序入口
//
// 项目: CS-2D-BP-Arc
// 描述: 二维下料问题分支定价求解器, 支持Arc Flow和DP子问题求解
//
// 算法流程:
// 1. 数据读取: 从文件加载问题实例 (母板尺寸, 子板类型, 需求量)
// 2. 启发式: 生成对角矩阵初始解, 确保列生成可行启动
// 3. 根节点列生成: 求解LP松弛, 生成最优列池
// 4. 整数性检查: 若LP解为整数则直接输出, 否则进入分支定价
// 5. 分支定价: 使用Arc分支策略逐步求解整数最优解
//
// 子问题求解方法 (可配置):
// - kCplexIP: CPLEX整数规划 (默认)
// - kArcFlow: Arc Flow网络流模型 (支持Arc分支)
// - kDP: 动态规划 (快速, 但不支持Arc约束)

#include "2DBP.h"

#include <chrono>

using namespace std;

int main() {
    // 创建输出目录 (日志和LP文件)
    filesystem::create_directories("logs");
    filesystem::create_directories("lp");

    // 初始化日志系统
    // 日志文件命名: logs/log_2DBP_Arc_YYYYMMDD_HHMMSS.log
    string log_file = "logs/log_2DBP_Arc_" + GetTimestampString();
    Logger logger(log_file);

    LOG("[系统] 日志初始化完成");
    LOG_FMT("[系统] 日志文件: %s\n", logger.GetLogFilePath().c_str());

    // 记录开始时间 (用于计算总耗时)
    auto start_time = chrono::high_resolution_clock::now();

    // 程序标题
    LOG("============================================================");
    LOG("  二维下料问题分支定价求解器 (CS-2D-BP-Arc)");
    LOG("  2D Cutting Stock Problem - Branch and Price with Arc Flow");
    LOG("============================================================");

    // 初始化数据结构
    ProblemData data;    // 问题数据 (子板类型, 条带类型, 索引映射)
    ProblemParams params; // 问题参数 (尺寸, 方法设置, 全局最优解)

    // 配置子问题求解方法
    // SP1: 宽度方向背包问题 (在母板上选择条带)
    // SP2: 长度方向背包问题 (在条带上选择子板)
    params.sp1_method_ = kCplexIP;
    params.sp2_method_ = kCplexIP;

    // 初始化根节点
    BPNode root_node;
    root_node.id_ = 1;  // 根节点ID = 1

    // 阶段1: 数据读取
    LOG("------------------------------------------------------------");
    LOG("[阶段1] 数据读取与预处理");
    LOG("------------------------------------------------------------");

    auto [status, num_items, num_strips] = LoadInput(params, data);
    if (status != 0) {
        LOG("[错误] 数据读取失败");
        return 1;
    }

    // 如果使用Arc Flow方法, 预先生成Arc网络
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
        // LP解恰好为整数, 无需分支
        LOG("[结果] 根节点解为整数解, 无需分支");
        params.global_best_int_ = root_node.solution_.obj_val_;
        params.global_best_y_cols_ = root_node.solution_.y_columns_;
        params.global_best_x_cols_ = root_node.solution_.x_columns_;
    } else {
        // LP解为分数, 需要分支定价求整数解
        LOG("[结果] 根节点解非整数, 需要分支定价");

        // 阶段5: 分支定价
        LOG("------------------------------------------------------------");
        LOG("[阶段5] 分支定价求解");
        LOG("------------------------------------------------------------");

        RunBranchAndPrice(params, data, &root_node);
    }

    // 计算总耗时
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    double elapsed_sec = duration.count() / 1000.0;

    // 输出求解结果汇总
    LOG("============================================================");
    LOG("  求解结果 (Solution Summary)");
    LOG("============================================================");
    LOG_FMT("  最优目标值 (母板数): %.4f\n", params.global_best_int_);
    LOG_FMT("  根节点下界: %.4f\n", root_node.lower_bound_);
    LOG_FMT("  最优性间隙: %.2f%%\n", params.gap_ * 100);
    LOG_FMT("  分支节点数: %d\n", params.node_counter_);
    LOG_FMT("  总耗时: %.3f 秒\n", elapsed_sec);
    LOG("============================================================");

    // 输出最优切割方案
    if (params.global_best_int_ < INFINITY) {
        // 输出Y列 (母板切割方案)
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

        // 输出X列 (条带切割方案)
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
