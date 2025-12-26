// =============================================================================
// main.cpp - 程序入口
// =============================================================================
// 功能: 二维下料问题分支定价求解器的主函数
// =============================================================================

#include "2DBP.h"

#include <chrono>
#include <filesystem>

using namespace std;


// 主函数
int main() {
    // 初始化日志系统
    string log_file = "log_2DBP_" + GetTimestampString();
    Logger logger(log_file);

    cout << "[调试] 日志系统初始化成功\n";
    cout << "[调试] 日志文件: " << logger.GetLogFilePath() << "\n";

    // 记录程序开始时间
    auto start_time = chrono::high_resolution_clock::now();

    // 输出程序标题
    cout << "\n";
    cout << "============================================================\n";
    cout << "  二维下料问题分支定价求解器 (CS-2D-BP)\n";
    cout << "  2D Cutting Stock Problem - Branch and Price Solver\n";
    cout << "============================================================\n";
    cout << "\n";

    // 初始化全局数据容器
    ProblemData data;
    ProblemParams params;

    // 初始化根节点
    BPNode root_node;
    root_node.id_ = 1;
    params.branch_state_ = 0;

    // 阶段 1: 数据读取与预处理
    cout << "[阶段1] 数据读取与预处理\n";
    cout << "------------------------------------------------------------\n";
    LoadInput(params, data);
    cout << "\n";

    // 阶段 2: 原始启发式生成初始解
    cout << "[阶段2] 原始启发式生成初始解\n";
    cout << "------------------------------------------------------------\n";
    RunHeuristic(params, data, root_node);
    cout << "\n";

    // 阶段 3: 根节点列生成求解
    cout << "[阶段3] 根节点列生成求解\n";
    cout << "------------------------------------------------------------\n";
    SolveRootCG(params, data, root_node);
    cout << "\n";

    // 阶段 4: 检查整数性并决定是否分支
    cout << "[阶段4] 整数性检查\n";
    cout << "------------------------------------------------------------\n";
    params.need_search_ = ProcessNode(params, data, root_node);

    // 将根节点加入节点列表
    data.nodes_.push_back(root_node);
    params.is_at_root_ = 1;
    cout << "\n";

    // 阶段 5: 分支定界 (若需要)
    if (params.need_search_ == 0) {
        cout << "[阶段5] 分支定界求解\n";
        cout << "------------------------------------------------------------\n";
        params.branch_state_ = 1;
        RunBranchAndPrice(params, data);
    } else {
        cout << "[阶段5] 无需分支定界 (根节点已获得整数解)\n";
    }
    cout << "\n";

    // 输出求解结果
    cout << "============================================================\n";
    cout << "  求解结果 (Solution Summary)\n";
    cout << "============================================================\n";

    // 计算总耗时
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    double elapsed_sec = duration.count() / 1000.0;

    // 输出关键指标
    cout << fixed << setprecision(2);
    cout << "  最优目标值 (母板数): " << params.best_obj_ << "\n";
    cout << "  总耗时: " << elapsed_sec << " 秒\n";
    cout << "  分支节点数: " << params.num_nodes_ << "\n";
    cout.unsetf(ios::fixed);

    cout << "============================================================\n";
    cout << "\n";
    cout << "[完成] 程序执行结束\n";

    return 0;
}
