// column_generation.cpp - 列生成方法选择与调度
//
// 本文件实现子问题求解方法的选择和调度
// 支持三种子问题求解方法:
// - kCplexIP (0): 使用CPLEX求解整数背包问题 (默认方法)
// - kArcFlow (1): 使用Arc Flow网络流模型 (支持Arc分支)
// - kDP (2): 使用动态规划 (适用于小规模问题)
//
// 方法选择策略:
// - 根节点: 可使用任意方法, 推荐CPLEX或Arc Flow
// - 分支节点: 若使用Arc分支策略, 必须使用Arc Flow方法
// - DP方法不支持Arc约束, 仅适用于无约束或约束可忽略的情况

#include "2DBP.h"

using namespace std;

// 根节点SP1子问题方法选择
// 功能: 根据sp1_method_设置选择对应的求解函数
// 参数: node.sp1_method_ 指定求解方法
// 返回值: true=列生成收敛, false=找到改进列
bool SolveRootSP1(ProblemParams& params, ProblemData& data, BPNode& node) {
    int method = node.sp1_method_;

    switch (method) {
        case kArcFlow:
            // Arc Flow: 支持Arc分支约束
            return SolveRootSP1ArcFlow(params, data, node);
        case kDP:
            // 动态规划: 快速但不支持Arc约束
            return SolveRootSP1DP(params, data, node);
        case kCplexIP:
        default:
            // CPLEX整数规划: 通用方法
            return SolveRootSP1Knapsack(params, data, node);
    }
}

// 根节点SP2子问题方法选择
// 功能: 根据sp2_method_设置选择对应的求解函数
// 参数:
//   - node.sp2_method_: 求解方法
//   - strip_type_id: 当前条带类型索引
// 返回值: true=该条带类型收敛, false=找到改进列
bool SolveRootSP2(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id) {

    int method = node.sp2_method_;

    switch (method) {
        case kArcFlow:
            return SolveRootSP2ArcFlow(params, data, node, strip_type_id);
        case kDP:
            return SolveRootSP2DP(params, data, node, strip_type_id);
        case kCplexIP:
        default:
            return SolveRootSP2Knapsack(params, data, node, strip_type_id);
    }
}

// 非根节点SP1子问题方法选择
// 功能: 与根节点类似, 但使用指针访问节点
// 注意: 若有Arc分支约束, 应使用Arc Flow方法
bool SolveNodeSP1(ProblemParams& params, ProblemData& data, BPNode* node) {
    int method = node->sp1_method_;

    switch (method) {
        case kArcFlow:
            // Arc Flow: 在函数内部应用sp1_*_arcs_约束
            return SolveNodeSP1ArcFlow(params, data, node);
        case kDP:
            // DP不支持Arc约束, 可能导致分支无效
            return SolveNodeSP1DP(params, data, node);
        case kCplexIP:
        default:
            // CPLEX背包不包含Arc约束
            return SolveNodeSP1Knapsack(params, data, node);
    }
}

// 非根节点SP2子问题方法选择
// 功能: 与根节点类似, 但使用指针访问节点
// 注意: Arc约束按条带类型索引存储在sp2_*_arcs_中
bool SolveNodeSP2(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id) {

    int method = node->sp2_method_;

    switch (method) {
        case kArcFlow:
            // Arc Flow: 在函数内部应用sp2_*_arcs_[strip_type_id]约束
            return SolveNodeSP2ArcFlow(params, data, node, strip_type_id);
        case kDP:
            return SolveNodeSP2DP(params, data, node, strip_type_id);
        case kCplexIP:
        default:
            return SolveNodeSP2Knapsack(params, data, node, strip_type_id);
    }
}
