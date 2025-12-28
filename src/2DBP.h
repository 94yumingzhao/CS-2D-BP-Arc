// =============================================================================
// 2DBP.h - 二维下料问题分支定价求解器 主头文件
// 项目: CS-2D-BP-Arc
// 描述: 采用两阶段切割的二维下料问题分支定价算法, 子问题支持Arc Flow/DP求解
// =============================================================================

#ifndef CS_2D_BP_ARC_H_
#define CS_2D_BP_ARC_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <vector>

#include <ilcplex/ilocplex.h>

#include "logger.h"

using namespace std;

// 全局常量
constexpr double kRcTolerance = 1.0e-6;     // 检验数容差
constexpr double kZeroTolerance = 1.0e-10;  // 零值容差
constexpr int kMaxCgIter = 100;             // 列生成最大迭代次数
const string kFilePath = "data/test.txt";   // 默认数据文件路径
const string kLogDir = "logs/";             // 日志输出目录
const string kLpDir = "lp/";                // LP文件输出目录
constexpr bool kExportLp = false;           // 是否导出LP文件 (调试用)

// 子问题求解方法枚举
enum SPMethod {
    kCplexIP = 0,   // CPLEX整数规划
    kArcFlow = 1,   // Arc Flow模型
    kDP = 2         // 动态规划
};

// 分支类型枚举
enum BranchType {
    kBranchNone = 0,    // 无需分支 (整数解)
    kBranchSP1Arc = 1,  // SP1 Arc 分支 (宽度方向)
    kBranchSP2Arc = 2   // SP2 Arc 分支 (长度方向)
};

// 子件类型: 存储同一规格子件的类型信息
struct ItemType {
    int type_id_ = -1;      // 类型编号 (0, 1, 2, ...)
    int length_ = -1;       // 长度 (X轴)
    int width_ = -1;        // 宽度 (Y轴)
    int demand_ = -1;       // 需求量
};

// 条带类型: 存储同一宽度条带的类型信息
// 条带宽度由其包含的子件宽度决定, 长度等于母板长度
struct StripType {
    int type_id_ = -1;      // 类型编号 (0, 1, 2, ...)
    int width_ = -1;        // 宽度 (Y轴, 等于对应子件宽度)
    int length_ = -1;       // 长度 (X轴, 等于母板长度)
};

// SP2 Arc Flow网络数据: 用于条带上的子件排列 (长度方向背包)
// 节点表示条带上的位置 (0到stock_length), Arc表示放置一个子件
struct SP2ArcFlowData {
    int strip_type_id_ = -1;                // 对应的条带类型
    vector<int> begin_nodes_;               // 起点节点 (位置0)
    vector<int> end_nodes_;                 // 终点节点 (位置stock_length)
    vector<int> mid_nodes_;                 // 中间节点
    vector<array<int, 2>> arc_list_;        // Arc列表 [起点, 终点]
    map<array<int, 2>, int> arc_to_index_;  // Arc到索引的映射

    // Arc分类索引
    vector<int> begin_arc_indices_;         // 从起点出发的Arc
    vector<int> end_arc_indices_;           // 到达终点的Arc
    vector<vector<int>> mid_in_arcs_;       // 中间节点入弧
    vector<vector<int>> mid_out_arcs_;      // 中间节点出弧
};

// SP1 Arc Flow网络数据: 用于母板上的条带排列 (宽度方向背包)
// 节点表示母板宽度方向的位置 (0到stock_width), Arc表示放置一种条带
struct SP1ArcFlowData {
    vector<int> begin_nodes_;               // 起点节点 (位置0)
    vector<int> end_nodes_;                 // 终点节点 (位置stock_width)
    vector<int> mid_nodes_;                 // 中间节点
    vector<array<int, 2>> arc_list_;        // Arc列表 [起点, 终点]
    map<array<int, 2>, int> arc_to_index_;  // Arc到索引的映射

    // Arc分类索引
    vector<int> begin_arc_indices_;         // 从起点出发的Arc
    vector<int> end_arc_indices_;           // 到达终点的Arc
    vector<vector<int>> mid_in_arcs_;       // 中间节点入弧
    vector<vector<int>> mid_out_arcs_;      // 中间节点出弧
};

// 新列: 列生成过程中子问题产生的新切割方案
struct NewColumn {
    vector<int> pattern_;               // 切割方案系数
    set<array<int, 2>> arc_set_;        // 对应的Arc集合
};

// Y列 (第一阶段): 母板切割为条带的方案
struct YColumn {
    vector<int> pattern_;               // pattern_[j] = 条带类型j的数量
    set<array<int, 2>> arc_set_;        // 对应的Arc集合 (宽度方向)
    double value_ = 0.0;                // LP解值
};

// X列 (第二阶段): 条带切割为子件的方案
struct XColumn {
    int strip_type_id_ = -1;            // 所属条带类型
    vector<int> pattern_;               // pattern_[i] = 子件类型i的数量
    set<array<int, 2>> arc_set_;        // 对应的Arc集合 (长度方向)
    double value_ = 0.0;                // LP解值
};

// 节点解: 存储分支定价节点的LP求解结果
struct NodeSolution {
    vector<YColumn> y_columns_;         // Y列集合
    vector<XColumn> x_columns_;         // X列集合
    double obj_val_ = -1;               // 目标函数值 (母板使用量)
};

// 分支定价节点: 分支定价树中的节点
struct BPNode {
    // 子问题求解方法
    int sp1_method_ = 0;        // SP1求解方法: 0=CPLEX, 1=ArcFlow, 2=DP
    int sp2_method_ = 0;        // SP2求解方法: 0=CPLEX, 1=ArcFlow, 2=DP

    // 节点标识
    int id_ = -1;               // 节点编号
    int parent_id_ = -1;        // 父节点编号 (-1表示根节点)
    double lower_bound_ = -1;   // 节点下界 (LP松弛解)

    // 分支状态
    int branch_dir_ = -1;       // 分支方向: 1=左, 2=右
    int prune_flag_ = 0;        // 剪枝标志: 0=未剪枝, 1=已剪枝
    int branched_flag_ = 0;     // 分支完成标志: 0=未分支, 1=已分支

    // 分支变量信息
    int branch_var_id_ = -1;            // 待分支变量索引
    double branch_var_val_ = -1;        // 待分支变量解值 (分数值)
    double branch_floor_ = -1;          // 向下取整值
    double branch_ceil_ = -1;           // 向上取整值

    // 分支历史 (累积的分支约束) - 变量分支 (已废弃, 保留兼容)
    vector<int> branched_var_ids_;      // 已分支变量索引
    vector<double> branched_bounds_;    // 已分支变量整数边界

    // Arc 分支类型和待分支 Arc 信息
    int branch_type_ = kBranchNone;             // 分支类型
    array<int, 2> branch_arc_ = {-1, -1};       // 待分支 Arc [起点, 终点]
    double branch_arc_flow_ = -1;               // Arc 流量值
    int branch_arc_strip_type_ = -1;            // SP2 分支时的条带类型

    // SP1 Arc 约束 (宽度方向, 从父节点累积继承)
    set<array<int, 2>> sp1_zero_arcs_;          // Arc = 0 (禁用)
    vector<array<int, 2>> sp1_lower_arcs_;      // Arc <= N
    vector<int> sp1_lower_bounds_;
    vector<array<int, 2>> sp1_greater_arcs_;    // Arc >= N
    vector<int> sp1_greater_bounds_;

    // SP2 Arc 约束 (长度方向, 按条带类型存储, 从父节点累积继承)
    // sp2_zero_arcs_[j] = 条带类型 j 的禁用 Arc 集合
    map<int, set<array<int, 2>>> sp2_zero_arcs_;
    map<int, vector<array<int, 2>>> sp2_lower_arcs_;
    map<int, vector<int>> sp2_lower_bounds_;
    map<int, vector<array<int, 2>>> sp2_greater_arcs_;
    map<int, vector<int>> sp2_greater_bounds_;

    // 主问题系数矩阵
    vector<vector<double>> matrix_;             // 完整系数矩阵
    vector<YColumn> y_columns_;                 // Y列集合
    vector<XColumn> x_columns_;                 // X列集合
    vector<set<array<int, 2>>> y_arc_sets_;     // Y列对应的Arc集合
    vector<set<array<int, 2>>> x_arc_sets_;     // X列对应的Arc集合

    // 列生成迭代信息
    int iter_ = -1;                     // 当前迭代次数
    vector<double> duals_;              // 对偶价格
    NewColumn new_y_col_;               // 新Y列
    NewColumn new_x_col_;               // 新X列
    int new_strip_type_ = -1;           // 新X列对应的条带类型

    // SP2临时数据
    double sp2_obj_ = -1;               // SP2目标值
    vector<double> sp2_solution_;       // SP2解

    // 节点解
    NodeSolution solution_;             // 求解结果

    // 链表指针: 用于节点队列管理
    BPNode* next_ = nullptr;
};

// 问题参数: 存储算法运行过程中的全局参数
struct ProblemParams {
    // 问题规模
    int num_item_types_ = -1;           // 子件类型数量 (N)
    int num_strip_types_ = -1;          // 条带类型数量 (J)
    int num_items_ = -1;                // 子件总数

    // 母板尺寸
    int stock_length_ = -1;             // 长度 (L, X轴)
    int stock_width_ = -1;              // 宽度 (W, Y轴)

    // 子问题方法设置
    int sp1_method_ = kCplexIP;         // SP1默认方法
    int sp2_method_ = kCplexIP;         // SP2默认方法

    // 分支定价树
    int node_counter_ = 1;              // 节点编号计数器
    double optimal_lb_ = INFINITY;      // 当前最优下界

    // 全局最优整数解信息
    double global_best_int_ = INFINITY;         // 最优整数解目标值
    vector<YColumn> global_best_y_cols_;        // 最优解Y列
    vector<XColumn> global_best_x_cols_;        // 最优解X列
    double gap_ = INFINITY;                     // 最优性间隙

    // 初始矩阵 (启发式生成)
    vector<vector<int>> init_y_matrix_;         // 初始Y列矩阵
    vector<vector<int>> init_x_matrix_;         // 初始X列矩阵
};

// 问题数据: 存储问题的输入数据和Arc Flow模型数据
struct ProblemData {
    // 基本数据
    vector<ItemType> item_types_;               // 子件类型列表
    vector<StripType> strip_types_;             // 条带类型列表
    vector<int> item_lengths_;                  // 子件长度列表 (降序)
    vector<int> strip_widths_;                  // 条带宽度列表 (降序)

    // 索引映射
    map<int, int> length_to_item_index_;        // 长度到子件类型索引
    map<int, int> width_to_strip_index_;        // 宽度到条带类型索引
    map<int, vector<int>> width_to_item_indices_;  // 宽度到该宽度的子件类型列表

    // SP1 Arc Flow网络 (宽度方向)
    SP1ArcFlowData sp1_arc_data_;

    // SP2 Arc Flow网络 (长度方向, 每种条带类型一个)
    vector<SP2ArcFlowData> sp2_arc_data_;
};

// Arc Flow函数 (arc_flow.cpp)
void GenerateSP1Arcs(ProblemData& data, ProblemParams& params);
void GenerateSP2Arcs(ProblemData& data, ProblemParams& params, int strip_type_id);
void GenerateAllArcs(ProblemData& data, ProblemParams& params);
void ConvertPatternToArcSet(vector<int>& pattern, vector<int>& sizes,
    set<array<int, 2>>& arc_set);
void GenerateYArcSetMatrix(BPNode& node, vector<int>& strip_widths);
void GenerateXArcSetMatrix(BPNode& node, vector<int>& item_lengths, int strip_type);

// Arc Flow 解转换函数 (arc_flow.cpp)
void ConvertYColsToSP1ArcFlow(vector<YColumn>& y_columns, ProblemData& data,
    map<int, tuple<int, int, double>>& arc_flow_solution);
void ConvertXColsToSP2ArcFlow(vector<XColumn>& x_columns, int strip_type_id,
    ProblemData& data, map<int, tuple<int, int, double>>& arc_flow_solution);
bool FindBranchArcSP1(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow);
bool FindBranchArcSP2(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow);
void PrintSP1ArcFlowSolution(map<int, tuple<int, int, double>>& solution);
void PrintSP2ArcFlowSolution(map<int, tuple<int, int, double>>& solution, int strip_type);

// 输入输出函数 (input.cpp)
void SplitString(const string& s, vector<string>& v, const string& c);
tuple<int, int, int> LoadInput(ProblemParams& params, ProblemData& data);
void BuildLengthIndex(ProblemData& data);
void BuildWidthIndex(ProblemData& data);

// 打印函数 (input.cpp)
void PrintParams(ProblemParams& params);
void PrintDemand(ProblemData& data);
void PrintInitMatrix(ProblemParams& params);
void PrintCGSolution(BPNode* node, ProblemData& data);
void PrintNodeInfo(BPNode* node);

// 启发式函数 (heuristic.cpp)
void RunHeuristic(ProblemParams& params, ProblemData& data, BPNode& root_node);

// 根节点列生成函数 (root_node.cpp)
void SolveRootCG(ProblemParams& params, ProblemData& data, BPNode& root_node);
bool SolveRootInitMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& root_node);
bool SolveRootUpdateMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& node);
bool SolveRootFinalMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& node);

// 根节点子问题函数 (root_node_sub.cpp)
// SP1: 宽度背包 - 选择条带放置在母板上
// 目标: max sum(v_j * G_j), 约束: sum(w_j * G_j) <= W
bool SolveRootSP1Knapsack(ProblemParams& params, ProblemData& data, BPNode& node);
bool SolveRootSP1ArcFlow(ProblemParams& params, ProblemData& data, BPNode& node);
bool SolveRootSP1DP(ProblemParams& params, ProblemData& data, BPNode& node);

// SP2: 长度背包 - 选择子件放置在条带上
// 目标: max sum(pi_i * D_i), 约束: sum(l_i * D_i) <= L
bool SolveRootSP2Knapsack(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);
bool SolveRootSP2ArcFlow(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);
bool SolveRootSP2DP(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);

// 非根节点列生成函数 (new_node.cpp)
int SolveNodeCG(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeInitMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode* node);
bool SolveNodeUpdateMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode* node);
bool SolveNodeFinalMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode* node);

// 非根节点子问题函数 (new_node_sub.cpp)
bool SolveNodeSP1Knapsack(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP1ArcFlow(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP1DP(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP2Knapsack(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);
bool SolveNodeSP2ArcFlow(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);
bool SolveNodeSP2DP(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);

// 列生成主流程函数 (column_generation.cpp)
bool SolveRootSP1(ProblemParams& params, ProblemData& data, BPNode& node);
bool SolveRootSP2(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);
bool SolveNodeSP1(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP2(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);

// 分支定价函数 (branch_and_price.cpp)
bool IsIntegerSolution(NodeSolution& solution);
int SelectBranchVar(BPNode* node);  // 变量分支 (已废弃)
int SelectBranchArc(ProblemParams& params, ProblemData& data, BPNode* node);  // Arc 分支
void CreateLeftChild(BPNode* parent, int new_id, BPNode* child);
void CreateRightChild(BPNode* parent, int new_id, BPNode* child);
int RunBranchAndPrice(ProblemParams& params, ProblemData& data, BPNode* root);
BPNode* SelectBranchNode(BPNode* head);

// 输出函数 (output.cpp)
void ExportResults(ProblemParams& params, ProblemData& data);
void ExportSolution(BPNode* node, ProblemData& data);

#endif  // CS_2D_BP_ARC_H_
