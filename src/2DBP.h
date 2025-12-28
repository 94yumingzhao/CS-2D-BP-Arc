// 2DBP.h - 二维下料问题分支定价求解器 主头文件
// 项目: CS-2D-BP-Arc
// 描述: 采用两阶段切割的二维下料问题分支定价算法
//
// 问题描述:
//   给定固定尺寸的母板 (长度L x 宽度W) 和多种子板类型 (各有长度、宽度、需求量)
//   目标是用最少的母板切割出所有需求的子板
//
// 两阶段切割:
//   第一阶段 (SP1): 沿宽度方向将母板切割成若干条带
//   第二阶段 (SP2): 沿长度方向将条带切割成子板
//   条带宽度由放入的子板宽度决定，条带长度等于母板长度
//
// 算法框架: Branch and Price = Column Generation + Branch and Bound
//   主问题 (MP): min sum_k y_k
//                s.t. sum_k c_{jk} y_k - sum_p x_p >= 0  (条带平衡约束)
//                     sum_p b_{ip} x_p >= d_i           (子板需求约束)
//   子问题 (SP1): 宽度方向背包，选择条带放置在母板上
//   子问题 (SP2): 长度方向背包，选择子板放置在条带上
//
// 子问题求解方法: CPLEX IP / Arc Flow / DP
// 分支策略: Arc 流量分支 (对分数流量的 Arc 进行分支)

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

// 全局常量定义
// 这些容差值用于处理浮点数比较，避免数值误差导致的错误判断
constexpr double kRcTolerance = 1.0e-6;     // 检验数 (Reduced Cost) 容差
                                            // RC > 1 + kRcTolerance 才认为找到改进列
constexpr double kZeroTolerance = 1.0e-10;  // 零值容差，|x| < kZeroTolerance 视为 0
constexpr int kMaxCgIter = 100;             // 列生成最大迭代次数，防止无限循环
const string kDataDir = "../CS-2D-Data/data/";  // 算例数据目录 (CS-2D-Data输出)
const string kFilePattern = "inst_";            // 算例文件名前缀
const string kLogDir = "logs/";             // 日志输出目录
const string kLpDir = "lp/";                // LP 文件输出目录 (调试用)
constexpr bool kExportLp = false;           // 是否导出 LP 文件，开启会降低性能
constexpr int kMaxBPTimeSec = 30;           // 分支定价最大运行时间 (秒)，超时输出当前最优解

// 子问题求解方法枚举
// 三种方法各有特点:
//   kCplexIP: 直接用 CPLEX 求解整数背包，简单但调用开销大
//   kArcFlow: 将背包建模为网络流，便于添加 Arc 约束实现分支
//   kDP: 动态规划求解完全背包，速度快但不支持 Arc 约束
enum SPMethod {
    kCplexIP = 0,   // CPLEX 整数规划求解
    kArcFlow = 1,   // Arc Flow 网络流模型
    kDP = 2         // 动态规划
};

// 分支类型枚举
// Arc 流量分支策略: 若某 Arc 的流量为分数，则对该 Arc 进行分支
//   左分支: Arc 流量 <= floor(流量)
//   右分支: Arc 流量 >= ceil(流量)
enum BranchType {
    kBranchNone = 0,    // 无需分支，当前解已是整数解
    kBranchSP1Arc = 1,  // SP1 Arc 分支，对宽度方向的 Arc 进行分支
    kBranchSP2Arc = 2   // SP2 Arc 分支，对长度方向的 Arc 进行分支
};

// 子板类型结构体
// 存储同一规格子板的聚合信息，相同尺寸的子板归为同一类型
struct ItemType {
    int type_id_ = -1;      // 类型编号，从 0 开始
    int length_ = -1;       // 长度 (沿 X 轴方向)
    int width_ = -1;        // 宽度 (沿 Y 轴方向)
    int demand_ = -1;       // 需求数量
};

// 条带类型结构体
// 条带是母板沿宽度方向切割后的中间产物
// 条带宽度由其包含的子板宽度决定，长度等于母板长度
struct StripType {
    int type_id_ = -1;      // 类型编号，从 0 开始
    int width_ = -1;        // 宽度 (沿 Y 轴，等于对应子板宽度)
    int length_ = -1;       // 长度 (沿 X 轴，等于母板长度)
};

// SP2 Arc Flow 网络数据结构
// 用于条带上的子板排列问题 (长度方向背包)
// 网络节点表示条带上的位置 (0 到 stock_length)
// Arc 表示在某位置放置一个子板，Arc 长度等于子板长度
struct SP2ArcFlowData {
    int strip_type_id_ = -1;                // 对应的条带类型编号

    // 节点分类: 起点 (位置0)、终点 (位置L)、中间节点
    vector<int> begin_nodes_;               // 起点节点列表 (只有一个元素: 0)
    vector<int> end_nodes_;                 // 终点节点列表 (只有一个元素: L)
    vector<int> mid_nodes_;                 // 中间节点列表 (位置 1 到 L-1 中有效的)

    // Arc 信息
    vector<array<int, 2>> arc_list_;        // Arc 列表，每个 Arc 为 [起点位置, 终点位置]
    map<array<int, 2>, int> arc_to_index_;  // Arc 到索引的映射，用于快速查找

    // Arc 分类索引，用于构建流量守恒约束
    vector<int> begin_arc_indices_;         // 从起点 (位置0) 出发的 Arc 索引列表
    vector<int> end_arc_indices_;           // 到达终点 (位置L) 的 Arc 索引列表
    vector<vector<int>> mid_in_arcs_;       // mid_in_arcs_[i] = 进入中间节点 i 的 Arc 索引
    vector<vector<int>> mid_out_arcs_;      // mid_out_arcs_[i] = 离开中间节点 i 的 Arc 索引
};

// SP1 Arc Flow 网络数据结构
// 用于母板上的条带排列问题 (宽度方向背包)
// 网络节点表示母板宽度方向的位置 (0 到 stock_width)
// Arc 表示在某位置放置一种条带，Arc 长度等于条带宽度
struct SP1ArcFlowData {
    // 节点分类
    vector<int> begin_nodes_;               // 起点节点 (位置0)
    vector<int> end_nodes_;                 // 终点节点 (位置W)
    vector<int> mid_nodes_;                 // 中间节点

    // Arc 信息
    vector<array<int, 2>> arc_list_;        // Arc 列表 [起点, 终点]
    map<array<int, 2>, int> arc_to_index_;  // Arc 到索引的快速映射

    // Arc 分类索引
    vector<int> begin_arc_indices_;         // 从起点出发的 Arc
    vector<int> end_arc_indices_;           // 到达终点的 Arc
    vector<vector<int>> mid_in_arcs_;       // 中间节点入弧
    vector<vector<int>> mid_out_arcs_;      // 中间节点出弧
};

// 新列结构体
// 列生成过程中子问题产生的新切割方案
struct NewColumn {
    vector<int> pattern_;               // 切割方案系数向量
                                        // Y列: pattern_[j] = 条带类型 j 的数量
                                        // X列: pattern_[i] = 子板类型 i 的数量
    set<array<int, 2>> arc_set_;        // 对应的 Arc 集合，用于 Arc 分支
};

// Y 列结构体 (第一阶段切割方案)
// 表示一种将母板切割为条带的方案
struct YColumn {
    vector<int> pattern_;               // pattern_[j] = 该方案中条带类型 j 的产出数量
    set<array<int, 2>> arc_set_;        // 对应的 SP1 Arc 集合 (宽度方向)
    double value_ = 0.0;                // LP 解中该列的取值
};

// X 列结构体 (第二阶段切割方案)
// 表示一种将条带切割为子板的方案
struct XColumn {
    int strip_type_id_ = -1;            // 所属条带类型编号
    vector<int> pattern_;               // pattern_[i] = 该方案中子板类型 i 的产出数量
    set<array<int, 2>> arc_set_;        // 对应的 SP2 Arc 集合 (长度方向)
    double value_ = 0.0;                // LP 解中该列的取值
};

// 节点解结构体
// 存储分支定价节点的 LP 求解结果
struct NodeSolution {
    vector<YColumn> y_columns_;         // Y 列集合及其 LP 解值
    vector<XColumn> x_columns_;         // X 列集合及其 LP 解值
    double obj_val_ = -1;               // 目标函数值 (母板使用量)
};

// 分支定价节点结构体
// 分支定价树中的一个节点，包含该节点的所有状态信息
struct BPNode {
    // 子问题求解方法配置
    int sp1_method_ = 0;        // SP1 求解方法: 0=CPLEX, 1=ArcFlow, 2=DP
    int sp2_method_ = 0;        // SP2 求解方法: 0=CPLEX, 1=ArcFlow, 2=DP

    // 节点标识信息
    int id_ = -1;               // 节点编号，从 1 开始
    int parent_id_ = -1;        // 父节点编号，-1 表示根节点
    double lower_bound_ = -1;   // 节点下界 (LP 松弛解的目标值)

    // 分支状态标志
    int branch_dir_ = -1;       // 分支方向: 1=左分支(<=), 2=右分支(>=)
    int prune_flag_ = 0;        // 剪枝标志: 0=未剪枝, 1=已剪枝
                                // 节点被剪枝的条件: 不可行 或 下界 >= 全局最优整数解
    int branched_flag_ = 0;     // 分支完成标志: 0=未分支, 1=已创建子节点

    // 变量分支信息 (已废弃，保留兼容性)
    int branch_var_id_ = -1;            // 待分支变量索引
    double branch_var_val_ = -1;        // 待分支变量的 LP 解值 (分数)
    double branch_floor_ = -1;          // 向下取整值
    double branch_ceil_ = -1;           // 向上取整值
    vector<int> branched_var_ids_;      // 已分支变量索引历史
    vector<double> branched_bounds_;    // 已分支变量的整数边界

    // Arc 分支信息
    int branch_type_ = kBranchNone;             // 当前分支类型
    array<int, 2> branch_arc_ = {-1, -1};       // 待分支 Arc [起点, 终点]
    double branch_arc_flow_ = -1;               // Arc 流量值 (分数)
    int branch_arc_strip_type_ = -1;            // SP2 分支时的条带类型编号

    // SP1 Arc 约束 (宽度方向)
    // 这些约束从父节点累积继承，用于限制 SP1 子问题的可行域
    set<array<int, 2>> sp1_zero_arcs_;          // 禁用的 Arc (流量 = 0)
    vector<array<int, 2>> sp1_lower_arcs_;      // 上界约束的 Arc
    vector<int> sp1_lower_bounds_;              // sp1_lower_arcs_[i] 的流量 <= sp1_lower_bounds_[i]
    vector<array<int, 2>> sp1_greater_arcs_;    // 下界约束的 Arc
    vector<int> sp1_greater_bounds_;            // sp1_greater_arcs_[i] 的流量 >= sp1_greater_bounds_[i]

    // SP2 Arc 约束 (长度方向，按条带类型存储)
    // 不同条带类型有独立的 SP2 网络，约束也分开存储
    map<int, set<array<int, 2>>> sp2_zero_arcs_;        // sp2_zero_arcs_[j] = 条带类型 j 禁用的 Arc
    map<int, vector<array<int, 2>>> sp2_lower_arcs_;    // 条带类型 j 的上界约束 Arc
    map<int, vector<int>> sp2_lower_bounds_;            // 对应的上界值
    map<int, vector<array<int, 2>>> sp2_greater_arcs_;  // 条带类型 j 的下界约束 Arc
    map<int, vector<int>> sp2_greater_bounds_;          // 对应的下界值

    // 主问题数据
    vector<vector<double>> matrix_;             // 完整系数矩阵 (调试用)
    vector<YColumn> y_columns_;                 // 当前节点的 Y 列集合
    vector<XColumn> x_columns_;                 // 当前节点的 X 列集合
    vector<set<array<int, 2>>> y_arc_sets_;     // Y 列对应的 Arc 集合
    vector<set<array<int, 2>>> x_arc_sets_;     // X 列对应的 Arc 集合

    // 列生成迭代状态
    int iter_ = -1;                     // 当前迭代次数
    vector<double> duals_;              // 对偶价格向量
                                        // duals_[0..J-1] = 条带平衡约束的对偶价格
                                        // duals_[J..J+N-1] = 子板需求约束的对偶价格
    NewColumn new_y_col_;               // 本次迭代 SP1 产生的新 Y 列
    NewColumn new_x_col_;               // 本次迭代 SP2 产生的新 X 列
    int new_strip_type_ = -1;           // 新 X 列对应的条带类型

    // SP2 临时数据
    double sp2_obj_ = -1;               // SP2 目标函数值
    vector<double> sp2_solution_;       // SP2 解向量

    // 节点求解结果
    NodeSolution solution_;             // LP 求解结果

    // 链表指针，用于节点队列管理
    BPNode* next_ = nullptr;
};

// 问题参数结构体
// 存储算法运行过程中的全局参数和最优解信息
struct ProblemParams {
    // 问题规模
    int num_item_types_ = -1;           // 子板类型数量 (N)
    int num_strip_types_ = -1;          // 条带类型数量 (J)，等于不同子板宽度的数量
    int num_items_ = -1;                // 子板总数 (所有需求之和)

    // 母板尺寸
    int stock_length_ = -1;             // 母板长度 (L，沿 X 轴)
    int stock_width_ = -1;              // 母板宽度 (W，沿 Y 轴)

    // 子问题求解方法设置
    int sp1_method_ = kCplexIP;         // SP1 默认求解方法
    int sp2_method_ = kCplexIP;         // SP2 默认求解方法

    // 分支定价树状态
    int node_counter_ = 1;              // 节点编号计数器
    double optimal_lb_ = INFINITY;      // 当前最优下界 (所有未剪枝节点下界的最小值)

    // 全局最优整数解信息
    double global_best_int_ = INFINITY;         // 最优整数解目标值
    vector<YColumn> global_best_y_cols_;        // 最优整数解的 Y 列
    vector<XColumn> global_best_x_cols_;        // 最优整数解的 X 列
    double gap_ = INFINITY;                     // 最优性间隙 = (UB - LB) / UB

    // 初始解矩阵 (启发式生成)
    vector<vector<int>> init_y_matrix_;         // 初始 Y 列矩阵
    vector<vector<int>> init_x_matrix_;         // 初始 X 列矩阵
};

// 问题数据结构体
// 存储问题的输入数据和 Arc Flow 模型数据
struct ProblemData {
    // 基本数据
    vector<ItemType> item_types_;               // 子板类型列表
    vector<StripType> strip_types_;             // 条带类型列表
    vector<int> item_lengths_;                  // 子板长度列表 (按降序排列)
    vector<int> strip_widths_;                  // 条带宽度列表 (按降序排列)

    // 索引映射，用于快速查找
    map<int, int> length_to_item_index_;        // 长度 -> 子板类型索引
    map<int, int> width_to_strip_index_;        // 宽度 -> 条带类型索引
    map<int, vector<int>> width_to_item_indices_;  // 宽度 -> 该宽度的子板类型索引列表

    // SP1 Arc Flow 网络 (宽度方向，只有一个)
    SP1ArcFlowData sp1_arc_data_;

    // SP2 Arc Flow 网络 (长度方向，每种条带类型一个)
    vector<SP2ArcFlowData> sp2_arc_data_;
};

// Arc Flow 网络生成函数 (arc_flow.cpp)
// 生成 SP1 宽度方向的 Arc Flow 网络
void GenerateSP1Arcs(ProblemData& data, ProblemParams& params);

// 生成指定条带类型的 SP2 长度方向 Arc Flow 网络
void GenerateSP2Arcs(ProblemData& data, ProblemParams& params, int strip_type_id);

// 生成所有 Arc Flow 网络 (SP1 + 所有 SP2)
void GenerateAllArcs(ProblemData& data, ProblemParams& params);

// 将切割方案 (pattern) 转换为 Arc 集合
void ConvertPatternToArcSet(vector<int>& pattern, vector<int>& sizes,
    set<array<int, 2>>& arc_set);

// 为节点的 Y 列生成 Arc 集合
void GenerateYArcSetMatrix(BPNode& node, vector<int>& strip_widths);

// 为节点的 X 列生成 Arc 集合
void GenerateXArcSetMatrix(BPNode& node, vector<int>& item_lengths, int strip_type);

// Arc Flow 解转换函数 (arc_flow.cpp)
// 将 Y 列 LP 解转换为 SP1 Arc 流量
void ConvertYColsToSP1ArcFlow(vector<YColumn>& y_columns, ProblemData& data,
    map<int, tuple<int, int, double>>& arc_flow_solution);

// 将 X 列 LP 解转换为 SP2 Arc 流量
void ConvertXColsToSP2ArcFlow(vector<XColumn>& x_columns, int strip_type_id,
    ProblemData& data, map<int, tuple<int, int, double>>& arc_flow_solution);

// 在 SP1 Arc 流量中寻找分数流量的 Arc (用于分支)
bool FindBranchArcSP1(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow);

// 在 SP2 Arc 流量中寻找分数流量的 Arc
bool FindBranchArcSP2(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow);

// 打印 Arc Flow 解 (调试用)
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
// 使用贪心策略生成初始可行解，加速列生成收敛
void RunHeuristic(ProblemParams& params, ProblemData& data, BPNode& root_node);

// 根节点列生成函数 (root_node.cpp)
// 根节点列生成主循环
void SolveRootCG(ProblemParams& params, ProblemData& data, BPNode& root_node);

// 构建根节点初始主问题 (复用cplex对象)
bool SolveRootInitMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars,
    IloCplex& cplex, BPNode& root_node);

// 更新根节点主问题 (添加新列, 复用cplex对象)
bool SolveRootUpdateMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars,
    IloCplex& cplex, BPNode& node);

// 求解根节点最终主问题并提取解 (复用cplex对象)
bool SolveRootFinalMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars,
    IloCplex& cplex, BPNode& node);

// 根节点子问题函数 (root_node_sub.cpp)
// SP1: 宽度背包问题 - 选择条带放置在母板上
// 数学模型: max sum(v_j * G_j), s.t. sum(w_j * G_j) <= W
// 其中 v_j 是条带类型 j 的对偶价格，w_j 是条带宽度
bool SolveRootSP1Knapsack(ProblemParams& params, ProblemData& data, BPNode& node);
bool SolveRootSP1ArcFlow(ProblemParams& params, ProblemData& data, BPNode& node);
bool SolveRootSP1DP(ProblemParams& params, ProblemData& data, BPNode& node);

// SP2: 长度背包问题 - 选择子板放置在条带上
// 数学模型: max sum(pi_i * D_i), s.t. sum(l_i * D_i) <= L
// 其中 pi_i 是子板类型 i 的对偶价格，l_i 是子板长度
// 只考虑宽度不超过条带宽度的子板
bool SolveRootSP2Knapsack(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);
bool SolveRootSP2ArcFlow(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);
bool SolveRootSP2DP(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);

// 非根节点列生成函数 (new_node.cpp)
// 非根节点需要考虑从父节点继承的 Arc 约束
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
// 与根节点类似，但需要应用该节点累积的 Arc 约束
bool SolveNodeSP1Knapsack(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP1ArcFlow(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP1DP(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP2Knapsack(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);
bool SolveNodeSP2ArcFlow(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);
bool SolveNodeSP2DP(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);

// 列生成调度函数 (column_generation.cpp)
// 根据配置的求解方法调用对应的子问题求解函数
bool SolveRootSP1(ProblemParams& params, ProblemData& data, BPNode& node);
bool SolveRootSP2(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id);
bool SolveNodeSP1(ProblemParams& params, ProblemData& data, BPNode* node);
bool SolveNodeSP2(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id);

// 分支定价函数 (branch_and_price.cpp)
// 检查 LP 解是否为整数解
bool IsIntegerSolution(NodeSolution& solution);

// 选择分支变量 (变量分支，已废弃)
int SelectBranchVar(BPNode* node);

// 选择分支 Arc (Arc 流量分支)
// 优先检查 SP1 Arc，若全整数再检查 SP2 Arc
int SelectBranchArc(ProblemParams& params, ProblemData& data, BPNode* node);

// 创建左子节点 (Arc <= floor)
void CreateLeftChild(BPNode* parent, int new_id, BPNode* child);

// 创建右子节点 (Arc >= ceil)
void CreateRightChild(BPNode* parent, int new_id, BPNode* child);

// 选择待分支节点 (选择下界最小的未剪枝节点)
BPNode* SelectBranchNode(BPNode* head);

// 分支定价主循环
int RunBranchAndPrice(ProblemParams& params, ProblemData& data, BPNode* root);

// 输出函数 (output.cpp)
void ExportSolution(ProblemParams& params, ProblemData& data);
void ExportResults(ProblemParams& params, ProblemData& data);

#endif  // CS_2D_BP_ARC_H_
