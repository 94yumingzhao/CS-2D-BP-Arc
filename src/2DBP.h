// =============================================================================
// 2DBP.h - 二维下料问题分支定价求解器 主头文件
// =============================================================================
//
// 项目: CS-2D-BP (2D Cutting Stock Problem - Branch and Price)
// 描述: 采用两阶段切割的二维下料问题分支定价算法实现
//
// 切割层次: 母板(Stock) -> 条带(Strip) -> 子件(Item)
//   - 第一阶段: 沿宽度方向将母板切割成条带
//   - 第二阶段: 沿长度方向将条带切割成子件
//
// 缩写说明:
//   CG  - Column Generation (列生成)
//   MP  - Master Problem (主问题)
//   SP  - Sub Problem (子问题)
//   LB  - Lower Bound (下界)
//   UB  - Upper Bound (上界)
//   var - variable (变量)
//   con - constraint (约束)
//   col - column (列)
//   val - value (值)
//   soln - solution (解)
//   fsb - feasible (可行)
//   int - integer (整数)
//
// =============================================================================

#ifndef CS_2D_BP_H_
#define CS_2D_BP_H_

#include <vector>
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <array>
#include <algorithm>
#include <cstdio>
#include <ilcplex/ilocplex.h>

using namespace std;

// 检验数阈值: 用于判断是否找到改进列
#define RC_EPS 1.0e-6

// =============================================================================
// 数据结构定义
// =============================================================================

// -----------------------------------------------------------------------------
// 子件类型结构体
// 描述: 存储一种子件类型的基本信息
// -----------------------------------------------------------------------------
struct One_Item_Type {
	int item_type_idx = -1;       // 子件类型索引
	double demand = -1;           // 需求量
	int length = -1;              // 长度
	int width = -1;               // 宽度
	int this_item_type_num = 0;   // 当前模式中该类型的数量
};

// -----------------------------------------------------------------------------
// 条带类型结构体
// 描述: 存储一种条带类型的基本信息
// -----------------------------------------------------------------------------
struct One_Strip_Type {
	int strip_type_idx = -1;      // 条带类型索引
	int width = -1;               // 宽度 (由首个子件宽度决定)
	int length = -1;              // 长度 (等于母板长度)
	int this_strip_type_num = 0;  // 当前模式中该类型的数量
};

// -----------------------------------------------------------------------------
// 母板类型结构体
// 描述: 存储一种母板类型的基本信息
// -----------------------------------------------------------------------------
struct One_Stock_Type {
	int stock_type_idx = -1;      // 母板类型索引
	int this_stock_type_num = 0;  // 当前方案中该类型的数量
};

// -----------------------------------------------------------------------------
// 单个子件结构体
// 描述: 存储单个子件实例的详细信息
// -----------------------------------------------------------------------------
struct One_Item {
	int item_idx = -1;            // 子件实例索引
	int item_type_idx = -1;       // 所属子件类型
	int demand = -1;              // 需求量
	int length = -1;              // 长度
	int width = -1;               // 宽度
	int area = -1;                // 面积

	int pos_x = -1;               // 左上角 X 坐标
	int pos_y = -1;               // 左上角 Y 坐标

	int strip_idx = -1;           // 所属条带索引
	int stock_idx = -1;           // 所属母板索引

	int occupied_flag = 0;        // 是否已被分配: 0=未分配, 1=已分配

	int cutting_distance = -1;    // 切割距离
	int material_cutting_loss = -1; // 切割损耗
};

// -----------------------------------------------------------------------------
// 单个条带结构体
// 描述: 存储单个条带的详细信息及其包含的子件
// -----------------------------------------------------------------------------
struct One_Strip {
	int strip_idx = -1;           // 条带索引
	int strip_type_idx = -1;      // 所属条带类型
	int pattern = -1;             // 切割模式编号

	vector<One_Item> items_list;            // 该条带包含的所有子件
	vector<One_Item_Type> item_types_list;  // 各子件类型的统计信息

	int length = -1;              // 长度 (等于母板长度)
	int width = -1;               // 宽度 (由首个子件决定)
	int area = -1;                // 面积

	int pos_x = -1;               // 左上角 X 坐标
	int pos_y = -1;               // 左上角 Y 坐标

	int stock_idx = -1;           // 所属母板索引

	int cutting_distance = -1;    // 切割距离
	int material_cutting_loss = -1; // 切割损耗
	int wasted_area = -1;         // 废料面积
	int material_area_loss = -1;  // 面积损耗
};

// -----------------------------------------------------------------------------
// 单个母板结构体
// 描述: 存储单个母板的详细信息及其包含的条带
// -----------------------------------------------------------------------------
struct One_Stock {
	int stock_idx = -1;           // 母板索引
	int stock_type_idx = 0;       // 所属母板类型
	int pattern = -1;             // 切割模式编号

	vector<One_Strip> strips_list;            // 该母板包含的所有条带
	vector<One_Strip_Type> strip_types_list;  // 各条带类型的统计信息

	int length = -1;              // 长度
	int width = -1;               // 宽度
	int area = -1;                // 面积

	int pos_x = -1;               // 左上角 X 坐标
	int pos_y = -1;               // 左上角 Y 坐标

	int cutting_distance = -1;    // 切割距离
	int material_cutting_loss = -1; // 切割损耗
	int wasted_area = -1;         // 废料面积
	int material_area_loss = -1;  // 面积损耗
};

// -----------------------------------------------------------------------------
// 分支定界节点结构体
// 描述: 存储分支定价树中单个节点的完整状态
// -----------------------------------------------------------------------------
struct Node {
	int index = -1;               // 节点索引 (根节点为1)

	// --- 父节点信息 ---
	int parent_index = -1;                // 父节点索引
	int parent_branching_flag = -1;       // 父节点的分支方向: 1=左, 2=右
	double parent_var_to_branch_val = -1; // 父节点分支变量的原始值

	// --- 节点状态 ---
	double LB = -1;               // 该节点的下界值
	int node_branched_flag = -1;  // 分支标志: 1=左分支, 2=右分支
	int node_pruned_flag = -1;    // 剪枝标志: 0=未剪枝, 1=已剪枝

	// --- 分支变量信息 ---
	int var_to_branch_idx = -1;      // 待分支变量的列索引
	double var_to_branch_soln = -1;  // 待分支变量的解值
	double var_to_branch_floor = -1; // 向下取整值
	double var_to_branch_ceil = -1;  // 向上取整值
	double var_to_branch_final = -1; // 最终确定的整数值

	// --- 分支历史 (从根节点到当前节点的路径) ---
	vector<int> branched_idx_list;      // 所有已分支变量的列索引
	vector<double> branched_int_list;   // 所有已分支变量的整数值
	vector<double> branched_solns_ist;  // 所有已分支变量的原始解值

	// --- 解信息 ---
	vector<double> all_solns_val_list;  // 所有变量的最终解值 (包含零值)

	// --- 列生成迭代数据 ---
	int iter = -1;                          // 当前迭代次数
	vector<vector<double>> model_matrix;    // 当前主问题的系数矩阵
	vector<double> dual_prices_list;        // 主问题约束的对偶价格

	// --- 切割模式存储 ---
	vector<One_Stock> Y_patterns_list;  // 第一阶段模式详细信息 (母板切割)
	vector<One_Strip> X_patterns_list;  // 第二阶段模式详细信息 (条带切割)
	vector<vector<double>> Y_cols_list; // 第一阶段模式的系数列
	vector<vector<double>> X_cols_list; // 第二阶段模式的系数列

	// --- 新生成的列 ---
	vector<double> new_Y_col;                 // 新的第一阶段模式列
	vector<vector<double>> new_X_cols_list;   // 新的第二阶段模式列集合

	// --- 子问题信息 ---
	double SP2_obj_val = -1;          // 第二阶段子问题目标值
	vector<double> SP2_solns_list;    // 第二阶段子问题的解
	int Y_col_flag = -1;              // 新列类型标志: 1=Y列, 0=X列
};

// -----------------------------------------------------------------------------
// 全局参数结构体
// 描述: 存储问题的全局参数和算法状态
// -----------------------------------------------------------------------------
struct All_Values {
	bool Finish;                  // 启发式完成标志

	int item_types_num = -1;      // 子件类型数量
	int strip_types_num = -1;     // 条带类型数量
	int stocks_num = -1;          // 母板数量
	int stock_types_num = -1;     // 母板类型数量
	int stock_length = -1;        // 母板长度
	int stock_width = -1;         // 母板宽度
	int items_num = -1;           // 子件总数
	int strips_num = -1;          // 条带总数

	int level_num = -1;           // 分支树层数
	int node_num = -1;            // 节点总数

	int unit_cut_loss = -1;       // 单位切割损耗
	int unit_area_loss = -1;      // 单位面积损耗
	int final_cut_loss = -1;      // 最终切割损耗
	int final_area_loss = -1;     // 最终面积损耗

	double optimal_LB = -1;       // 当前最优下界

	// --- 分支状态 ---
	// 1 = 生成左分支节点
	// 2 = 生成右分支节点
	// 3 = 搜索已生成的节点
	int branch_status = -1;

	// --- 搜索标志 ---
	// 0 = 继续分支当前节点
	// 1 = 停止分支, 搜索其他节点
	int search_flag = -1;

	// --- 深入标志 ---
	// 1 = 深入左节点
	// 2 = 深入右节点
	int fathom_flag = -1;

	int root_flag = -1;           // 根节点标志
};

// -----------------------------------------------------------------------------
// 全局列表结构体
// 描述: 存储所有数据对象的列表
// -----------------------------------------------------------------------------
struct All_Lists {
	vector<Node> all_nodes_list;              // 所有分支节点

	vector<One_Stock> all_stocks_list;        // 所有母板
	vector<One_Strip> all_strips_list;        // 所有条带
	vector<One_Item> all_items_list;          // 所有子件

	vector<One_Strip_Type> all_strip_types_list;  // 所有条带类型
	vector<One_Item_Type> all_item_types_list;    // 所有子件类型

	vector<One_Stock> occupied_stocks_list;   // 已使用的母板
	vector<One_Item> occupied_items_list;     // 已分配的子件
};

// =============================================================================
// 函数声明
// =============================================================================

// --- 工具函数 ---
// 按分隔符分割字符串
void SplitString(const string& s, vector<string>& v, const string& c);

// --- 数据读取 ---
// 从文件读取问题数据
void ReadData(All_Values& Values, All_Lists& Lists);

// --- 启发式 ---
// 原始启发式: 生成初始可行解和模型矩阵
void PrimalHeuristic(All_Values& Values, All_Lists& Lists, Node& root_node);

// --- 根节点列生成 ---
// 根节点的列生成主循环
void RootNodeColumnGeneration(All_Values& Values, All_Lists& Lists, Node& root_node);

// 求解根节点的初始主问题
// 返回值: true=可行, false=不可行
bool SolveRootNodeFirstMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_List_MP,
	IloNumVarArray& Vars_List_MP,
	Node& root_node);

// --- 子问题求解 ---
// 求解第一阶段子问题 (宽度方向背包)
// 返回值: 0=无改进列, 1=找到改进列
int SolveStageOneSubProblem(All_Values& Values, All_Lists& Lists, Node& this_node);

// 求解第二阶段子问题 (长度方向背包)
// 参数 strip_type_idx: 条带类型索引
// 返回值: 0=无改进列, 1=找到改进列
int SolveStageTwoSubProblem(All_Values& Values, All_Lists& Lists, Node& this_node, int strip_type_idx);

// --- 主问题更新 ---
// 添加新列并更新主问题
void SolveUpdateMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_MP,
	IloNumVarArray& Vars_MP,
	Node& this_node);

// 求解最终主问题, 提取最优解
void SolveFinalMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_List_MP,
	IloNumVarArray& Vars_List_MP,
	Node& this_node);

// --- 节点处理 ---
// 完成节点处理: 检查整数性, 更新最优解
// 返回值: 0=继续分支, 1=搜索其他节点
int FinishNode(All_Values& Values, All_Lists& Lists, Node& this_node);

// 选择待分支变量
// 返回值: 0=存在分数解, 1=全为整数解
int ChooseVarToBranch(All_Values& Values, All_Lists& Lists, Node& this_node);

// --- 分支定界 ---
// 分支定价树的主循环
int BranchAndPriceTree(All_Values& Values, All_Lists& Lists);

// 选择待分支节点
// 返回值: 0=无可分支节点, 1=找到待分支节点
int ChooseNodeToBranch(All_Values& Values, All_Lists& Lists, Node& parent_node);

// 生成新的分支节点
void GenerateNewNode(All_Values& Values, All_Lists& Lists, Node& new_node, Node& parent_node);

// --- 非根节点列生成 ---
// 非根节点的列生成循环
void NewNodeColumnGeneration(All_Values& Values, All_Lists& Lists, Node& this_node, Node& parent_node);

// 求解非根节点的初始主问题
// 返回值: true=可行, false=不可行
bool SolveNewNodeFirstMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_List_MP,
	IloNumVarArray& Vars_List_MP,
	Node& this_node,
	Node& parent_node);

// --- 输出函数 ---
// 输出主问题模型
void OutputMasterProblem(All_Values& Values, All_Lists& Lists, Node& this_node);

// 输出对偶主问题模型
void OutputDualMasterProblem(All_Values& Values, All_Lists& Lists, Node& this_node);

// 输出启发式结果
void OutputHeuristicResults(All_Values& Values, All_Lists& Lists);

// 输出最终结果
void OutputFinalResults(All_Values& Values, All_Lists& Lists);

#endif  // CS_2D_BP_H_
