// =============================================================================
// update_master_problem.cpp - 主问题更新模块
// =============================================================================
//
// 功能: 在列生成过程中动态更新主问题
//
// 包含两个核心函数:
//   1. SolveUpdateMasterProblem: 添加新列并重新求解
//   2. SolveFinalMasterProblem: 列生成收敛后求解最终问题
//
// 列生成迭代流程:
//
//   ┌─────────────────┐
//   │  求解主问题 MP  │
//   │  (获取对偶价格) │
//   └────────┬────────┘
//            │
//            v
//   ┌─────────────────┐
//   │  求解子问题 SP  │
//   │  (寻找改进列)   │
//   └────────┬────────┘
//            │
//    ┌───────┴───────┐
//    │ Reduced Cost  │
//    │    < 0 ?      │
//    └───────┬───────┘
//       Yes  │  No
//            │   └──> 收敛, 调用 SolveFinalMasterProblem
//            v
//   ┌─────────────────┐
//   │  添加新列到 MP  │──┐
//   └─────────────────┘  │
//            ^           │
//            └───────────┘
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveUpdateMasterProblem - 添加新列并重新求解主问题
// -----------------------------------------------------------------------------
// 功能: 将子问题找到的改进列添加到主问题, 然后重新求解
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   Env_MP    - CPLEX 环境
//   Model_MP  - 主问题模型
//   Obj_MP    - 目标函数对象
//   Cons_MP   - 约束数组
//   Vars_MP   - 变量数组
//   this_node - 当前节点
//
// 新列类型判断:
//   - Y_col_flag = 1: 添加新的 Y 列 (母板切割模式)
//   - Y_col_flag = 0: 添加新的 X 列 (条带切割模式)
// -----------------------------------------------------------------------------
void SolveUpdateMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_MP,
	IloNumVarArray& Vars_MP,
	Node& this_node) {

	// =========================================================================
	// 问题规模
	// =========================================================================
	int K_num = this_node.Y_cols_list.size();
	int P_num = this_node.X_cols_list.size();
	int J_num = Values.strip_types_num;
	int N_num = Values.item_types_num;
	int all_cols_num = K_num + P_num;
	int all_rows_num = J_num + N_num;

	// =========================================================================
	// 情况 1: 添加新的 Y 列 (母板切割模式)
	// =========================================================================
	// Y 变量特点:
	//   - 目标系数 = 1 (每使用一块母板, 目标函数 +1)
	//   - 添加到变量列表末尾
	// =========================================================================
	if (this_node.Y_col_flag == 1) {
		// ----- 构建 CPLEX 列 -----
		IloNum obj_para = 1;
		IloNumColumn CplexCol = (Obj_MP)(obj_para);

		for (int row = 0; row < all_rows_num; row++) {
			IloNum row_para = this_node.new_Y_col[row];
			CplexCol += (Cons_MP)[row](row_para);
		}

		// ----- 创建新变量 -----
		int cols_num = this_node.Y_cols_list.size();
		string var_name = "Y_" + to_string(cols_num + 1);
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, var_name.c_str());
		(Vars_MP).add(Var_Y);
		CplexCol.end();

		// ----- 更新节点的列集合 -----
		vector<double> new_col;
		for (int row = 0; row < all_rows_num; row++) {
			new_col.push_back(this_node.new_Y_col[row]);
		}

		this_node.Y_cols_list.push_back(new_col);
		this_node.model_matrix.insert(this_node.model_matrix.begin() + this_node.Y_cols_list.size(), new_col);
		this_node.new_Y_col.clear();
	}

	// =========================================================================
	// 情况 2: 添加新的 X 列 (条带切割模式)
	// =========================================================================
	// X 变量特点:
	//   - 目标系数 = 0 (条带模式不直接影响目标)
	//   - 可能同时添加多个新列
	// =========================================================================
	if (this_node.Y_col_flag == 0) {
		int new_cols_num = this_node.new_X_cols_list.size();

		for (int col = 0; col < new_cols_num; col++) {
			// ----- 构建 CPLEX 列 -----
			IloNum obj_para = 0;
			IloNumColumn CplexCol = (Obj_MP)(obj_para);

			for (int row = 0; row < all_rows_num; row++) {
				IloNum row_para = this_node.new_X_cols_list[col][row];
				CplexCol += (Cons_MP)[row](row_para);
			}

			// ----- 创建新变量 -----
			int old_item_cols_num = this_node.X_cols_list.size();
			string var_name = "X_" + to_string(old_item_cols_num + 1);
			IloNum var_min = 0;
			IloNum var_max = IloInfinity;
			IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, var_name.c_str());
			(Vars_MP).add(Var_X);
			CplexCol.end();

			// ----- 更新节点的列集合 -----
			vector<double> temp_col;
			for (int row = 0; row < all_rows_num; row++) {
				temp_col.push_back(this_node.new_X_cols_list[col][row]);
			}

			this_node.X_cols_list.push_back(temp_col);
			this_node.model_matrix.insert(this_node.model_matrix.end(), temp_col);
		}

		this_node.new_X_cols_list.clear();
	}

	// =========================================================================
	// 求解更新后的主问题
	// =========================================================================
	IloCplex MP_cplex(Model_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.setOut(Env_MP.getNullStream());
	MP_cplex.solve();

	cout << "[主问题] 迭代 " << this_node.iter << ", 目标值 = "
	     << fixed << setprecision(4) << MP_cplex.getValue(Obj_MP) << "\n";
	cout.unsetf(ios::fixed);

	// =========================================================================
	// 提取对偶价格 (用于下一次子问题定价)
	// =========================================================================
	this_node.dual_prices_list.clear();

	// ----- 条带约束对偶价格 -----
	for (int row = 0; row < J_num; row++) {
		double dual_val = MP_cplex.getDual(Cons_MP[row]);
		if (dual_val == -0) dual_val = 0;
		this_node.dual_prices_list.push_back(dual_val);
	}

	// ----- 子件约束对偶价格 -----
	for (int row = J_num; row < J_num + N_num; row++) {
		double dual_val = MP_cplex.getDual(Cons_MP[row]);
		if (dual_val == -0) dual_val = 0;
		this_node.dual_prices_list.push_back(dual_val);
	}
}

// -----------------------------------------------------------------------------
// SolveFinalMasterProblem - 求解列生成收敛后的最终主问题
// -----------------------------------------------------------------------------
// 功能: 当子问题无法找到改进列时, 求解最终 LP 并记录下界
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   Env_MP    - CPLEX 环境
//   Model_MP  - 主问题模型
//   Obj_MP    - 目标函数对象
//   Cons_MP   - 约束数组
//   Vars_MP   - 变量数组
//   this_node - 当前节点
//
// 输出:
//   - this_node.LB: 节点的 LP 下界
//   - this_node.all_solns_val_list: 所有变量的解值
// -----------------------------------------------------------------------------
void SolveFinalMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_MP,
	IloNumVarArray& Vars_MP,
	Node& this_node) {

	// =========================================================================
	// 问题规模
	// =========================================================================
	int K_num = this_node.Y_cols_list.size();
	int P_num = this_node.X_cols_list.size();
	int N_num = Values.item_types_num;
	int J_num = Values.strip_types_num;
	int all_rows_num = N_num + J_num;
	int all_cols_num = K_num + P_num;

	// =========================================================================
	// 求解最终主问题
	// =========================================================================
	IloCplex MP_cplex(Model_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.setOut(Env_MP.getNullStream());
	MP_cplex.exportModel("Final Master Problem.lp");
	MP_cplex.solve();

	// ----- 记录 LP 下界 -----
	this_node.LB = MP_cplex.getValue(Obj_MP);

	cout << "[主问题] 最终目标值 = " << fixed << setprecision(4) << this_node.LB << "\n";
	cout.unsetf(ios::fixed);

	// =========================================================================
	// 保存所有变量的解值
	// =========================================================================
	// 用于:
	//   1. 分支决策 (选择分数解进行分支)
	//   2. 启发式构造整数解
	// =========================================================================
	for (int col = 0; col < all_cols_num; col++) {
		IloNum soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val == -0) soln_val = 0;
		this_node.all_solns_val_list.push_back(soln_val);
	}

	// =========================================================================
	// 统计非零解 (用于调试输出)
	// =========================================================================
	int Y_fsb_num = 0;
	int X_fsb_num = 0;

	for (int col = 0; col < K_num; col++) {
		double soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val > 0) Y_fsb_num++;
	}

	for (int col = K_num; col < K_num + P_num; col++) {
		double soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val > 0) X_fsb_num++;
	}

	cout << "[主问题] 非零解: Y=" << Y_fsb_num << ", X=" << X_fsb_num
	     << " (总变量: " << all_cols_num << ")\n";
}
