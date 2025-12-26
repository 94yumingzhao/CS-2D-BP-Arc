// =============================================================================
// new_node_first_master_problem.cpp - 新节点初始主问题
// =============================================================================
//
// 功能: 构建并求解分支定界树中新节点的初始受限主问题
//
// 与根节点的区别:
//   根节点: 所有变量边界为 [0, +inf)
//   新节点: 部分变量被固定为整数值
//
// 变量边界处理逻辑:
//
//   ┌─────────────────────────────────────────────────────────────────┐
//   │                        变量 Y_k 或 X_p                          │
//   └─────────────────────────────────────────────────────────────────┘
//                                   │
//                    ┌──────────────┴──────────────┐
//                    │  是当前分支变量?            │
//                    └──────────────┬──────────────┘
//                      Yes          │          No
//                       │           │           │
//                       v           │           v
//              ┌────────────────┐   │   ┌─────────────────────────┐
//              │  Case 1:       │   │   │  检查是否为已分支变量   │
//              │  固定为        │   │   └────────────┬────────────┘
//              │  floor/ceil    │   │         Yes    │      No
//              └────────────────┘   │          │     │       │
//                                   │          v     │       v
//                                   │   ┌──────────┐ │  ┌──────────┐
//                                   │   │ Case 2.1:│ │  │ Case 2.2:│
//                                   │   │ 固定为   │ │  │ 保持     │
//                                   │   │ 历史值   │ │  │ [0,+inf) │
//                                   │   └──────────┘ │  └──────────┘
//
// 数学模型:
//
//   min  sum_{k} y_k
//
//   s.t. sum_{k} c_{jk} * y_k - sum_{p} d_{jp} * x_p >= 0  (条带平衡)
//        sum_{p} b_{ip} * x_p >= demand_i                  (子件需求)
//        y_k = fixed_val  (对于已分支变量)
//        y_k >= 0         (对于未分支变量)
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveNewNodeFirstMasterProblem - 求解新节点初始主问题
// -----------------------------------------------------------------------------
// 功能: 构建考虑分支约束的初始主问题并求解
//
// 参数:
//   Values      - 全局参数
//   Lists       - 全局列表
//   Env_MP      - CPLEX 环境
//   Model_MP    - 主问题模型
//   Obj_MP      - 目标函数对象
//   Cons_MP     - 约束数组
//   Vars_MP     - 变量数组
//   this_node   - 当前新节点
//   parent_node - 父节点 (用于获取分支变量信息)
//
// 返回值:
//   true  = 问题可行
//   false = 问题不可行 (节点需剪枝)
// -----------------------------------------------------------------------------
bool SolveNewNodeFirstMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_MP,
	IloNumVarArray& Vars_MP,
	Node& this_node,
	Node& parent_node) {

	// =========================================================================
	// 问题规模
	// =========================================================================
	int K_num = this_node.Y_cols_list.size();  // Y 变量数 (母板切割模式)
	int P_num = this_node.X_cols_list.size();  // X 变量数 (条带切割模式)
	int J_num = Values.strip_types_num;        // 条带类型数
	int N_num = Values.item_types_num;         // 子件类型数
	int all_cols_num = K_num + P_num;
	int all_rows_num = J_num + N_num;

	// =========================================================================
	// 第一步: 构建约束
	// =========================================================================
	IloNumArray con_min(Env_MP);
	IloNumArray con_max(Env_MP);

	for (int row = 0; row < J_num + N_num; row++) {
		if (row < J_num) {
			// ----- 条带平衡约束: >= 0 -----
			con_min.add(IloNum(0));
			con_max.add(IloNum(IloInfinity));
		}
		if (row >= J_num) {
			// ----- 子件需求约束: >= demand -----
			int row_pos = row - J_num;
			double demand_val = Lists.all_item_types_list[row_pos].demand;
			con_min.add(IloNum(demand_val));
			con_max.add(IloNum(IloInfinity));
		}
	}

	Cons_MP = IloRangeArray(Env_MP, con_min, con_max);
	Model_MP.add(Cons_MP);

	con_min.end();
	con_max.end();

	// =========================================================================
	// 第二步: 构建 Y 变量 (母板切割模式)
	// =========================================================================
	for (int col = 0; col < K_num; col++) {
		// ----- 设置目标函数系数 -----
		IloNum obj_para = 1;
		IloNumColumn CplexCol = Obj_MP(obj_para);

		// ----- 设置约束系数 -----
		for (int row = 0; row < all_rows_num; row++) {
			IloNum row_para = this_node.Y_cols_list[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		string Y_name = "Y_" + to_string(col + 1);

		// =====================================================================
		// Case 1: 当前分支变量
		// =====================================================================
		// 固定为 floor (左子节点) 或 ceil (右子节点)
		// =====================================================================
		if (col == parent_node.var_to_branch_idx) {
			IloNum var_min = this_node.var_to_branch_final;
			IloNum var_max = this_node.var_to_branch_final;
			IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
			Vars_MP.add(Var_Y);
		}
		// =====================================================================
		// Case 2: 非当前分支变量
		// =====================================================================
		else {
			int branched_vars_num = parent_node.branched_int_list.size();
			bool find_flag = 0;

			// ----- Case 2.1: 检查是否为已分支变量 -----
			for (int index = 0; index < branched_vars_num; index++) {
				int branched_idx = parent_node.branched_idx_list[index];
				if (col == branched_idx) {
					// 固定为之前确定的整数值
					IloNum var_min = parent_node.branched_int_list[index];
					IloNum var_max = parent_node.branched_int_list[index];
					IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
					Vars_MP.add(Var_Y);

					find_flag = 1;
					break;
				}
			}

			// ----- Case 2.2: 未分支变量 -----
			if (find_flag == 0) {
				// 保持 [0, +inf) 范围
				IloNum var_min = 0;
				IloNum var_max = IloInfinity;
				IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
				Vars_MP.add(Var_Y);
			}
		}

		CplexCol.end();
	}

	// =========================================================================
	// 第三步: 构建 X 变量 (条带切割模式)
	// =========================================================================
	for (int col = 0; col < P_num; col++) {
		// ----- 设置目标函数系数 -----
		IloNum obj_para = 0;
		IloNumColumn CplexCol = Obj_MP(obj_para);

		// ----- 设置约束系数 -----
		for (int row = 0; row < all_rows_num; row++) {
			IloNum row_para = this_node.X_cols_list[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		string X_name = "X_" + to_string(col + 1);

		// =====================================================================
		// Case 1: 当前分支变量
		// =====================================================================
		if (col + K_num == parent_node.var_to_branch_idx) {
			IloNum var_min = this_node.var_to_branch_final;
			IloNum var_max = this_node.var_to_branch_final;
			IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, X_name.c_str());
			Vars_MP.add(Var_X);
		}
		// =====================================================================
		// Case 2: 非当前分支变量
		// =====================================================================
		else {
			int branched_vars_num = parent_node.branched_int_list.size();
			bool find_flag = 0;

			// ----- Case 2.1: 检查是否为已分支变量 -----
			for (int pos = 0; pos < branched_vars_num; pos++) {
				int branched_idx = parent_node.branched_idx_list[pos];
				if (col == branched_idx) {
					IloNum var_min = parent_node.branched_int_list[pos];
					IloNum var_max = parent_node.branched_int_list[pos];
					IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, X_name.c_str());
					Vars_MP.add(Var_X);

					find_flag = 1;
					break;
				}
			}

			// ----- Case 2.2: 未分支变量 -----
			if (find_flag == 0) {
				IloNum var_min = 0;
				IloNum var_max = IloInfinity;
				IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, X_name.c_str());
				Vars_MP.add(Var_X);
			}
		}

		CplexCol.end();
	}

	// =========================================================================
	// 第四步: 求解主问题
	// =========================================================================
	cout << "[节点_" << this_node.index << "] 求解初始主问题...\n";
	IloCplex MP_cplex(Env_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.setOut(Env_MP.getNullStream());
	MP_cplex.exportModel("New Node First Master Problem.lp");
	bool MP_flag = MP_cplex.solve();

	if (MP_flag == 0) {
		// ----- 不可行: 标记剪枝 -----
		this_node.node_pruned_flag = 1;
		cout << "[节点_" << this_node.index << "] 初始主问题不可行, 需剪枝\n";
	}
	else {
		// ----- 可行: 输出结果并提取对偶价格 -----
		cout << "[节点_" << this_node.index << "] 初始主问题可行, 目标值 = "
		     << fixed << setprecision(4) << MP_cplex.getValue(Obj_MP) << "\n";
		cout.unsetf(ios::fixed);

		// ----- 统计非零解 -----
		int Y_fsb_num = 0;
		int X_fsb_num = 0;
		double soln_val = -1;

		for (int col = 0; col < K_num; col++) {
			soln_val = MP_cplex.getValue(Vars_MP[col]);
			if (soln_val > 0) Y_fsb_num++;
		}

		for (int col = K_num; col < K_num + P_num; col++) {
			soln_val = MP_cplex.getValue(Vars_MP[col]);
			if (soln_val > 0) X_fsb_num++;
		}

		cout << "[节点_" << this_node.index << "] 非零解: Y=" << Y_fsb_num << ", X=" << X_fsb_num << "\n";

		// =====================================================================
		// 提取对偶价格 (用于子问题定价)
		// =====================================================================
		this_node.dual_prices_list.clear();

		double dual_val = -1;
		for (int row = 0; row < J_num; row++) {
			dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) {
				dual_val = 0;
			}
			this_node.dual_prices_list.push_back(dual_val);
		}

		for (int row = J_num; row < J_num + N_num; row++) {
			dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) {
				dual_val = 0;
			}
			this_node.dual_prices_list.push_back(dual_val);
		}
	}

	MP_cplex.end();
	return MP_flag;
}
