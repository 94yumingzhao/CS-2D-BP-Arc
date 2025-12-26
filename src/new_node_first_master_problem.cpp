// =============================================================================
// new_node_first_master_problem.cpp - 新节点初始主问题
// =============================================================================
//
// 功能: 构建并求解新节点的初始受限主问题
//
// 变量边界处理:
//   Case 1: 当前分支变量 - 固定为 floor 或 ceil 值
//   Case 2.1: 已分支变量 - 固定为之前确定的整数值
//   Case 2.2: 未分支变量 - 保持 [0, +inf) 范围
//
// 模型矩阵结构:
//                    模式列 (Pattern Columns)
//   --------------------------------------------------
//   |      K_num (Y)     |      P_num (X)     |
//   |   母板切割模式   |   条带切割模式   |
//   --------------------------------------------------
//   |         C          |         D          | J_num |
//   --------------------------------------------------
//   |         0          |         B          | N_num |
//   --------------------------------------------------
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveNewNodeFirstMasterProblem - 求解新节点初始主问题
// -----------------------------------------------------------------------------
// 功能: 构建考虑分支约束的初始主问题并求解
// 参数:
//   Values      - 全局参数
//   Lists       - 全局列表
//   Env_MP      - CPLEX 环境
//   Model_MP    - 主问题模型
//   Obj_MP      - 目标函数
//   Cons_MP     - 约束数组
//   Vars_MP     - 变量数组
//   this_node   - 当前新节点
//   parent_node - 父节点
// 返回值: true = 可行, false = 不可行 (需剪枝)
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

	int K_num = this_node.Y_cols_list.size();  // Y 变量数
	int P_num = this_node.X_cols_list.size();  // X 变量数

	int J_num = Values.strip_types_num;        // 条带类型数
	int N_num = Values.item_types_num;         // 子件类型数

	int all_cols_num = K_num + P_num;
	int all_rows_num = J_num + N_num;

	// =========================================================================
	// 构建约束
	// =========================================================================
	IloNumArray con_min(Env_MP);
	IloNumArray con_max(Env_MP);

	for (int row = 0; row < J_num + N_num; row++) {
		if (row < J_num) {
			// 条带平衡约束: >= 0
			con_min.add(IloNum(0));
			con_max.add(IloNum(IloInfinity));
		}
		if (row >= J_num) {
			// 子件需求约束: >= demand
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
	// 构建 Y 变量 (母板切割模式)
	// =========================================================================
	for (int col = 0; col < K_num; col++) {
		IloNum obj_para = 1;  // 目标系数 = 1
		IloNumColumn CplexCol = Obj_MP(obj_para);

		// 设置约束系数
		for (int row = 0; row < all_rows_num; row++) {
			IloNum row_para = this_node.Y_cols_list[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		string Y_name = "Y_" + to_string(col + 1);

		// ----- Case 1: 当前分支变量 -----
		if (col == parent_node.var_to_branch_idx) {
			IloNum var_min = this_node.var_to_branch_final;
			IloNum var_max = this_node.var_to_branch_final;
			IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
			Vars_MP.add(Var_Y);
			printf("\n\t Y_var_%d is set as %f, to be branched", col + 1, var_min);
		}
		// ----- Case 2: 非当前分支变量 -----
		else {
			int branched_vars_num = parent_node.branched_int_list.size();
			bool find_flag = 0;

			// Case 2.1: 检查是否为已分支变量
			for (int index = 0; index < branched_vars_num; index++) {
				int branched_idx = parent_node.branched_idx_list[index];
				if (col == branched_idx) {
					IloNum var_min = parent_node.branched_int_list[index];
					IloNum var_max = parent_node.branched_int_list[index];
					IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
					Vars_MP.add(Var_Y);
					printf("\n\t Y_var_%d is set as %f, branched", col + 1, var_min);

					find_flag = 1;
					break;
				}
			}

			// Case 2.2: 未分支变量
			if (find_flag == 0) {
				IloNum var_min = 0;
				IloNum var_max = IloInfinity;
				IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
				Vars_MP.add(Var_Y);
			}
		}

		CplexCol.end();
	}

	// =========================================================================
	// 构建 X 变量 (条带切割模式)
	// =========================================================================
	for (int col = 0; col < P_num; col++) {
		IloNum obj_para = 0;  // 目标系数 = 0
		IloNumColumn CplexCol = Obj_MP(obj_para);

		// 设置约束系数
		for (int row = 0; row < all_rows_num; row++) {
			IloNum row_para = this_node.X_cols_list[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		string X_name = "X_" + to_string(col + 1);

		// ----- Case 1: 当前分支变量 -----
		if (col + K_num == parent_node.var_to_branch_idx) {
			IloNum var_min = this_node.var_to_branch_final;
			IloNum var_max = this_node.var_to_branch_final;
			IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, X_name.c_str());
			Vars_MP.add(Var_X);
			printf("\n\t X_var_%d is set as %f, to be branched", col + 1, var_min);
		}
		// ----- Case 2: 非当前分支变量 -----
		else {
			int branched_vars_num = parent_node.branched_int_list.size();
			bool find_flag = 0;

			// Case 2.1: 检查是否为已分支变量
			for (int pos = 0; pos < branched_vars_num; pos++) {
				int branched_idx = parent_node.branched_idx_list[pos];
				if (col == branched_idx) {
					IloNum var_min = parent_node.branched_int_list[pos];
					IloNum var_max = parent_node.branched_int_list[pos];
					IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, X_name.c_str());
					Vars_MP.add(Var_X);

					printf("\n\t x_var_%d is set as %f, branched", col + 1, var_min);

					find_flag = 1;
					break;
				}
			}

			// Case 2.2: 未分支变量
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
	// 求解主问题
	// =========================================================================
	printf("\n\n############# Node_%d MP-1 CPLEX SOLVING START #############\n\n", this_node.index);
	IloCplex MP_cplex(Env_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.exportModel("New Node First Master Problem.lp");
	bool MP_flag = MP_cplex.solve();
	printf("\n############# Node_%d MP-1 CPLEX SOLVING OVER ###############\n\n", this_node.index);

	if (MP_flag == 0) {
		// 不可行, 标记剪枝
		this_node.node_pruned_flag = 1;
		printf("\n\t Node_%d MP-1 is NOT FEASIBLE\n", this_node.index);
		printf("\n\t Node_%d MP-1 has to be pruned\n", this_node.index);
	}
	else {
		printf("\n\t Node_%d MP-1 is FEASIBLE\n", this_node.index);
		printf("\n\t OBJ of Node_%d MP-1 is %f\n\n", this_node.index, MP_cplex.getValue(Obj_MP));

		// ----- 输出解信息 -----
		int sum_vars = 0;
		int Y_fsb_num = 0;
		int X_fsb_num = 0;
		double soln_val = -1;

		printf("\n\t Y Solns (stock cutting patterns):\n\n");
		for (int col = 0; col < K_num; col++) {
			soln_val = MP_cplex.getValue(Vars_MP[col]);
			if (soln_val > 0) {
				Y_fsb_num++;
				sum_vars++;
				printf("\t var_Y_%d = %f\n", col + 1, soln_val);
			}
		}

		printf("\n\t X Solns (strip cutting patterns):\n\n");
		for (int col = K_num; col < K_num + P_num; col++) {
			soln_val = MP_cplex.getValue(Vars_MP[col]);
			if (soln_val > 0) {
				X_fsb_num++;
				sum_vars++;
				printf("\t var_X_%d = %f\n", col + 1 - K_num, soln_val);
			}
		}

		// ----- 输出已分支变量 -----
		printf("\n\t BRANCHED VARS: \n\n");
		int branched_vars_num = this_node.branched_int_list.size();
		for (int k = 0; k < branched_vars_num; k++) {
			printf("\t var_X_%d = %f branched \n",
				this_node.branched_idx_list[k] + 1, this_node.branched_int_list[k]);
		}

		// ----- 提取对偶价格 -----
		this_node.dual_prices_list.clear();

		printf("\n\t strip_type cons dual prices: \n\n");
		double dual_val = -1;
		for (int row = 0; row < J_num; row++) {
			dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) {
				dual_val = 0;
			}
			this_node.dual_prices_list.push_back(dual_val);
			printf("\t dual_r_%d = %f\n", row + 1, dual_val);
		}

		printf("\n\t item_type cons dual prices: \n\n");
		for (int row = J_num; row < J_num + N_num; row++) {
			dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) {
				dual_val = 0;
			}
			this_node.dual_prices_list.push_back(dual_val);
			printf("\t dual_r_%d = %f\n", row + 1, dual_val);
		}

		// ----- 输出统计信息 -----
		printf("\n\t Node_%d MP-1:\n", this_node.index);
		printf("\n\t Lower Bound = %f", MP_cplex.getValue(Obj_MP));
		printf("\n\t Sum of all solns = %d", sum_vars);
		printf("\n\t Number of all solns = %d", K_num + P_num);
		printf("\n\t Number of all fsb-solns = %d", Y_fsb_num + X_fsb_num);
		printf("\n\t Number of Y fsb-solns = %d", Y_fsb_num);
		printf("\n\t Number of X fsb-solns = %d", X_fsb_num);
		printf("\n\t Number of branched-vars = %d\n", branched_vars_num);
	}

	MP_cplex.end();
	return MP_flag;
}
