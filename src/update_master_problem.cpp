// =============================================================================
// update_master_problem.cpp - 主问题更新模块
// =============================================================================
//
// 功能: 在列生成过程中动态更新和求解主问题
//
// 模型矩阵结构:
//                    模式列 (Pattern Columns)
//   --------------------------------------------------
//   |      K_num (Y)     |      P_num (X)     |
//   |   母板切割模式   |   条带切割模式   |
//   --------------------------------------------------
//   |                    |                    |       |
//   |         C          |         D          | J_num | 条带平衡约束 >= 0
//   |                    |                    |       |
//   --------------------------------------------------
//   |                    |                    |       |
//   |         0          |         B          | N_num | 子件需求约束 >= demand
//   |                    |                    |       |
//   --------------------------------------------------
//
// 列添加规则:
//   - Y_col_flag = 1: 添加新 Y 列 (母板切割模式), 目标系数 = 1
//   - Y_col_flag = 0: 添加新 X 列 (条带切割模式), 目标系数 = 0
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveUpdateMasterProblem - 添加新列并重新求解主问题
// -----------------------------------------------------------------------------
// 功能: 将子问题找到的改进列添加到主问题, 重新求解并更新对偶价格
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   Env_MP    - CPLEX 环境
//   Model_MP  - 主问题模型
//   Obj_MP    - 目标函数
//   Cons_MP   - 约束数组
//   Vars_MP   - 变量数组
//   this_node - 当前节点
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

	int K_num = this_node.Y_cols_list.size();  // Y 变量数 (母板模式)
	int P_num = this_node.X_cols_list.size();  // X 变量数 (条带模式)

	int J_num = Values.strip_types_num;        // 条带类型数
	int N_num = Values.item_types_num;         // 子件类型数

	int all_cols_num = K_num + P_num;
	int all_rows_num = J_num + N_num;

	// =========================================================================
	// 添加新列到主问题
	// =========================================================================

	if (this_node.Y_col_flag == 1) {
		// ----- 添加新 Y 列 (母板切割模式) -----
		IloNum obj_para = 1;  // 目标系数 = 1 (计入母板使用数)
		IloNumColumn CplexCol = (Obj_MP)(obj_para);

		// 设置约束系数
		for (int row = 0; row < all_rows_num; row++) {
			IloNum row_para = this_node.new_Y_col[row];
			CplexCol += (Cons_MP)[row](row_para);
		}

		// 创建变量并添加到模型
		int cols_num = this_node.Y_cols_list.size();
		string var_name = "Y_" + to_string(cols_num + 1);
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, var_name.c_str());
		(Vars_MP).add(Var_Y);

		CplexCol.end();

		// 更新节点数据结构
		vector<double> new_col;
		double val;
		for (int row = 0; row < all_rows_num; row++) {
			val = this_node.new_Y_col[row];
			new_col.push_back(val);
		}

		this_node.Y_cols_list.push_back(new_col);
		this_node.model_matrix.insert(this_node.model_matrix.begin() + this_node.Y_cols_list.size(), new_col);
		this_node.new_Y_col.clear();
	}

	if (this_node.Y_col_flag == 0) {
		// ----- 添加新 X 列 (条带切割模式) -----
		int new_cols_num = this_node.new_X_cols_list.size();
		for (int col = 0; col < new_cols_num; col++) {

			IloNum obj_para = 0;  // 目标系数 = 0 (不计入目标)
			IloNumColumn CplexCol = (Obj_MP)(obj_para);

			// 设置约束系数
			for (int row = 0; row < all_rows_num; row++) {
				IloNum row_para = this_node.new_X_cols_list[col][row];
				CplexCol += (Cons_MP)[row](row_para);
			}

			// 创建变量并添加到模型
			int old_item_cols_num = this_node.X_cols_list.size();
			string var_name = "X_" + to_string(old_item_cols_num + 1);
			IloNum var_min = 0;
			IloNum var_max = IloInfinity;
			IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, var_name.c_str());
			(Vars_MP).add(Var_X);

			CplexCol.end();

			// 更新节点数据结构
			vector<double> temp_col;
			for (int row = 0; row < all_rows_num; row++) {
				double val = this_node.new_X_cols_list[col][row];
				temp_col.push_back(val);
			}

			this_node.X_cols_list.push_back(temp_col);
			this_node.model_matrix.insert(this_node.model_matrix.end(), temp_col);
		}
		this_node.new_X_cols_list.clear();
	}

	// =========================================================================
	// 求解更新后的主问题
	// =========================================================================
	printf("\n\n///////////////// MP-%d CPLEX solving START /////////////////\n", this_node.iter);
	IloCplex MP_cplex(Model_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.solve();
	printf("\n///////////////// MP-%d CPLEX solving OVER /////////////////\n\n", this_node.iter);
	printf("\n\t Obj = %f\n", MP_cplex.getValue(Obj_MP));

	// ----- 输出解信息 -----
	double sum_vars = 0;
	int Y_fsb_num = 0;
	int X_fsb_num = 0;

	printf("\n\t Y Solns:\n\n");
	for (int col = 0; col < K_num; col++) {
		double soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val > 0) {
			Y_fsb_num++;
			sum_vars = sum_vars + soln_val;
			printf("\t var_y_%d = %f\n", col + 1, soln_val);
		}
	}

	printf("\n\t X Soln:\n\n");
	for (int col = K_num; col < K_num + P_num; col++) {
		double soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val > 0) {
			X_fsb_num++;
			sum_vars = sum_vars + soln_val;
			printf("\t var_x_%d = %f\n", col + 1 - K_num, soln_val);
		}
	}

	// ----- 提取对偶价格 -----
	this_node.dual_prices_list.clear();

	printf("\n\t strip cons dual: \n\n");
	for (int row = 0; row < J_num; row++) {
		double dual_val = MP_cplex.getDual(Cons_MP[row]);
		if (dual_val == -0) {
			dual_val = 0;
		}
		this_node.dual_prices_list.push_back(dual_val);
		printf("\t dual_r_%d = %f\n", row + 1, dual_val);
	}

	printf("\n\t item cons dual: \n\n");
	for (int row = J_num; row < J_num + N_num; row++) {
		double dual_val = MP_cplex.getDual(Cons_MP[row]);
		if (dual_val == -0) {
			dual_val = 0;
		}
		this_node.dual_prices_list.push_back(dual_val);
		printf("\t dual_r_%d = %f\n", row + 1, dual_val);
	}

	// ----- 输出统计信息 -----
	printf("\n\t MP-%d:", this_node.iter);
	printf("\n\t Lower Bound = %f", MP_cplex.getValue(Obj_MP));
	printf("\n\t Sum of all solns = %f", sum_vars);
	printf("\n\t Number of all solns = %d", K_num + P_num);
	printf("\n\t Number of y fsb-solns = %d", Y_fsb_num);
	printf("\n\t Number of x fsb-solns = %d", X_fsb_num);
	printf("\n\t Number of all fsb-solns = %d", Y_fsb_num + X_fsb_num);

	cout << endl;
}

// -----------------------------------------------------------------------------
// SolveFinalMasterProblem - 求解列生成收敛后的最终主问题
// -----------------------------------------------------------------------------
// 功能: 列生成收敛后求解最终主问题, 记录下界和解向量
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   Env_MP    - CPLEX 环境
//   Model_MP  - 主问题模型
//   Obj_MP    - 目标函数
//   Cons_MP   - 约束数组
//   Vars_MP   - 变量数组
//   this_node - 当前节点
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

	int K_num = this_node.Y_cols_list.size();
	int P_num = this_node.X_cols_list.size();
	int N_num = Values.item_types_num;
	int J_num = Values.strip_types_num;
	int all_rows_num = N_num + J_num;
	int all_cols_num = K_num + P_num;

	// =========================================================================
	// 求解最终主问题
	// =========================================================================
	printf("\n\n///////////////// MP_final CPLEX solving START /////////////////\n");
	IloCplex MP_cplex(Model_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.exportModel("Final Master Problem.lp");
	MP_cplex.solve();
	printf("\n///////////////// MP_final CPLEX solving OVER /////////////////\n\n");

	// 记录节点下界
	this_node.LB = MP_cplex.getValue(Obj_MP);
	printf("\n\t OBJ of Node_%d MP-final is %f \n\n", this_node.index, MP_cplex.getValue(Obj_MP));

	// ----- 保存所有变量的解 (包括零解) -----
	for (int col = 0; col < all_cols_num; col++) {
		IloNum soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val == -0) {
			soln_val = 0;
		}
		this_node.all_solns_val_list.push_back(soln_val);
	}

	// ----- 输出解信息 -----
	int sum_vars = 0;
	int Y_fsb_num = 0;
	int X_fsb_num = 0;

	printf("\n\t Y Solns (stock cutting patterns):\n\n");
	for (int col = 0; col < K_num; col++) {
		double soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val > 0) {
			Y_fsb_num++;
			sum_vars++;
			printf("\t var_Y_%d = %f\n", col + 1, soln_val);
		}
	}

	printf("\n\t X Solns (this_strip cutting patterns):\n\n");
	for (int col = K_num; col < K_num + P_num; col++) {
		double soln_val = MP_cplex.getValue(Vars_MP[col]);
		if (soln_val > 0) {
			X_fsb_num++;
			sum_vars++;
			printf("\t var_X_%d = %f\n", col + 1 - K_num, soln_val);
		}
	}

	// ----- 输出已分支的变量 -----
	printf("\n\t BRANCHED VARS: \n\n");
	int branched_cols_num = this_node.branched_int_list.size();
	int var_idx = -1;
	double var_int_val = -1;
	for (int k = 0; k < branched_cols_num; k++) {
		var_idx = this_node.branched_idx_list[k] + 1;
		var_int_val = this_node.branched_int_list[k];
		printf("\t var_x_%d = %f branched \n", var_idx, var_int_val);
	}

	// ----- 输出统计信息 -----
	printf("\n\t Node_%d MP-1:\n", this_node.index);
	printf("\n\t Lower Bound = %f", MP_cplex.getValue(Obj_MP));
	printf("\n\t Sum of all solns = %d", sum_vars);
	printf("\n\t Number of all solns = %d", K_num + P_num);
	printf("\n\t Number of Y fsb-solns = %d", Y_fsb_num);
	printf("\n\t Number of X fsb-solns = %d", X_fsb_num);
	printf("\n\t Number of all fsb-solns = %d\n", Y_fsb_num + X_fsb_num);

	cout << endl;
}
