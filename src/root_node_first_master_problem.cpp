// =============================================================================
// root_node_first_master_problem.cpp - 根节点初始主问题
// =============================================================================
//
// 功能: 构建并求解根节点的初始受限主问题 (Restricted Master Problem)
//
// 主问题公式:
//   Min  sum(Y_k)                           (最小化母板使用数)
//   s.t. C*Y + D*X >= 0                     (条带平衡约束)
//        B*X >= demand                       (子件需求约束)
//        Y, X >= 0                           (非负约束)
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveRootNodeFirstMasterProblem - 求解根节点初始主问题
// -----------------------------------------------------------------------------
// 功能: 使用 CPLEX 构建并求解初始主问题, 获取对偶价格
// 返回值: true = 可行, false = 不可行
// -----------------------------------------------------------------------------
bool SolveRootNodeFirstMasterProblem(
	All_Values& Values,
	All_Lists& Lists,
	IloEnv& Env_MP,
	IloModel& Model_MP,
	IloObjective& Obj_MP,
	IloRangeArray& Cons_MP,
	IloNumVarArray& Vars_MP,
	Node& root_node) {

	int K_num = root_node.Y_cols_list.size();  // Y 变量数 (母板模式)
	int P_num = root_node.X_cols_list.size();  // X 变量数 (条带模式)
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

		// 添加约束系数
		for (int row = 0; row < J_num + N_num; row++) {
			IloNum row_para = root_node.model_matrix[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		string Y_name = "Y_" + to_string(col + 1);
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
		Vars_MP.add(Var_Y);

		CplexCol.end();
	}

	// =========================================================================
	// 构建 X 变量 (条带切割模式)
	// =========================================================================
	for (int col = K_num; col < K_num + P_num; col++) {
		IloNum obj_para = 0;  // 目标系数 = 0
		IloNumColumn CplexCol = Obj_MP(obj_para);

		// 添加约束系数
		for (int row = 0; row < J_num + N_num; row++) {
			IloNum row_para = root_node.model_matrix[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		string X_name = "X_" + to_string(col + 1 - K_num);
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, X_name.c_str());
		Vars_MP.add(Var_X);

		CplexCol.end();
	}

	// =========================================================================
	// 求解主问题
	// =========================================================================
	printf("\n///////////////// MP_1 CPLEX solving START /////////////////\n");
	IloCplex MP_cplex(Model_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.exportModel("The First Master Problem.lp");
	bool MP_flag = MP_cplex.solve();
	printf("\n///////////////// MP_1 CPLEX solving OVER /////////////////\n\n");

	if (MP_flag == 0) {
		printf("\n\t MP_1 is NOT FEASIBLE\n");
	} else {
		printf("\n\t MP_1 is FEASIBLE\n");

		// ----- 输出解信息 -----
		int sum_vars = 0;
		int Y_fsb_num = 0;
		int X_fsb_num = 0;

		printf("\n\t Y Solns (stock cutting patterns):\n\n");
		for (int col = 0; col < K_num; col++) {
			double soln_val = MP_cplex.getValue(Vars_MP[col]);
			if (soln_val > 0) {
				printf("\t var_Y_%d = %f\n", col + 1, soln_val);
				Y_fsb_num++;
				sum_vars++;
			}
		}

		printf("\n\t X Solns (strip cutting patterns):\n\n");
		for (int col = K_num; col < K_num + P_num; col++) {
			double soln_val = MP_cplex.getValue(Vars_MP[col]);
			if (soln_val > 0) {
				printf("\t var_X_%d = %f\n", col + 1 - K_num, soln_val);
				X_fsb_num++;
				sum_vars++;
			}
		}

		// ----- 提取对偶价格 -----
		root_node.dual_prices_list.clear();

		printf("\n\t strip cons dual: \n\n");
		for (int row = 0; row < J_num; row++) {
			double dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) {
				dual_val = 0;
			}
			root_node.dual_prices_list.push_back(dual_val);
			printf("\t dual_r_%d = %f\n", row + 1, dual_val);
		}

		printf("\n\t item cons dual: \n\n");
		for (int row = J_num; row < J_num + N_num; row++) {
			double dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) {
				dual_val = 0;
			}
			root_node.dual_prices_list.push_back(dual_val);
			printf("\t dual_r_%d = %f\n", row + 1, dual_val);
		}

		// ----- 输出统计信息 -----
		printf("\n\t Node_%d MP-1:\n", root_node.index);
		printf("\n\t Lower Bound = %f", MP_cplex.getValue(Obj_MP));
		printf("\n\t Sum of all solns = %d", sum_vars);
		printf("\n\t Number of all solns = %d", K_num + P_num);
		printf("\n\t Number of all fsb-solns = %d", Y_fsb_num + X_fsb_num);
		printf("\n\t Number of Y fsb-solns = %d", Y_fsb_num);
		printf("\n\t Number of X fsb-solns = %d", X_fsb_num);
	}

	cout << endl;
	return MP_flag;
}
