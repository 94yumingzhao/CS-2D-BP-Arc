// =============================================================================
// root_node_first_master_problem.cpp - 根节点初始主问题
// =============================================================================
//
// 功能: 构建并求解分支定价树根节点的初始受限主问题 (RMP)
//
// 数学模型 (LP 松弛):
//
//   min  sum_{k} y_k                              (最小化母板使用数)
//
//   s.t. sum_{k} c_{jk} * y_k - sum_{p} d_{jp} * x_p >= 0    (条带平衡约束)
//        sum_{p} b_{ip} * x_p >= demand_i                    (子件需求约束)
//        y_k, x_p >= 0                                        (非负约束)
//
// 模型矩阵结构:
//
//   变量:    y_1  y_2  ...  y_K  |  x_1  x_2  ...  x_P
//           ─────────────────────┼─────────────────────
//   约束 1:    c_11 c_12 ... c_1K | -d_11 -d_12 ... -d_1P  >= 0  (条带1)
//   约束 2:    c_21 c_22 ... c_2K | -d_21 -d_22 ... -d_2P  >= 0  (条带2)
//     ...        ...               |       ...              ...
//   约束 J:    c_J1 c_J2 ... c_JK | -d_J1 -d_J2 ... -d_JP  >= 0  (条带J)
//           ─────────────────────┼─────────────────────
//   约束 J+1:   0    0  ...   0   |  b_11  b_12 ...  b_1P  >= d_1 (子件1)
//   约束 J+2:   0    0  ...   0   |  b_21  b_22 ...  b_2P  >= d_2 (子件2)
//     ...        ...               |       ...              ...
//   约束 J+N:   0    0  ...   0   |  b_N1  b_N2 ...  b_NP  >= d_N (子件N)
//
// CPLEX 建模说明:
//   - 使用列式建模 (Column-wise Modeling) 逐列添加变量
//   - 每列包含: 目标系数 + 约束系数向量
//   - 对偶价格用于子问题的定价决策
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveRootNodeFirstMasterProblem - 求解根节点初始主问题
// -----------------------------------------------------------------------------
// 功能: 基于启发式生成的初始模式, 构建并求解第一个受限主问题
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   Env_MP    - CPLEX 环境
//   Model_MP  - 主问题模型
//   Obj_MP    - 目标函数对象
//   Cons_MP   - 约束数组
//   Vars_MP   - 变量数组
//   root_node - 根节点
// 返回值:
//   true  = 问题可行, 成功求解
//   false = 问题不可行 (通常表示初始模式不足)
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

	// =========================================================================
	// 问题规模
	// =========================================================================
	int K_num = root_node.Y_cols_list.size();  // Y 变量数 (母板切割模式)
	int P_num = root_node.X_cols_list.size();  // X 变量数 (条带切割模式)
	int J_num = Values.strip_types_num;        // 条带类型数 (= 子件类型数)
	int N_num = Values.item_types_num;         // 子件类型数
	int all_cols_num = K_num + P_num;          // 总变量数
	int all_rows_num = J_num + N_num;          // 总约束数

	cout << "[主问题] 构建初始主问题 (Y=" << K_num << ", X=" << P_num << ")\n";

	// =========================================================================
	// 第一步: 构建约束
	// =========================================================================
	// 约束结构:
	//   - 前 J 个约束: 条带平衡约束, >= 0
	//   - 后 N 个约束: 子件需求约束, >= demand
	// =========================================================================
	IloNumArray con_min(Env_MP);
	IloNumArray con_max(Env_MP);

	for (int row = 0; row < J_num + N_num; row++) {
		if (row < J_num) {
			// ----- 条带平衡约束 -----
			// 母板产出的条带数 >= 条带模式消耗的条带数
			con_min.add(IloNum(0));
			con_max.add(IloNum(IloInfinity));
		}
		if (row >= J_num) {
			// ----- 子件需求约束 -----
			// 条带模式产出的子件数 >= 子件需求量
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
	// Y_k: 使用第 k 种母板切割模式的数量
	// 目标系数: 1 (每使用一块母板, 目标函数增加 1)
	// 约束系数: model_matrix[k][row]
	// =========================================================================
	for (int col = 0; col < K_num; col++) {
		// ----- 设置目标函数系数 -----
		IloNum obj_para = 1;
		IloNumColumn CplexCol = Obj_MP(obj_para);

		// ----- 设置约束系数 -----
		for (int row = 0; row < J_num + N_num; row++) {
			IloNum row_para = root_node.model_matrix[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		// ----- 创建变量 -----
		string Y_name = "Y_" + to_string(col + 1);
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		IloNumVar Var_Y(CplexCol, var_min, var_max, ILOFLOAT, Y_name.c_str());
		Vars_MP.add(Var_Y);
		CplexCol.end();
	}

	// =========================================================================
	// 第三步: 构建 X 变量 (条带切割模式)
	// =========================================================================
	// X_p: 使用第 p 种条带切割模式的数量
	// 目标系数: 0 (条带模式不直接影响目标函数)
	// 约束系数: model_matrix[K_num + p][row]
	// =========================================================================
	for (int col = K_num; col < K_num + P_num; col++) {
		// ----- 设置目标函数系数 -----
		IloNum obj_para = 0;
		IloNumColumn CplexCol = Obj_MP(obj_para);

		// ----- 设置约束系数 -----
		for (int row = 0; row < J_num + N_num; row++) {
			IloNum row_para = root_node.model_matrix[col][row];
			CplexCol += Cons_MP[row](row_para);
		}

		// ----- 创建变量 -----
		string X_name = "X_" + to_string(col + 1 - K_num);
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		IloNumVar Var_X(CplexCol, var_min, var_max, ILOFLOAT, X_name.c_str());
		Vars_MP.add(Var_X);
		CplexCol.end();
	}

	// =========================================================================
	// 第四步: 求解主问题
	// =========================================================================
	cout << "[主问题] 求解初始主问题...\n";
	IloCplex MP_cplex(Model_MP);
	MP_cplex.extract(Model_MP);
	MP_cplex.setOut(Env_MP.getNullStream());  // 关闭 CPLEX 输出
	MP_cplex.exportModel("The First Master Problem.lp");
	bool MP_flag = MP_cplex.solve();

	if (MP_flag == 0) {
		// ----- 不可行情况 -----
		cout << "[主问题] 初始主问题不可行\n";
	} else {
		// ----- 可行情况: 输出结果并提取对偶价格 -----
		cout << "[主问题] 初始主问题可行, 目标值 = "
		     << fixed << setprecision(4) << MP_cplex.getValue(Obj_MP) << "\n";
		cout.unsetf(ios::fixed);

		// ----- 统计非零解 -----
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

		cout << "[主问题] 非零解: Y=" << Y_fsb_num << ", X=" << X_fsb_num << "\n";

		// =====================================================================
		// 提取对偶价格 (用于子问题定价)
		// =====================================================================
		// 对偶价格含义:
		//   - v_j (条带约束对偶): 增加一单位条带 j 供给的边际价值
		//   - w_i (子件约束对偶): 减少一单位子件 i 需求的边际价值
		// =====================================================================
		root_node.dual_prices_list.clear();

		// ----- 条带约束对偶价格 -----
		for (int row = 0; row < J_num; row++) {
			double dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) dual_val = 0;
			root_node.dual_prices_list.push_back(dual_val);
		}

		// ----- 子件约束对偶价格 -----
		for (int row = J_num; row < J_num + N_num; row++) {
			double dual_val = MP_cplex.getDual(Cons_MP[row]);
			if (dual_val == -0) dual_val = 0;
			root_node.dual_prices_list.push_back(dual_val);
		}
	}

	return MP_flag;
}
