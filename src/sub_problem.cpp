// =============================================================================
// sub_problem.cpp - 子问题求解模块
// =============================================================================
//
// 功能: 实现两阶段切割的定价子问题
//
// 子问题结构:
//   SP1 (宽度背包): 决定条带在母板宽度方向的排列
//       Max  sum(alpha_j * G_j)           (alpha_j = 条带j的对偶价格)
//       s.t. sum(w_j * G_j) <= W          (宽度约束)
//            G_j >= 0, integer            (条带数量非负整数)
//
//   SP2 (长度背包): 决定子件在条带长度方向的排列
//       Max  sum(beta_i * D_i)            (beta_i = 子件i的对偶价格)
//       s.t. sum(l_i * D_i) <= L          (长度约束)
//            D_i >= 0, integer            (子件数量非负整数)
//
// 定价策略:
//   - SP1 reduced cost > 1: 找到改进的母板切割模式, 添加 Y 列
//   - SP1 reduced cost <= 1: 需检查 SP2, 寻找改进的条带切割模式
//   - SP2 目标值 > alpha_j: 找到改进的条带切割模式, 添加 X 列
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveStageOneSubProblem - 求解第一阶段子问题 (宽度背包)
// -----------------------------------------------------------------------------
// 功能: 寻找具有负 reduced cost 的新母板切割模式或条带切割模式
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点
// 返回值:
//   1 = 找到改进列, 0 = 无改进列 (列生成收敛)
// -----------------------------------------------------------------------------
int SolveStageOneSubProblem(All_Values& Values, All_Lists& Lists, Node& this_node) {

	int K_num = this_node.Y_cols_list.size();  // Y 变量数 (母板模式)
	int P_num = this_node.X_cols_list.size();  // X 变量数 (条带模式)

	int J_num = Values.strip_types_num;        // 条带类型数
	int N_num = Values.item_types_num;         // 子件类型数

	int all_cols_num = K_num + P_num;
	int all_rows_num = J_num + N_num;

	int loop_continue_flag = -1;

	// =========================================================================
	// 构建 SP1 模型 (宽度背包)
	// =========================================================================
	IloEnv Env_SP1;
	IloModel Model_SP1(Env_SP1);
	IloNumVarArray Ga_Vars(Env_SP1);  // G_j: 条带 j 的切割数量

	// ----- 创建决策变量 -----
	for (int j = 0; j < J_num; j++) {
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		string var_name = "G_" + to_string(j + 1);
		IloNumVar Var_Ga(Env_SP1, var_min, var_max, ILOINT, var_name.c_str());
		Ga_Vars.add(Var_Ga);
	}

	// ----- 目标函数: Max sum(alpha_j * G_j) -----
	IloExpr obj_sum(Env_SP1);
	for (int j = 0; j < J_num; j++) {
		double val = this_node.dual_prices_list[j];  // 条带约束对偶价格
		obj_sum += val * Ga_Vars[j];
	}
	IloObjective Obj_SP1 = IloMaximize(Env_SP1, obj_sum);
	Model_SP1.add(Obj_SP1);
	obj_sum.end();

	// ----- 宽度约束: sum(w_j * G_j) <= W -----
	IloExpr con_sum(Env_SP1);
	for (int j = 0; j < J_num; j++) {
		double val = Lists.all_strip_types_list[j].width;
		con_sum += val * Ga_Vars[j];
	}
	Model_SP1.add(con_sum <= Values.stock_width);
	con_sum.end();

	// =========================================================================
	// 求解 SP1
	// =========================================================================
	printf("\n///////////////// SP_%d CPLEX solving START /////////////////\n\n", this_node.iter);
	IloCplex Cplex_SP1(Env_SP1);
	Cplex_SP1.extract(Model_SP1);
	bool SP1_flag = Cplex_SP1.solve();
	printf("\n///////////////// SP_%d CPLEX solving OVER /////////////////\n\n", this_node.iter);

	if (SP1_flag == 0) {
		printf("\n\t SP_%d is NOT FEASIBLE\n", this_node.iter);
	}
	else {
		printf("\n\n\t SP_%d is FEASIBLE\n", this_node.iter);

		printf("\n\n\t Obj = %f\n", Cplex_SP1.getValue(Obj_SP1));
		double SP1_obj_val = Cplex_SP1.getValue(Obj_SP1);

		// ----- 提取 SP1 解并构建新 Y 列 -----
		this_node.new_Y_col.clear();
		printf("\n\t SP_%d VARS:\n\n", this_node.iter);
		for (int j = 0; j < J_num; j++) {
			double soln_val = Cplex_SP1.getValue(Ga_Vars[j]);
			if (soln_val == -0) {
				soln_val = 0;
			}
			printf("\t var_G_%d = %f\n", j + 1, soln_val);
			this_node.new_Y_col.push_back(soln_val);
		}

		// ----- 输出新列 (条带部分 + 子件部分) -----
		printf("\n\t SP_%d new col:\n\n", this_node.iter);
		for (int j = 0; j < J_num + N_num; j++) {
			if (j < J_num) {
				double soln_val = Cplex_SP1.getValue(Ga_Vars[j]);
				if (soln_val == -0) {
					soln_val = 0;
				}
				printf("\t row_%d = %f\n", j + 1, soln_val);
			}
			else {
				// Y 列的子件部分全为 0
				printf("\t row_%d = 0\n", j + 1);
				this_node.new_Y_col.push_back(0);
			}
		}

		// =====================================================================
		// 定价决策
		// =====================================================================
		if (SP1_obj_val > 1 + RC_EPS) {
			// SP1 的 reduced cost > 1, 找到改进的母板切割模式
			printf("\n\n\t SP reduced cost = %f > 1,  \n", SP1_obj_val);
			printf("\n\t No need to solve SP2\n");

			this_node.Y_col_flag = 1;
			loop_continue_flag = 1;
		}
		else {
			// SP1 的 reduced cost <= 1, 需要继续求解 SP2
			printf("\n\t SP reduced cost = %f <=1 \n", SP1_obj_val);
			printf("\n\t Continue to solve SP2\n");

			this_node.new_Y_col.clear();
			this_node.new_X_cols_list.clear();
			this_node.Y_col_flag = 0;
			this_node.SP2_obj_val = -1;

			int feasible_flag = 0;
			int SP2_flag = -1;
			int K_num = this_node.Y_cols_list.size();
			int P_num = this_node.X_cols_list.size();

			// ----- 对每种条带类型求解 SP2 -----
			for (int k = 0; k < J_num; k++) {

				SP2_flag = SolveStageTwoSubProblem(Values, Lists, this_node, k + 1);
				if (SP2_flag == 1) {
					double a_val = this_node.dual_prices_list[k];

					// 检查 reduced cost: SP2_obj > alpha_k 则找到改进列
					if (this_node.SP2_obj_val > a_val + RC_EPS) {
						feasible_flag = 1;
						printf("\n\t SP_%d_%d obj = %f > strip con_%d dual = %f:\n",
							this_node.iter, k + 1, this_node.SP2_obj_val, k + 1, a_val);

						// 构建新 X 列
						vector<double> temp_col;

						// 条带部分: 仅第 k 行为 -1 (消耗条带)
						for (int j = 0; j < J_num; j++) {
							if (k == j) {
								temp_col.push_back(-1);  // 消耗一个条带
							}
							else {
								temp_col.push_back(0);
							}
						}

						// 子件部分: SP2 的解 (生产子件)
						for (int i = 0; i < N_num; i++) {
							double soln_val = this_node.SP2_solns_list[i];
							if (soln_val == -0) {
								soln_val = 0;
							}
							temp_col.push_back(soln_val);
						}

						printf("\n\t SP_%d_%d new col:\n\n", this_node.iter, k + 1);
						for (int row = 0; row < J_num + N_num; row++) {
							printf("\t row_%d = %f\n", row + 1, temp_col[row]);
						}
						printf("\n\t Add SP_%d_%d new col to MP\n\n", this_node.iter, k + 1);

						this_node.new_X_cols_list.push_back(temp_col);
						cout << endl;
					}
					else {
						printf("\n\t SP_%d_%d Obj = %f < strip con_%d dual = %f:\n",
							this_node.iter, k + 1, this_node.SP2_obj_val, k + 1, a_val);
						cout << endl;
					}
				}
			}

			if (feasible_flag == 0) {
				// 所有 SP2 都没有找到改进列, 列生成收敛
				printf("\n\t Every SP_%d_* has no new col \n\n", this_node.iter);
				printf("\n\t Column Generation loop break\n");
				cout << endl;
			}

			loop_continue_flag = feasible_flag;
		}
	}

	// ----- 释放 CPLEX 资源 -----
	Obj_SP1.removeAllProperties();
	Obj_SP1.end();
	Ga_Vars.clear();
	Ga_Vars.end();
	Model_SP1.removeAllProperties();
	Model_SP1.end();
	Env_SP1.removeAllProperties();
	Env_SP1.end();

	return loop_continue_flag;
}

// -----------------------------------------------------------------------------
// SolveStageTwoSubProblem - 求解第二阶段子问题 (长度背包)
// -----------------------------------------------------------------------------
// 功能: 为指定条带类型寻找具有负 reduced cost 的新切割模式
// 参数:
//   Values         - 全局参数
//   Lists          - 全局列表
//   this_node      - 当前节点
//   strip_type_idx - 条带类型索引 (1-based)
// 返回值:
//   1 = 求解成功, 0 = 无可行解或不存在
// -----------------------------------------------------------------------------
int SolveStageTwoSubProblem(All_Values& Values, All_Lists& Lists, Node& this_node, int strip_type_idx) {

	printf("\n\t SP_%d_%d strip width = %d\n", this_node.iter, strip_type_idx,
		Lists.all_strip_types_list[strip_type_idx - 1].width);

	int all_cols_num = this_node.model_matrix.size();
	int strip_types_num = Values.strip_types_num;
	int item_types_num = Values.item_types_num;
	int J_num = strip_types_num;
	int N_num = item_types_num;

	int final_return = -1;

	// =========================================================================
	// 构建 SP2 模型 (长度背包)
	// =========================================================================
	IloEnv Env_SP2;
	IloModel Model_SP2(Env_SP2);
	IloNumVarArray De_Vars(Env_SP2);  // D_i: 子件 i 的切割数量

	// ----- 创建决策变量 -----
	for (int i = 0; i < N_num; i++) {
		IloNum var_min = 0;
		IloNum var_max = IloInfinity;
		string var_name = "D_" + to_string(i + 1);
		IloNumVar De_Var(Env_SP2, var_min, var_max, ILOINT, var_name.c_str());
		De_Vars.add(De_Var);
	}

	IloObjective Obj_SP2(Env_SP2);
	IloExpr obj_sum(Env_SP2);
	IloExpr sum_1(Env_SP2);
	double sum_val = 0;
	vector<int> fsb_idx;  // 可行子件索引列表

	// ----- 筛选可放入当前条带的子件 -----
	for (int i = 0; i < N_num; i++) {
		// 检查子件宽度是否小于等于条带宽度
		if (Lists.all_item_types_list[i].width
			<= Lists.all_strip_types_list[strip_type_idx - 1].width) {
			int row_pos = i + N_num;  // 子件约束在对偶价格列表中的位置
			double b_val = this_node.dual_prices_list[row_pos];
			if (b_val > 0) {
				// 仅考虑正对偶价格的子件 (对目标有贡献)
				obj_sum += b_val * De_Vars[i];
				sum_val += b_val;
				double l_val = Lists.all_item_types_list[i].length;
				sum_1 += l_val * De_Vars[i];
				fsb_idx.push_back(i);
			}
		}
	}

	if (sum_val > 0) {
		// 存在可行子件, 构建并求解模型
		Obj_SP2 = IloMaximize(Env_SP2, obj_sum);
		Model_SP2.add(Obj_SP2);
		Model_SP2.add(sum_1 <= Values.stock_length);  // 长度约束
		for (int i = 0; i < N_num; i++) {
			Lists.all_item_types_list[i].width* De_Vars[i] <= Lists.all_strip_types_list[strip_type_idx - 1].length;
		}
		obj_sum.end();
		sum_1.end();
		final_return = 1;
	}
	if (sum_val == 0) {
		// 无可行子件, SP2 不存在
		printf("\n\t SP_%d_%d does NOT EXIST\n", this_node.iter, strip_type_idx);
		sum_1.end();
		final_return = 1;
		Obj_SP2.removeAllProperties();
		Obj_SP2.end();
		De_Vars.clear();
		De_Vars.end();
		Model_SP2.removeAllProperties();
		Model_SP2.end();
		Env_SP2.removeAllProperties();
		Env_SP2.end();
		final_return = 0;
		return final_return;
	}

	// =========================================================================
	// 求解 SP2
	// =========================================================================
	printf("\n///////////////// SP_%d_%d CPLEX solving START /////////////////\n\n", this_node.iter, strip_type_idx);
	IloCplex Cplex_SP2(Env_SP2);
	Cplex_SP2.extract(Model_SP2);
	bool SP2_flag = Cplex_SP2.solve();
	printf("\n///////////////// SP_%d_%d CPLEX solving OVER /////////////////\n\n", this_node.iter, strip_type_idx);

	if (SP2_flag == 0) {
		printf("\n\t SP_%d_%d is NOT FEASIBLE\n", this_node.iter, strip_type_idx);
	}
	else {
		printf("\n\t SP_%d_%d is FEASIBLE\n", this_node.iter, strip_type_idx);

		printf("\n\t Obj = %f\n", Cplex_SP2.getValue(Obj_SP2));
		this_node.SP2_obj_val = Cplex_SP2.getValue(Obj_SP2);

		// ----- 提取 SP2 解 -----
		printf("\n\t SP_%d_%d VARS:\n\n", this_node.iter, strip_type_idx);
		this_node.SP2_solns_list.clear();
		int fsb_num = fsb_idx.size();

		for (int i = 0; i < N_num; i++) {
			int fsb_flag = -1;
			for (int k = 0; k < fsb_num; k++) {
				if (i == fsb_idx[k]) {
					fsb_flag = 1;
					double soln_val = Cplex_SP2.getValue(De_Vars[i]);
					if (soln_val == -0) {
						soln_val = 0;
					}
					printf("\t var_D_%d = %f\n", i + 1, soln_val);
					this_node.SP2_solns_list.push_back(soln_val);
				}
			}
			if (fsb_flag != 1) {
				// 非可行子件, 解为 0
				this_node.SP2_solns_list.push_back(0);
			}
		}
	}

	// ----- 释放 CPLEX 资源 -----
	Obj_SP2.removeAllProperties();
	Obj_SP2.end();
	De_Vars.clear();
	De_Vars.end();
	Model_SP2.removeAllProperties();
	Model_SP2.end();
	Env_SP2.removeAllProperties();
	Env_SP2.end();

	return final_return;
}
