// =============================================================================
// new_node_generating.cpp - 新节点生成模块
// =============================================================================
//
// 功能: 实现分支定界中的节点选择和新节点生成
//
// 节点选择策略:
//   - branch_status = 3: 搜索之前生成的未分支未剪枝节点
//   - branch_status = 1/2: 继续分支当前父节点的左/右子节点
//
// 新节点生成:
//   - 继承父节点的模型矩阵和列集合
//   - 记录分支变量的固定值 (floor 或 ceil)
//   - 维护已分支变量的历史记录
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// ChooseNodeToBranch - 选择待分支节点
// -----------------------------------------------------------------------------
// 功能: 根据当前搜索状态选择下一个要分支的节点
// 参数:
//   Values      - 全局参数
//   Lists       - 全局列表
//   parent_node - 输出: 选中的父节点
// 返回值:
//   0 = 无可分支节点
//   1 = 找到待分支节点
// -----------------------------------------------------------------------------
int ChooseNodeToBranch(All_Values& Values, All_Lists& Lists, Node& parent_node) {

	int parent_branch_flag = -1;
	int pos = -1;
	int all_nodes_num = Lists.all_nodes_list.size();

	if (Values.branch_status == 3) {
		// 搜索之前生成的未分支未剪枝节点
		for (int k = 0; k < all_nodes_num; k++) {
			if (Lists.all_nodes_list[k].node_branched_flag != 1 &&
				Lists.all_nodes_list[k].node_pruned_flag != 1) {
				// 未分支且未剪枝
				if (Lists.all_nodes_list[k].LB < Values.optimal_LB) {
					pos = k;
					cout << endl;
				}
				else {
					// 下界不优, 剪枝
					int temp_idx = Lists.all_nodes_list[k].index;
					printf("\n\t Node_%d has to be pruned\n", temp_idx);
					Lists.all_nodes_list[k].node_pruned_flag = 1;
				}
			}
		}
	}

	if (Values.branch_status != 3) {
		// 继续分支当前父节点
		if (Values.root_flag == 1) {
			// 父节点是根节点
			if (Values.branch_status == 1) {
				pos = all_nodes_num - 1;  // 左子节点
			}
			if (Values.branch_status == 2) {
				pos = all_nodes_num - 2;  // 右子节点
			}
		}

		if (Values.root_flag != 1) {
			// 父节点不是根节点
			if (Values.fathom_flag == 1) {
				// 父节点是左子节点
				if (Values.branch_status == 1) {
					pos = all_nodes_num - 2;
				}
				if (Values.branch_status == 2) {
					pos = all_nodes_num - 3;
				}
			}
			if (Values.fathom_flag == 2) {
				// 父节点是右子节点
				if (Values.branch_status == 1) {
					pos = all_nodes_num - 1;
				}
				if (Values.branch_status == 2) {
					pos = all_nodes_num - 2;
				}
			}
		}
	}

	if (pos == -1) {
		parent_branch_flag = 0;
		printf("\n\t No Node to branch! \n");
	}
	else {
		parent_branch_flag = 1;
		parent_node = Lists.all_nodes_list[pos];
		parent_node.LB = 1;
		printf("\n\t The Node to branch is Node_%d\n", parent_node.index);
	}

	return parent_branch_flag;
}

// -----------------------------------------------------------------------------
// GenerateNewNode - 生成新的子节点
// -----------------------------------------------------------------------------
// 功能: 基于父节点信息创建新的子节点
// 参数:
//   Values      - 全局参数
//   Lists       - 全局列表
//   new_node    - 输出: 新生成的子节点
//   parent_node - 父节点
// -----------------------------------------------------------------------------
void GenerateNewNode(All_Values& Values, All_Lists& Lists, Node& new_node, Node& parent_node) {

	int all_nodes_num = Lists.all_nodes_list.size();
	new_node.index = all_nodes_num + 1;
	new_node.LB = -1;

	// 输出分支信息
	if (Values.branch_status == 1) {
		printf("\n\t Node_%d is the LEFT branch of Node_%d	\n", new_node.index, parent_node.index);
	}
	if (Values.branch_status == 2) {
		printf("\n\t Node_%d is the RIGHT branch of Node_%d	\n", new_node.index, parent_node.index);
	}

	// 记录父节点信息
	new_node.parent_index = parent_node.index;
	new_node.parent_branching_flag = Values.branch_status;
	new_node.parent_var_to_branch_val = parent_node.var_to_branch_soln;

	printf("\n###########################################\n");
	printf("###########################################\n");
	printf("################## NEW NODE_%d ##################\n", new_node.index);
	printf("###########################################\n");
	printf("###########################################\n\n");

	// 初始化分支变量信息
	new_node.var_to_branch_idx = -1;
	new_node.var_to_branch_soln = -1;
	new_node.var_to_branch_floor = -1;
	new_node.var_to_branch_ceil = -1;
	new_node.var_to_branch_final = -1;

	// =========================================================================
	// 复制父节点的模型矩阵
	// =========================================================================
	int all_cols_num = parent_node.model_matrix.size();
	int all_rows_num = parent_node.model_matrix[0].size();

	for (int col = 0; col < all_cols_num; col++) {
		vector<double> temp_col;
		for (int row = 0; row < all_rows_num; row++) {
			double val = parent_node.model_matrix[col][row];
			temp_col.push_back(val);
		}
		new_node.model_matrix.push_back(temp_col);
	}

	// 复制 Y 列集合
	int K_num = parent_node.Y_cols_list.size();
	for (int col = 0; col < K_num; col++) {
		vector<double> temp_col;
		for (int row = 0; row < all_rows_num; row++) {
			double val = parent_node.Y_cols_list[col][row];
			temp_col.push_back(val);
		}
		new_node.Y_cols_list.push_back(temp_col);
	}

	// 复制 X 列集合
	int P_num = parent_node.X_cols_list.size();
	for (int col = 0; col < P_num; col++) {
		vector<double> temp_col;
		for (int row = 0; row < all_rows_num; row++) {
			double val = parent_node.X_cols_list[col][row];
			temp_col.push_back(val);
		}
		new_node.X_cols_list.push_back(temp_col);
	}

	// =========================================================================
	// 复制已分支变量的索引列表
	// =========================================================================
	int branched_vars_num = parent_node.branched_idx_list.size();
	for (int col = 0; col < branched_vars_num; col++) {
		int temp_idx = parent_node.branched_idx_list[col];
		new_node.branched_idx_list.push_back(temp_idx);
	}

	// =========================================================================
	// 设置当前分支变量的固定值
	// =========================================================================
	if (Values.branch_status == 1) {
		// 左子节点: 取 floor
		new_node.var_to_branch_final = parent_node.var_to_branch_floor;
	}
	if (Values.branch_status == 2) {
		// 右子节点: 取 ceil
		new_node.var_to_branch_final = parent_node.var_to_branch_ceil;
	}

	// 更新已分支变量值列表
	double final_int_val = new_node.var_to_branch_final;
	if (branched_vars_num <= 1) {
		// 根节点的左/右子节点
		new_node.branched_int_list.push_back(final_int_val);
	}
	else {
		// 其他节点
		for (int col = 0; col < branched_vars_num - 1; col++) {
			double val = parent_node.branched_int_list[col];
			new_node.branched_int_list.push_back(val);
		}
		new_node.branched_int_list.push_back(final_int_val);
	}

	// =========================================================================
	// 清理临时列表
	// =========================================================================
	new_node.all_solns_val_list.clear();
	new_node.dual_prices_list.clear();
	new_node.new_Y_col.clear();
	new_node.new_X_cols_list.clear();

	cout << endl;
}
