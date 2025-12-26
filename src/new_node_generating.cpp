// =============================================================================
// new_node_generating.cpp - 新节点生成模块
// =============================================================================
//
// 功能: 实现分支定界树的节点选择和新节点生成
//
// 模块职责:
//   1. ChooseNodeToBranch - 从待处理节点中选择下一个分支节点
//   2. GenerateNewNode    - 基于父节点创建新的子节点
//
// 节点状态:
//   - node_branched_flag: 0=未分支, 1=已分支
//   - node_pruned_flag:   0=未剪枝, 1=已剪枝
//
// 分支搜索状态 (branch_status):
//   - 1: 继续分支左子节点 (变量固定为 floor)
//   - 2: 继续分支右子节点 (变量固定为 ceil)
//   - 3: 回溯搜索之前生成的未处理节点
//
// 分支定界树结构示例:
//
//                    [根节点]
//                   /        \
//            [左子节点]    [右子节点]
//            y_k <= floor   y_k >= ceil
//           /      \       /      \
//         ...      ...   ...      ...
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// ChooseNodeToBranch - 选择待分支节点
// -----------------------------------------------------------------------------
// 功能: 根据当前搜索状态选择下一个要分支的节点
//
// 选择策略:
//   1. 如果 branch_status = 1 或 2:
//      - 继续深度优先搜索, 选择刚生成的子节点
//   2. 如果 branch_status = 3:
//      - 回溯模式, 从节点列表中寻找未处理节点
//      - 优先选择 LB < optimal_LB 的节点
//
// 参数:
//   Values      - 全局参数
//   Lists       - 全局列表
//   parent_node - 输出: 选中的待分支节点
//
// 返回值:
//   0 = 无可分支节点 (搜索完成)
//   1 = 找到待分支节点
// -----------------------------------------------------------------------------
int ChooseNodeToBranch(All_Values& Values, All_Lists& Lists, Node& parent_node) {

	int parent_branch_flag = -1;
	int pos = -1;
	int all_nodes_num = Lists.all_nodes_list.size();

	// =========================================================================
	// 情况 1: 回溯搜索 (branch_status = 3)
	// =========================================================================
	// 遍历所有节点, 寻找未分支且未剪枝的候选节点
	// =========================================================================
	if (Values.branch_status == 3) {
		for (int k = 0; k < all_nodes_num; k++) {
			if (Lists.all_nodes_list[k].node_branched_flag != 1 &&
				Lists.all_nodes_list[k].node_pruned_flag != 1) {
				// ----- 检查是否有剪枝条件 -----
				if (Lists.all_nodes_list[k].LB < Values.optimal_LB) {
					// 下界优于当前最优, 保留该节点
					pos = k;
				}
				else {
					// 下界不优, 执行剪枝
					int temp_idx = Lists.all_nodes_list[k].index;
					cout << "[节点选择] 节点_" << temp_idx << " 需剪枝\n";
					Lists.all_nodes_list[k].node_pruned_flag = 1;
				}
			}
		}
	}

	// =========================================================================
	// 情况 2: 深度优先搜索 (branch_status = 1 或 2)
	// =========================================================================
	// 根据当前位置选择刚生成的子节点
	// =========================================================================
	if (Values.branch_status != 3) {
		if (Values.root_flag == 1) {
			// ----- 父节点是根节点 -----
			// 节点列表结构: [..., 右子节点, 左子节点]
			if (Values.branch_status == 1) {
				pos = all_nodes_num - 1;  // 选择左子节点
			}
			if (Values.branch_status == 2) {
				pos = all_nodes_num - 2;  // 选择右子节点
			}
		}

		if (Values.root_flag != 1) {
			// ----- 父节点不是根节点 -----
			if (Values.fathom_flag == 1) {
				// 父节点是其父节点的左子节点
				if (Values.branch_status == 1) {
					pos = all_nodes_num - 2;
				}
				if (Values.branch_status == 2) {
					pos = all_nodes_num - 3;
				}
			}
			if (Values.fathom_flag == 2) {
				// 父节点是其父节点的右子节点
				if (Values.branch_status == 1) {
					pos = all_nodes_num - 1;
				}
				if (Values.branch_status == 2) {
					pos = all_nodes_num - 2;
				}
			}
		}
	}

	// =========================================================================
	// 返回结果
	// =========================================================================
	if (pos == -1) {
		parent_branch_flag = 0;
		cout << "[节点选择] 无可分支节点\n";
	}
	else {
		parent_branch_flag = 1;
		parent_node = Lists.all_nodes_list[pos];
		parent_node.LB = 1;
		cout << "[节点选择] 待分支节点: 节点_" << parent_node.index << "\n";
	}

	return parent_branch_flag;
}

// -----------------------------------------------------------------------------
// GenerateNewNode - 生成新的子节点
// -----------------------------------------------------------------------------
// 功能: 基于父节点信息创建新的子节点, 继承并更新分支约束
//
// 继承内容:
//   1. 模型矩阵 (model_matrix)
//   2. Y 列集合 (母板切割模式)
//   3. X 列集合 (条带切割模式)
//   4. 已分支变量索引列表
//
// 新增内容:
//   1. 当前分支变量的固定值 (floor 或 ceil)
//   2. 节点索引和父节点关系
//
// 参数:
//   Values      - 全局参数
//   Lists       - 全局列表
//   new_node    - 输出: 新生成的子节点
//   parent_node - 父节点
// -----------------------------------------------------------------------------
void GenerateNewNode(All_Values& Values, All_Lists& Lists, Node& new_node, Node& parent_node) {

	// =========================================================================
	// 初始化节点基本信息
	// =========================================================================
	int all_nodes_num = Lists.all_nodes_list.size();
	new_node.index = all_nodes_num + 1;
	new_node.LB = -1;

	// ----- 记录父节点关系 -----
	new_node.parent_index = parent_node.index;
	new_node.parent_branching_flag = Values.branch_status;
	new_node.parent_var_to_branch_val = parent_node.var_to_branch_soln;

	// ----- 输出分支信息 -----
	string branch_type = (Values.branch_status == 1) ? "左" : "右";
	cout << "[节点生成] 节点_" << new_node.index << " (父节点_" << parent_node.index << " 的" << branch_type << "子节点)\n";

	// ----- 初始化分支变量信息 -----
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

	// =========================================================================
	// 复制 Y 列集合 (母板切割模式)
	// =========================================================================
	int K_num = parent_node.Y_cols_list.size();
	for (int col = 0; col < K_num; col++) {
		vector<double> temp_col;
		for (int row = 0; row < all_rows_num; row++) {
			double val = parent_node.Y_cols_list[col][row];
			temp_col.push_back(val);
		}
		new_node.Y_cols_list.push_back(temp_col);
	}

	// =========================================================================
	// 复制 X 列集合 (条带切割模式)
	// =========================================================================
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
	// 复制已分支变量索引列表
	// =========================================================================
	int branched_vars_num = parent_node.branched_idx_list.size();
	for (int col = 0; col < branched_vars_num; col++) {
		int temp_idx = parent_node.branched_idx_list[col];
		new_node.branched_idx_list.push_back(temp_idx);
	}

	// =========================================================================
	// 设置当前分支变量的固定值
	// =========================================================================
	// 左子节点: 变量 <= floor(分数值)
	// 右子节点: 变量 >= ceil(分数值)
	// =========================================================================
	if (Values.branch_status == 1) {
		new_node.var_to_branch_final = parent_node.var_to_branch_floor;
	}
	if (Values.branch_status == 2) {
		new_node.var_to_branch_final = parent_node.var_to_branch_ceil;
	}

	// =========================================================================
	// 更新已分支变量值列表
	// =========================================================================
	double final_int_val = new_node.var_to_branch_final;

	if (branched_vars_num <= 1) {
		// 根节点的直接子节点
		new_node.branched_int_list.push_back(final_int_val);
	}
	else {
		// 其他节点: 继承父节点的历史值并添加新值
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
}
