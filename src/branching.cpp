// =============================================================================
// branching.cpp - 分支决策模块
// =============================================================================
//
// 功能: 实现节点处理和分支变量选择
//
// 主要逻辑:
//   1. FinishNode: 检查节点解的整数性, 更新全局最优解
//   2. ChooseVarToBranch: 选择第一个非整数变量作为分支变量
//
// 剪枝条件:
//   - 节点不可行
//   - 节点下界 >= 当前最优上界
//   - 所有变量均为整数 (叶节点)
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// FinishNode - 完成节点处理
// -----------------------------------------------------------------------------
// 功能: 检查节点解的整数性, 决定下一步操作 (分支或搜索其他节点)
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点
// 返回值:
//   0 = 存在非整数解, 继续分支
//   1 = 全为整数解, 搜索其他节点
// -----------------------------------------------------------------------------
int FinishNode(All_Values& Values, All_Lists& Lists, Node& this_node) {

	// node_int_flag: 0 = 存在非整数解, 1 = 全为整数解
	int node_int_flag = -1;
	// tree_search_flag: 0 = 继续分支, 1 = 搜索其他节点
	int tree_search_flag = -1;

	// 检查整数性并选择分支变量
	node_int_flag = ChooseVarToBranch(Values, Lists, this_node);

	if (this_node.node_pruned_flag == 1) {
		// 节点已被剪枝
		tree_search_flag = 1;
	}
	else {
		if (node_int_flag == 0) {
			// 存在非整数解, 记录分支变量信息
			int var_idx = this_node.var_to_branch_idx;
			this_node.branched_idx_list.push_back(var_idx);

			double var_val = this_node.var_to_branch_soln;
			this_node.branched_solns_ist.push_back(var_val);

			tree_search_flag = 0;
		}

		if (node_int_flag == 1) {
			// 所有非零解均为整数
			if (this_node.index == 1) {
				// 根节点即为整数解
				Values.optimal_LB = this_node.LB;
				printf("\n\t Current Optimal Lower Bound = %f\n", Values.optimal_LB);
			}
			if (this_node.index > 1) {
				// 非根节点
				if (Values.optimal_LB == -1) {
					// 首个整数解节点
					Values.optimal_LB = this_node.LB;
					printf("\n\t Current Optimal Lower Bound = %f\n", Values.optimal_LB);
				}
				else {
					// 已有整数解, 比较并更新
					if (this_node.LB < Values.optimal_LB) {
						Values.optimal_LB = this_node.LB;
						printf("\n\t Current Optimal Lower Bound = %f\n", Values.optimal_LB);
					}
					if (this_node.LB >= Values.optimal_LB) {
						// 下界不优, 剪枝
						this_node.node_pruned_flag = 1;
						printf("\n\t Node_%d has to be pruned\n", this_node.index);
					}
				}
			}
			tree_search_flag = 1;
		}
	}

	// 清理临时列表
	Lists.occupied_stocks_list.clear();
	Lists.occupied_items_list.clear();
	Lists.all_strips_list.clear();

	return tree_search_flag;
}

// -----------------------------------------------------------------------------
// ChooseVarToBranch - 选择分支变量
// -----------------------------------------------------------------------------
// 功能: 遍历所有正值变量, 找到第一个非整数变量作为分支变量
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点
// 返回值:
//   0 = 存在非整数解
//   1 = 全为整数解
// -----------------------------------------------------------------------------
int ChooseVarToBranch(All_Values& Values, All_Lists& Lists, Node& this_node) {

	int node_int_flag = 1;  // 假设全为整数
	double soln_val;

	// 遍历所有变量寻找非整数解
	int all_solns_num = this_node.all_solns_val_list.size();
	for (int col = 0; col < all_solns_num; col++) {
		soln_val = this_node.all_solns_val_list[col];
		if (soln_val > 0) {
			int soln_int_val = int(soln_val);
			if (soln_int_val != soln_val) {
				// 找到非整数变量
				printf("\n\t Node_%d var_x_%d = %f is NOT an integer\n", this_node.index, col + 1, soln_val);

				// 记录分支变量信息
				this_node.var_to_branch_idx = col;
				this_node.var_to_branch_soln = soln_val;
				this_node.var_to_branch_floor = floor(soln_val);
				this_node.var_to_branch_ceil = ceil(soln_val);

				node_int_flag = 0;
				break;
			}
		}
	}

	return node_int_flag;
}
