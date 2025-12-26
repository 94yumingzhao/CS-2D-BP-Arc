// =============================================================================
// output_models.cpp - 模型输出模块
// =============================================================================
//
// 功能: 将主问题模型和对偶问题模型输出到文本文件, 用于调试和验证
//
// 包含函数:
//   1. OutputMasterProblem     - 输出原始主问题矩阵
//   2. OutputDualMasterProblem - 输出对偶主问题矩阵
//
// 输出文件:
//   - Master Problem.txt:      原始主问题系数矩阵
//   - Dual Master Problem.txt: 对偶主问题系数矩阵
//
// 原始问题与对偶问题关系:
//
//   原始问题 (Primal):              对偶问题 (Dual):
//   min  c^T * x                    max  b^T * pi
//   s.t. A * x >= b                 s.t. A^T * pi <= c
//        x >= 0                          pi >= 0
//
// 本问题中:
//   - 原始变量: y (母板模式), x (条带模式)
//   - 对偶变量: v (条带约束), w (子件约束)
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// OutputMasterProblem - 输出原始主问题矩阵
// -----------------------------------------------------------------------------
// 功能: 将当前主问题的系数矩阵输出到文本文件
//
// 输出格式:
//
//   MP-{迭代次数}
//   y1    y2    ...   x1    x2    ...
//   -----------------------------------
//   [C 矩阵系数]  |  [D 矩阵系数]   >= 0    (条带约束)
//   -----------------------------------
//   [0 矩阵系数]  |  [B 矩阵系数]   >= d    (子件约束)
//
// 矩阵含义:
//   - C[j,k]: 母板模式 k 产出条带类型 j 的数量
//   - D[j,p]: 条带模式 p 消耗条带类型 j 的数量 (通常为 -1)
//   - B[i,p]: 条带模式 p 产出子件类型 i 的数量
//
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点
// -----------------------------------------------------------------------------
void OutputMasterProblem(All_Values& Values, All_Lists& Lists, Node& this_node) {

	ofstream dataFile;
	dataFile.open("Master Problem.txt", ios::app);

	// =========================================================================
	// 问题规模
	// =========================================================================
	int K_num = this_node.Y_cols_list.size();  // Y 变量数 (母板模式)
	int P_num = this_node.X_cols_list.size();  // X 变量数 (条带模式)
	int J_num = Values.strip_types_num;        // 条带类型数
	int N_num = Values.item_types_num;         // 子件类型数

	int all_cols_num = K_num + P_num;
	int all_rows_num = J_num + N_num;

	// =========================================================================
	// 输出表头
	// =========================================================================
	dataFile << endl;
	dataFile << "MP-" << this_node.iter << endl;

	// ----- 列名: Y 变量 + X 变量 -----
	for (int col = 0; col < all_cols_num; col++) {
		if (col < K_num) {
			dataFile << "y" << col + 1 << "\t";
		}
		if (col >= K_num) {
			dataFile << "x" << col - K_num + 1 << "\t";
		}
	}
	dataFile << endl;

	// ----- 分隔线 -----
	for (int col = 0; col < all_cols_num; col++) {
		dataFile << ("-----------");
	}
	dataFile << endl;

	// =========================================================================
	// 输出系数矩阵
	// =========================================================================
	for (int row = 0; row < all_rows_num + 1; row++) {
		for (int col = 0; col < all_cols_num; col++) {
			if (col < K_num) {
				// ----- Y 列 (母板切割模式) -----
				int col_pos = col;
				if (row < J_num) {
					// C 矩阵部分
					dataFile << int(this_node.Y_cols_list[col_pos][row]) << "\t";
				}
				if (row == J_num) {
					// 分隔行
					dataFile << ("-----------");
				}
				if (row > J_num) {
					// 0 矩阵部分
					dataFile << int(this_node.Y_cols_list[col_pos][row - 1]) << "\t";
				}
			}
			if (col >= K_num) {
				// ----- X 列 (条带切割模式) -----
				int col_pos = col - K_num;
				if (row < J_num) {
					// D 矩阵部分
					dataFile << int(this_node.X_cols_list[col_pos][row]) << "\t";
				}
				if (row == J_num) {
					// 分隔行
					dataFile << ("-----------");
				}
				if (row > J_num) {
					// B 矩阵部分
					dataFile << int(this_node.X_cols_list[col_pos][row - 1]) << "\t";
				}
			}
		}

		// ----- 输出约束右端项 -----
		if (row < J_num) {
			// 条带平衡约束: >= 0
			dataFile << ">=" << "\t" << "0";
			dataFile << endl;
		}
		if (row == J_num) {
			// 分隔行
			dataFile << endl;
		}
		if (row > J_num) {
			// 子件需求约束: >= demand
			int row_pos = row - J_num - 1;
			dataFile << ">=" << "\t" << int(Lists.all_item_types_list[row_pos].demand);
			dataFile << endl;
		}
	}
	dataFile.close();
}

// -----------------------------------------------------------------------------
// OutputDualMasterProblem - 输出对偶主问题矩阵
// -----------------------------------------------------------------------------
// 功能: 将当前主问题的对偶形式输出到文本文件
//
// 对偶问题结构:
//   max  0 * v + d^T * w
//   s.t. C^T * v           <= 1  (Y 变量对偶约束)
//        D^T * v + B^T * w <= 0  (X 变量对偶约束)
//        v <= 0, w >= 0
//
// 输出格式:
//
//   MP-{迭代次数}
//   v1    v2    ...   w1    w2    ...   (对偶变量)
//   -----------------------------------
//   [目标系数行: 0...0  d1...dN]
//   -----------------------------------
//   [约束矩阵转置]                      <= c
//
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点
// -----------------------------------------------------------------------------
void OutputDualMasterProblem(All_Values& Values, All_Lists& Lists, Node& this_node) {

	ofstream dataFile;
	dataFile.open("Dual Master Problem.txt", ios::app);

	// =========================================================================
	// 问题规模 (对偶问题行列互换)
	// =========================================================================
	int K_num = this_node.Y_cols_list.size();
	int P_num = this_node.X_cols_list.size();
	int J_num = Values.strip_types_num;
	int N_num = Values.item_types_num;

	int all_rows_num = K_num + P_num;  // 对偶约束数 = 原始变量数
	int all_cols_num = J_num + N_num;  // 对偶变量数 = 原始约束数

	// =========================================================================
	// 输出表头
	// =========================================================================
	dataFile << endl;
	dataFile << "MP-" << this_node.iter << endl;

	// ----- 对偶变量名: v (条带对偶) + w (子件对偶) -----
	for (int col = 0; col < all_cols_num; col++) {
		if (col < J_num) {
			dataFile << "v" << col + 1 << "\t";
		}
		if (col >= J_num) {
			dataFile << "w" << col - J_num + 1 << "\t";
		}
	}
	dataFile << endl;

	// ----- 分隔线 -----
	for (int col = 0; col < all_cols_num; col++) {
		dataFile << ("-----------");
	}
	dataFile << endl;

	// =========================================================================
	// 输出对偶目标系数
	// =========================================================================
	// 对偶目标: max 0 * v + d^T * w
	// 其中 d = (demand_1, ..., demand_N)
	// =========================================================================
	for (int col = 0; col < all_cols_num; col++) {
		if (col < J_num) {
			// v 的系数 = 0 (条带约束右端项)
			dataFile << int(Lists.all_item_types_list[col].demand) << "\t";
		}
		if (col >= J_num) {
			// w 的系数 = demand (子件约束右端项)
			dataFile << "0" << "\t";
		}
	}
	dataFile << endl;

	// ----- 分隔线 -----
	for (int col = 0; col < all_cols_num; col++) {
		dataFile << ("-----------");
	}
	dataFile << endl;

	// =========================================================================
	// 输出对偶约束矩阵 (原始矩阵的转置)
	// =========================================================================
	for (int row = 0; row < all_rows_num; row++) {
		for (int col = 0; col < all_cols_num; col++) {
			if (row < K_num) {
				// ----- Y 变量对应的对偶约束 -----
				dataFile << int(this_node.Y_cols_list[row][col]) << "\t";
			}
			if (row >= K_num) {
				// ----- X 变量对应的对偶约束 -----
				int row_pos = row - K_num;
				dataFile << int(this_node.X_cols_list[row_pos][col]) << "\t";
			}
		}

		// ----- 输出对偶约束右端项 -----
		if (row < K_num) {
			// Y 变量对偶约束: <= 1 (目标系数)
			dataFile << ("<=\t1  y") << row + 1 << endl;
		}
		if (row >= K_num) {
			// X 变量对偶约束: <= 0 (目标系数)
			dataFile << ("<=\t0  x") << row - K_num + 1 << endl;
		}
	}
	dataFile << endl;
}
