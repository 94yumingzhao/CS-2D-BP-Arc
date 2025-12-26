// =============================================================================
// output_models.cpp - 模型输出模块
// =============================================================================
//
// 功能: 将主问题模型和对偶问题模型输出到文本文件
//
// 输出文件:
//   - Master Problem.txt: 原始主问题矩阵
//   - Dual Master Problem.txt: 对偶主问题矩阵
//
// 用途: 调试和验证模型正确性
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// OutputMasterProblem - 输出原始主问题矩阵
// -----------------------------------------------------------------------------
// 功能: 将当前主问题的系数矩阵输出到文本文件
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点
// 输出格式:
//   y1  y2  ... x1  x2  ...
//   -------------------------
//   [C 矩阵] | [D 矩阵]  >= 0     (条带约束)
//   -------------------------
//   [0 矩阵] | [B 矩阵]  >= d     (子件约束)
// -----------------------------------------------------------------------------
void OutputMasterProblem(All_Values& Values, All_Lists& Lists, Node& this_node) {

	ofstream dataFile;
	dataFile.open("Master Problem.txt", ios::app);

	int K_num = this_node.Y_cols_list.size();  // Y 变量数
	int P_num = this_node.X_cols_list.size();  // X 变量数
	int J_num = Values.strip_types_num;        // 条带类型数
	int N_num = Values.item_types_num;         // 子件类型数

	int all_cols_num = K_num + P_num;
	int all_rows_num = J_num + N_num;

	// ----- 输出表头 -----
	dataFile << endl;
	dataFile << "MP-" << this_node.iter << endl;

	// 列名: Y 变量 + X 变量
	for (int col = 0; col < all_cols_num; col++) {
		if (col < K_num) {
			dataFile << "y" << col + 1 << "\t";
		}
		if (col >= K_num) {
			dataFile << "x" << col - K_num + 1 << "\t";
		}
	}
	dataFile << endl;

	// 分隔线
	for (int col = 0; col < all_cols_num; col++) {
		dataFile << ("-----------");
	}
	dataFile << endl;

	// ----- 输出系数矩阵 -----
	for (int row = 0; row < all_rows_num + 1; row++) {
		for (int col = 0; col < all_cols_num; col++) {
			if (col < K_num) {
				// Y 列 (母板切割模式)
				int col_pos = col;
				if (row < J_num) {
					dataFile << int(this_node.Y_cols_list[col_pos][row]) << "\t";
				}
				if (row == J_num) {
					dataFile << ("-----------");
				}
				if (row > J_num) {
					dataFile << int(this_node.Y_cols_list[col_pos][row - 1]) << "\t";
				}
			}
			if (col >= K_num) {
				// X 列 (条带切割模式)
				int col_pos = col - K_num;
				if (row < J_num) {
					dataFile << int(this_node.X_cols_list[col_pos][row]) << "\t";
				}
				if (row == J_num) {
					dataFile << ("-----------");
				}
				if (row > J_num) {
					dataFile << int(this_node.X_cols_list[col_pos][row - 1]) << "\t";
				}
			}
		}

		// 输出约束右端项
		if (row < J_num) {
			// 条带平衡约束
			dataFile << ">=" << "\t" << "0";
			dataFile << endl;
		}
		if (row == J_num) {
			dataFile << endl;
		}
		if (row > J_num) {
			// 子件需求约束
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
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点
// 输出格式:
//   v1  v2  ... w1  w2  ...   (对偶变量)
//   -------------------------
//   [目标系数行]
//   -------------------------
//   [约束矩阵转置]  <= c       (对偶约束)
// -----------------------------------------------------------------------------
void OutputDualMasterProblem(All_Values& Values, All_Lists& Lists, Node& this_node) {

	ofstream dataFile;
	dataFile.open("Dual Master Problem.txt", ios::app);

	int K_num = this_node.Y_cols_list.size();
	int P_num = this_node.X_cols_list.size();
	int J_num = Values.strip_types_num;
	int N_num = Values.item_types_num;

	// 对偶问题: 行列互换
	int all_rows_num = K_num + P_num;
	int all_cols_num = J_num + N_num;

	// ----- 输出表头 -----
	dataFile << endl;
	dataFile << "MP-" << this_node.iter << endl;

	// 对偶变量名: v (条带对偶) + w (子件对偶)
	for (int col = 0; col < all_cols_num; col++) {
		if (col < J_num) {
			dataFile << "v" << col + 1 << "\t";
		}
		if (col >= J_num) {
			dataFile << "w" << col - J_num + 1 << "\t";
		}
	}
	dataFile << endl;

	// 分隔线
	for (int col = 0; col < all_cols_num; col++) {
		dataFile << ("-----------");
	}
	dataFile << endl;

	// ----- 输出对偶目标系数 -----
	for (int col = 0; col < all_cols_num; col++) {
		if (col < J_num) {
			dataFile << int(Lists.all_item_types_list[col].demand) << "\t";
		}
		if (col >= J_num) {
			dataFile << "0" << "\t";
		}
	}
	dataFile << endl;

	// 分隔线
	for (int col = 0; col < all_cols_num; col++) {
		dataFile << ("-----------");
	}
	dataFile << endl;

	// ----- 输出对偶约束矩阵 (转置) -----
	for (int row = 0; row < all_rows_num; row++) {
		for (int col = 0; col < all_cols_num; col++) {
			if (row < K_num) {
				// Y 变量对应的对偶约束
				dataFile << int(this_node.Y_cols_list[row][col]) << "\t";
			}
			if (row >= K_num) {
				// X 变量对应的对偶约束
				int row_pos = row - K_num;
				dataFile << int(this_node.X_cols_list[row_pos][col]) << "\t";
			}
		}

		// 输出对偶约束右端项
		if (row < K_num) {
			dataFile << ("<=\t1  y") << row + 1 << endl;
		}
		if (row >= K_num) {
			dataFile << ("<=\t0  x") << row - K_num + 1 << endl;
		}
	}
	dataFile << endl;
}
