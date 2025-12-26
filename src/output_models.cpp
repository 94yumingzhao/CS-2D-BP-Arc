// =============================================================================
// output_models.cpp - 模型输出模块
// =============================================================================
// 功能: 将主问题模型和对偶问题模型输出到文本文件, 用于调试和验证
//
// 包含函数:
//   1. OutputPrimalMP - 输出原始主问题矩阵
//   2. OutputDualMP   - 输出对偶主问题矩阵
// =============================================================================

#include "2DBP.h"

using namespace std;


// 输出原始主问题矩阵
void ExportMP(ProblemParams& params, ProblemData& data, BPNode& cur_node) {
    ofstream data_file;
    data_file.open("Master Problem.txt", ios::app);

    // 问题规模
    int num_y_cols = cur_node.y_cols_.size();
    int num_x_cols = cur_node.x_cols_.size();
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;

    int num_cols = num_y_cols + num_x_cols;
    int num_rows = num_strip_types + num_item_types;

    // 输出表头
    data_file << endl;
    data_file << "MP-" << cur_node.iter_ << endl;

    // 列名: Y 变量 + X 变量
    for (int col = 0; col < num_cols; col++) {
        if (col < num_y_cols) {
            data_file << "y" << col + 1 << "\t";
        }
        if (col >= num_y_cols) {
            data_file << "x" << col - num_y_cols + 1 << "\t";
        }
    }
    data_file << endl;

    // 分隔线
    for (int col = 0; col < num_cols; col++) {
        data_file << ("-----------");
    }
    data_file << endl;

    // 输出系数矩阵
    for (int row = 0; row < num_rows + 1; row++) {
        for (int col = 0; col < num_cols; col++) {
            if (col < num_y_cols) {
                // Y 列 (母板切割模式)
                int col_pos = col;
                if (row < num_strip_types) {
                    // C 矩阵部分
                    data_file << int(cur_node.y_cols_[col_pos][row]) << "\t";
                }
                if (row == num_strip_types) {
                    // 分隔行
                    data_file << ("-----------");
                }
                if (row > num_strip_types) {
                    // 0 矩阵部分
                    data_file << int(cur_node.y_cols_[col_pos][row - 1]) << "\t";
                }
            }
            if (col >= num_y_cols) {
                // X 列 (条带切割模式)
                int col_pos = col - num_y_cols;
                if (row < num_strip_types) {
                    // D 矩阵部分
                    data_file << int(cur_node.x_cols_[col_pos][row]) << "\t";
                }
                if (row == num_strip_types) {
                    // 分隔行
                    data_file << ("-----------");
                }
                if (row > num_strip_types) {
                    // B 矩阵部分
                    data_file << int(cur_node.x_cols_[col_pos][row - 1]) << "\t";
                }
            }
        }

        // 输出约束右端项
        if (row < num_strip_types) {
            // 条带平衡约束: >= 0
            data_file << ">=" << "\t" << "0";
            data_file << endl;
        }
        if (row == num_strip_types) {
            // 分隔行
            data_file << endl;
        }
        if (row > num_strip_types) {
            // 子件需求约束: >= demand
            int row_pos = row - num_strip_types - 1;
            data_file << ">=" << "\t" << int(data.item_types_[row_pos].demand_);
            data_file << endl;
        }
    }
    data_file.close();
}


// 输出对偶主问题矩阵
void ExportDualMP(ProblemParams& params, ProblemData& data, BPNode& cur_node) {
    ofstream data_file;
    data_file.open("Dual Master Problem.txt", ios::app);

    // 问题规模 (对偶问题行列互换)
    int num_y_cols = cur_node.y_cols_.size();
    int num_x_cols = cur_node.x_cols_.size();
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;

    int num_rows = num_y_cols + num_x_cols;
    int num_cols = num_strip_types + num_item_types;

    // 输出表头
    data_file << endl;
    data_file << "MP-" << cur_node.iter_ << endl;

    // 对偶变量名: v (条带对偶) + w (子件对偶)
    for (int col = 0; col < num_cols; col++) {
        if (col < num_strip_types) {
            data_file << "v" << col + 1 << "\t";
        }
        if (col >= num_strip_types) {
            data_file << "w" << col - num_strip_types + 1 << "\t";
        }
    }
    data_file << endl;

    // 分隔线
    for (int col = 0; col < num_cols; col++) {
        data_file << ("-----------");
    }
    data_file << endl;

    // 输出对偶目标系数
    // 对偶目标: max 0 * v + d^T * w, 其中 d = (demand_1, ..., demand_N)
    for (int col = 0; col < num_cols; col++) {
        if (col < num_strip_types) {
            // v 的系数 = 0 (条带约束右端项)
            data_file << int(data.item_types_[col].demand_) << "\t";
        }
        if (col >= num_strip_types) {
            // w 的系数 = demand (子件约束右端项)
            data_file << "0" << "\t";
        }
    }
    data_file << endl;

    // 分隔线
    for (int col = 0; col < num_cols; col++) {
        data_file << ("-----------");
    }
    data_file << endl;

    // 输出对偶约束矩阵 (原始矩阵的转置)
    for (int row = 0; row < num_rows; row++) {
        for (int col = 0; col < num_cols; col++) {
            if (row < num_y_cols) {
                // Y 变量对应的对偶约束
                data_file << int(cur_node.y_cols_[row][col]) << "\t";
            }
            if (row >= num_y_cols) {
                // X 变量对应的对偶约束
                int row_pos = row - num_y_cols;
                data_file << int(cur_node.x_cols_[row_pos][col]) << "\t";
            }
        }

        // 输出对偶约束右端项
        if (row < num_y_cols) {
            // Y 变量对偶约束: <= 1 (目标系数)
            data_file << ("<=\t1  y") << row + 1 << endl;
        }
        if (row >= num_y_cols) {
            // X 变量对偶约束: <= 0 (目标系数)
            data_file << ("<=\t0  x") << row - num_y_cols + 1 << endl;
        }
    }
    data_file << endl;
}
