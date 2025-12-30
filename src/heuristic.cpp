// heuristic.cpp - 启发式生成初始解
//
// 本文件实现启发式算法, 用于生成列生成的初始可行解
//
// 初始解策略 (对角矩阵):
// - Y列: 每列只切割一种条带类型 (共num_strip_types个Y列)
// - X列: 每列只切割一种子板类型 (每种条带类型对应一个X列)
//
// 这种简单策略保证:
// 1. 初始解一定可行 (能满足所有需求约束)
// 2. 初始列池足够大, 列生成能快速收敛
// 3. 避免复杂的组合优化, 快速启动列生成

#include "2DBP.h"

using namespace std;

// 启发式生成初始可行解
// 功能: 为根节点生成初始列池, 确保主问题可行
// 策略: 使用对角矩阵, 每个Y列只产出一种条带, 每个X列只切割一种子板
// 输出:
//   - root_node.y_columns_: 初始Y列集合
//   - root_node.x_columns_: 初始X列集合
//   - root_node.matrix_: 完整约束矩阵
void RunHeuristic(ProblemParams& params, ProblemData& data, BPNode& root_node) {
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;

    LOG("[启发式] 生成初始解");

    // 生成初始Y列 (母板切割方案)
    // 每个Y列对应一种条带类型, 只切割一个该类型条带
    // 这样可以确保每种条带都能被产出
    params.init_y_matrix_.clear();
    root_node.y_columns_.clear();

    for (int j = 0; j < num_strip_types; j++) {
        // pattern[j] = 1 表示切割一个j型条带, 其他为0
        vector<int> pattern(num_strip_types, 0);
        pattern[j] = 1;

        params.init_y_matrix_.push_back(pattern);

        YColumn y_col;
        y_col.pattern_ = pattern;
        root_node.y_columns_.push_back(y_col);
    }

    LOG_FMT("  生成Y列数: %d\n", num_strip_types);

    // 生成初始X列 (条带切割方案)
    // 为每种子板类型生成一个X列, 确保所有需求约束都能被满足
    // 每个X列只切割一种子板类型
    params.init_x_matrix_.clear();
    root_node.x_columns_.clear();

    for (int i = 0; i < num_item_types; i++) {
        int item_width = data.item_types_[i].width_;

        // 找到该子板对应的条带类型 (宽度相等)
        int strip_type = -1;
        for (int j = 0; j < num_strip_types; j++) {
            if (data.strip_types_[j].width_ == item_width) {
                strip_type = j;
                break;
            }
        }

        if (strip_type >= 0) {
            // pattern[i] = 1 表示切割一个i型子板
            vector<int> pattern(num_item_types, 0);
            pattern[i] = 1;

            params.init_x_matrix_.push_back(pattern);

            XColumn x_col;
            x_col.strip_type_id_ = strip_type;  // 该X列属于对应条带类型
            x_col.pattern_ = pattern;
            root_node.x_columns_.push_back(x_col);
        }
    }

    LOG_FMT("  生成X列数: %d\n", (int)root_node.x_columns_.size());

    // 构建完整模型矩阵
    // 矩阵结构 (行优先存储):
    //   行0 ~ J-1: 条带平衡约束
    //   行J ~ J+N-1: 子板需求约束
    //
    // Y列在矩阵中的系数:
    //   [C_1, C_2, ..., C_J, 0, 0, ..., 0]
    //   其中 C_j = pattern[j] 为产出的j型条带数量
    //
    // X列在矩阵中的系数:
    //   [0, ..., -1(位置j), ..., 0, B_1, B_2, ..., B_N]
    //   其中 -1 表示消耗一个j型条带, B_i = pattern[i] 为产出的子板数量
    root_node.matrix_.clear();

    int num_rows = num_strip_types + num_item_types;

    // 添加Y列到矩阵
    for (int col = 0; col < (int)root_node.y_columns_.size(); col++) {
        vector<double> col_data;

        // 条带产出部分 (C矩阵): Y列产出条带, 系数为正
        for (int j = 0; j < num_strip_types; j++) {
            col_data.push_back(root_node.y_columns_[col].pattern_[j]);
        }

        // 子板产出部分: Y列不直接产出子板, 系数为0
        for (int i = 0; i < num_item_types; i++) {
            col_data.push_back(0);
        }

        root_node.matrix_.push_back(col_data);
    }

    // 添加X列到矩阵
    for (int col = 0; col < (int)root_node.x_columns_.size(); col++) {
        vector<double> col_data;
        int strip_type = root_node.x_columns_[col].strip_type_id_;

        // 条带消耗部分 (D矩阵): X列消耗条带, 在对应位置为-1
        for (int j = 0; j < num_strip_types; j++) {
            col_data.push_back((j == strip_type) ? -1 : 0);
        }

        // 子板产出部分 (B矩阵): X列产出子板, 系数为pattern值
        for (int i = 0; i < num_item_types; i++) {
            col_data.push_back(root_node.x_columns_[col].pattern_[i]);
        }

        root_node.matrix_.push_back(col_data);
    }

    LOG("[启发式] 初始解生成完成");

    // 控制台输出: 初始解信息
    int init_plates = num_strip_types;  // 对角矩阵策略, Y列数=初始母板使用量
    CONSOLE_FMT("[启发] 初始解: %d块\n", init_plates);
}
