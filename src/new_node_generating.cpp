// =============================================================================
// new_node_generating.cpp - 新节点生成模块
// =============================================================================
//
// 功能: 实现分支定界树的节点选择和新节点生成
//
// 模块职责:
//   1. SelectBranchNode - 从待处理节点中选择下一个分支节点
//   2. CreateChildNode  - 基于父节点创建新的子节点
//
// 节点状态:
//   - branched_flag_: 0=未分支, 1=已分支
//   - prune_flag_:    0=未剪枝, 1=已剪枝
//
// 分支搜索状态 (branch_state_):
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
// SelectBranchNode - 选择待分支节点
// -----------------------------------------------------------------------------
// 功能: 根据当前搜索状态选择下一个要分支的节点
//
// 选择策略:
//   1. 如果 branch_state_ = 1 或 2:
//      - 继续深度优先搜索, 选择刚生成的子节点
//   2. 如果 branch_state_ = 3:
//      - 回溯模式, 从节点列表中寻找未处理节点
//      - 优先选择 LB < best_obj_ 的节点
//
// 参数:
//   params      - 全局参数
//   data        - 全局列表
//   parent_node - 输出: 选中的待分支节点
//
// 返回值:
//   0 = 无可分支节点 (搜索完成)
//   1 = 找到待分支节点
// -----------------------------------------------------------------------------
int SelectBranchNode(ProblemParams& params, ProblemData& data, BPNode& parent_node) {

    int parent_branch_flag = -1;
    int pos = -1;
    int num_nodes = data.nodes_.size();

    // =========================================================================
    // 情况 1: 回溯搜索 (branch_state_ = 3)
    // =========================================================================
    // 遍历所有节点, 寻找未分支且未剪枝的候选节点
    // =========================================================================
    if (params.branch_state_ == 3) {
        for (int k = 0; k < num_nodes; k++) {
            if (data.nodes_[k].branched_flag_ != 1 &&
                data.nodes_[k].prune_flag_ != 1) {
                // ----- 检查是否有剪枝条件 -----
                if (data.nodes_[k].lower_bound_ < params.best_obj_) {
                    // 下界优于当前最优, 保留该节点
                    pos = k;
                }
                else {
                    // 下界不优, 执行剪枝
                    int temp_idx = data.nodes_[k].id_;
                    cout << "[节点选择] 节点_" << temp_idx << " 需剪枝\n";
                    data.nodes_[k].prune_flag_ = 1;
                }
            }
        }
    }

    // =========================================================================
    // 情况 2: 深度优先搜索 (branch_state_ = 1 或 2)
    // =========================================================================
    // 根据当前位置选择刚生成的子节点
    // =========================================================================
    if (params.branch_state_ != 3) {
        if (params.is_at_root_ == 1) {
            // ----- 父节点是根节点 -----
            // 节点列表结构: [..., 右子节点, 左子节点]
            if (params.branch_state_ == 1) {
                pos = num_nodes - 1;  // 选择左子节点
            }
            if (params.branch_state_ == 2) {
                pos = num_nodes - 2;  // 选择右子节点
            }
        }

        if (params.is_at_root_ != 1) {
            // ----- 父节点不是根节点 -----
            if (params.fathom_dir_ == 1) {
                // 父节点是其父节点的左子节点
                if (params.branch_state_ == 1) {
                    pos = num_nodes - 2;
                }
                if (params.branch_state_ == 2) {
                    pos = num_nodes - 3;
                }
            }
            if (params.fathom_dir_ == 2) {
                // 父节点是其父节点的右子节点
                if (params.branch_state_ == 1) {
                    pos = num_nodes - 1;
                }
                if (params.branch_state_ == 2) {
                    pos = num_nodes - 2;
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
        parent_node = data.nodes_[pos];
        parent_node.lower_bound_ = 1;
        cout << "[节点选择] 待分支节点: 节点_" << parent_node.id_ << "\n";
    }

    return parent_branch_flag;
}

// -----------------------------------------------------------------------------
// CreateChildNode - 生成新的子节点
// -----------------------------------------------------------------------------
// 功能: 基于父节点信息创建新的子节点, 继承并更新分支约束
//
// 继承内容:
//   1. 模型矩阵 (matrix_)
//   2. Y 列集合 (母板切割模式)
//   3. X 列集合 (条带切割模式)
//   4. 已分支变量索引列表
//
// 新增内容:
//   1. 当前分支变量的固定值 (floor 或 ceil)
//   2. 节点索引和父节点关系
//
// 参数:
//   params      - 全局参数
//   data        - 全局列表
//   new_node    - 输出: 新生成的子节点
//   parent_node - 父节点
// -----------------------------------------------------------------------------
void CreateChildNode(ProblemParams& params, ProblemData& data, BPNode& new_node, BPNode& parent_node) {

    // =========================================================================
    // 初始化节点基本信息
    // =========================================================================
    int num_nodes = data.nodes_.size();
    new_node.id_ = num_nodes + 1;
    new_node.lower_bound_ = -1;

    // ----- 记录父节点关系 -----
    new_node.parent_id_ = parent_node.id_;
    new_node.parent_branch_dir_ = params.branch_state_;
    new_node.parent_branch_val_ = parent_node.branch_var_val_;

    // ----- 输出分支信息 -----
    string branch_type = (params.branch_state_ == 1) ? "左" : "右";
    cout << "[节点生成] 节点_" << new_node.id_ << " (父节点_" << parent_node.id_ << " 的" << branch_type << "子节点)\n";

    // ----- 初始化分支变量信息 -----
    new_node.branch_var_id_ = -1;
    new_node.branch_var_val_ = -1;
    new_node.branch_floor_ = -1;
    new_node.branch_ceil_ = -1;
    new_node.branch_bound_ = -1;

    // =========================================================================
    // 复制父节点的模型矩阵
    // =========================================================================
    int num_cols = parent_node.matrix_.size();
    int num_rows = parent_node.matrix_[0].size();

    for (int col = 0; col < num_cols; col++) {
        vector<double> temp_col;
        for (int row = 0; row < num_rows; row++) {
            double val = parent_node.matrix_[col][row];
            temp_col.push_back(val);
        }
        new_node.matrix_.push_back(temp_col);
    }

    // =========================================================================
    // 复制 Y 列集合 (母板切割模式)
    // =========================================================================
    int num_y_cols = parent_node.y_cols_.size();
    for (int col = 0; col < num_y_cols; col++) {
        vector<double> temp_col;
        for (int row = 0; row < num_rows; row++) {
            double val = parent_node.y_cols_[col][row];
            temp_col.push_back(val);
        }
        new_node.y_cols_.push_back(temp_col);
    }

    // =========================================================================
    // 复制 X 列集合 (条带切割模式)
    // =========================================================================
    int num_x_cols = parent_node.x_cols_.size();
    for (int col = 0; col < num_x_cols; col++) {
        vector<double> temp_col;
        for (int row = 0; row < num_rows; row++) {
            double val = parent_node.x_cols_[col][row];
            temp_col.push_back(val);
        }
        new_node.x_cols_.push_back(temp_col);
    }

    // =========================================================================
    // 复制已分支变量索引列表
    // =========================================================================
    int num_branched = parent_node.branched_var_ids_.size();
    for (int col = 0; col < num_branched; col++) {
        int temp_idx = parent_node.branched_var_ids_[col];
        new_node.branched_var_ids_.push_back(temp_idx);
    }

    // =========================================================================
    // 设置当前分支变量的固定值
    // =========================================================================
    // 左子节点: 变量 <= floor(分数值)
    // 右子节点: 变量 >= ceil(分数值)
    // =========================================================================
    if (params.branch_state_ == 1) {
        new_node.branch_bound_ = parent_node.branch_floor_;
    }
    if (params.branch_state_ == 2) {
        new_node.branch_bound_ = parent_node.branch_ceil_;
    }

    // =========================================================================
    // 更新已分支变量值列表
    // =========================================================================
    double final_int_val = new_node.branch_bound_;

    if (num_branched <= 1) {
        // 根节点的直接子节点
        new_node.branched_vals_.push_back(final_int_val);
    }
    else {
        // 其他节点: 继承父节点的历史值并添加新值
        for (int col = 0; col < num_branched - 1; col++) {
            double val = parent_node.branched_vals_[col];
            new_node.branched_vals_.push_back(val);
        }
        new_node.branched_vals_.push_back(final_int_val);
    }

    // =========================================================================
    // 清理临时列表
    // =========================================================================
    new_node.solution_.clear();
    new_node.duals_.clear();
    new_node.new_y_col_.clear();
    new_node.new_x_cols_.clear();
}
