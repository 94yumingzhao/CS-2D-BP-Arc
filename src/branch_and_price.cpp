// =============================================================================
// branch_and_price.cpp - 分支定价算法
// =============================================================================

#include "2DBP.h"

using namespace std;

// 检查解是否为整数解
bool IsIntegerSolution(NodeSolution& solution) {
    // 检查Y列
    for (int i = 0; i < (int)solution.y_columns_.size(); i++) {
        double val = solution.y_columns_[i].value_;
        if (val > kZeroTolerance) {
            double frac = val - floor(val);
            if (frac > kZeroTolerance && frac < 1 - kZeroTolerance) {
                return false;
            }
        }
    }

    // 检查X列
    for (int i = 0; i < (int)solution.x_columns_.size(); i++) {
        double val = solution.x_columns_[i].value_;
        if (val > kZeroTolerance) {
            double frac = val - floor(val);
            if (frac > kZeroTolerance && frac < 1 - kZeroTolerance) {
                return false;
            }
        }
    }

    return true;
}

// 选择分支变量 (已废弃, 保留兼容)
// 返回: 待分支变量索引, -1表示无需分支
int SelectBranchVar(BPNode* node) {
    double max_frac = 0;
    int branch_idx = -1;

    // 检查Y列
    int y_count = static_cast<int>(node->solution_.y_columns_.size());
    for (int i = 0; i < y_count; i++) {
        double val = node->solution_.y_columns_[i].value_;
        if (val > kZeroTolerance) {
            double frac = val - floor(val);
            if (frac > kZeroTolerance && frac < 1 - kZeroTolerance) {
                if (frac > max_frac) {
                    max_frac = frac;
                    branch_idx = i;
                    node->branch_var_val_ = val;
                }
            }
        }
    }

    // 检查X列
    for (int i = 0; i < (int)node->solution_.x_columns_.size(); i++) {
        double val = node->solution_.x_columns_[i].value_;
        if (val > kZeroTolerance) {
            double frac = val - floor(val);
            if (frac > kZeroTolerance && frac < 1 - kZeroTolerance) {
                if (frac > max_frac) {
                    max_frac = frac;
                    branch_idx = y_count + i;
                    node->branch_var_val_ = val;
                }
            }
        }
    }

    if (branch_idx >= 0) {
        node->branch_var_id_ = branch_idx;
        node->branch_floor_ = floor(node->branch_var_val_);
        node->branch_ceil_ = ceil(node->branch_var_val_);
    }

    return branch_idx;
}

// 选择分支 Arc (优先 SP1, 再 SP2)
// 返回: 分支类型 (kBranchNone/kBranchSP1Arc/kBranchSP2Arc)
int SelectBranchArc(ProblemParams& params, ProblemData& data, BPNode* node) {
    // 重置分支信息
    node->branch_type_ = kBranchNone;
    node->branch_arc_ = {-1, -1};
    node->branch_arc_flow_ = -1;
    node->branch_arc_strip_type_ = -1;

    // 1. 检查 SP1 Arc (宽度方向)
    map<int, tuple<int, int, double>> sp1_arc_flow;
    ConvertYColsToSP1ArcFlow(node->solution_.y_columns_, data, sp1_arc_flow);

    array<int, 2> branch_arc;
    double branch_flow;

    if (FindBranchArcSP1(sp1_arc_flow, branch_arc, branch_flow)) {
        node->branch_type_ = kBranchSP1Arc;
        node->branch_arc_ = branch_arc;
        node->branch_arc_flow_ = branch_flow;
        LOG_FMT("[分支] 选择 SP1 Arc [%d,%d] 流量=%.4f\n",
            branch_arc[0], branch_arc[1], branch_flow);
        return kBranchSP1Arc;
    }

    // 2. SP1 全整数, 检查 SP2 Arc (长度方向, 遍历所有条带类型)
    for (int j = 0; j < params.num_strip_types_; j++) {
        map<int, tuple<int, int, double>> sp2_arc_flow;
        ConvertXColsToSP2ArcFlow(node->solution_.x_columns_, j, data, sp2_arc_flow);

        if (FindBranchArcSP2(sp2_arc_flow, branch_arc, branch_flow)) {
            node->branch_type_ = kBranchSP2Arc;
            node->branch_arc_ = branch_arc;
            node->branch_arc_flow_ = branch_flow;
            node->branch_arc_strip_type_ = j;
            LOG_FMT("[分支] 选择 SP2 Arc [%d,%d] 条带类型=%d 流量=%.4f\n",
                branch_arc[0], branch_arc[1], j, branch_flow);
            return kBranchSP2Arc;
        }
    }

    // 3. 全部整数, 无需分支
    LOG("[分支] Arc 流量全部整数, 无需分支");
    return kBranchNone;
}

// 创建左子节点 (Arc <= floor)
void CreateLeftChild(BPNode* parent, int new_id, BPNode* child) {
    // 复制基本信息
    child->id_ = new_id;
    child->parent_id_ = parent->id_;
    child->branch_dir_ = 1;  // 左分支
    child->sp1_method_ = parent->sp1_method_;
    child->sp2_method_ = parent->sp2_method_;

    // 复制列集合
    child->y_columns_ = parent->y_columns_;
    child->x_columns_ = parent->x_columns_;

    // 继承父节点的 Arc 约束
    child->sp1_zero_arcs_ = parent->sp1_zero_arcs_;
    child->sp1_lower_arcs_ = parent->sp1_lower_arcs_;
    child->sp1_lower_bounds_ = parent->sp1_lower_bounds_;
    child->sp1_greater_arcs_ = parent->sp1_greater_arcs_;
    child->sp1_greater_bounds_ = parent->sp1_greater_bounds_;
    child->sp2_zero_arcs_ = parent->sp2_zero_arcs_;
    child->sp2_lower_arcs_ = parent->sp2_lower_arcs_;
    child->sp2_lower_bounds_ = parent->sp2_lower_bounds_;
    child->sp2_greater_arcs_ = parent->sp2_greater_arcs_;
    child->sp2_greater_bounds_ = parent->sp2_greater_bounds_;

    // 添加新的 Arc 约束 (Arc <= floor)
    if (parent->branch_type_ == kBranchSP1Arc) {
        int bound = static_cast<int>(floor(parent->branch_arc_flow_));
        if (bound == 0) {
            // Arc <= 0 等价于禁用该 Arc
            child->sp1_zero_arcs_.insert(parent->branch_arc_);
            LOG_FMT("[Branch] 左子节点 %d: SP1 Arc[%d,%d] = 0 (禁用)\n",
                new_id, parent->branch_arc_[0], parent->branch_arc_[1]);
        } else {
            child->sp1_lower_arcs_.push_back(parent->branch_arc_);
            child->sp1_lower_bounds_.push_back(bound);
            LOG_FMT("[Branch] 左子节点 %d: SP1 Arc[%d,%d] <= %d\n",
                new_id, parent->branch_arc_[0], parent->branch_arc_[1], bound);
        }
    } else if (parent->branch_type_ == kBranchSP2Arc) {
        int strip_type = parent->branch_arc_strip_type_;
        int bound = static_cast<int>(floor(parent->branch_arc_flow_));
        if (bound == 0) {
            child->sp2_zero_arcs_[strip_type].insert(parent->branch_arc_);
            LOG_FMT("[Branch] 左子节点 %d: SP2 Arc[%d,%d] 条带%d = 0 (禁用)\n",
                new_id, parent->branch_arc_[0], parent->branch_arc_[1], strip_type);
        } else {
            child->sp2_lower_arcs_[strip_type].push_back(parent->branch_arc_);
            child->sp2_lower_bounds_[strip_type].push_back(bound);
            LOG_FMT("[Branch] 左子节点 %d: SP2 Arc[%d,%d] 条带%d <= %d\n",
                new_id, parent->branch_arc_[0], parent->branch_arc_[1], strip_type, bound);
        }
    }
}

// 创建右子节点 (Arc >= ceil)
void CreateRightChild(BPNode* parent, int new_id, BPNode* child) {
    // 复制基本信息
    child->id_ = new_id;
    child->parent_id_ = parent->id_;
    child->branch_dir_ = 2;  // 右分支
    child->sp1_method_ = parent->sp1_method_;
    child->sp2_method_ = parent->sp2_method_;

    // 复制列集合
    child->y_columns_ = parent->y_columns_;
    child->x_columns_ = parent->x_columns_;

    // 继承父节点的 Arc 约束
    child->sp1_zero_arcs_ = parent->sp1_zero_arcs_;
    child->sp1_lower_arcs_ = parent->sp1_lower_arcs_;
    child->sp1_lower_bounds_ = parent->sp1_lower_bounds_;
    child->sp1_greater_arcs_ = parent->sp1_greater_arcs_;
    child->sp1_greater_bounds_ = parent->sp1_greater_bounds_;
    child->sp2_zero_arcs_ = parent->sp2_zero_arcs_;
    child->sp2_lower_arcs_ = parent->sp2_lower_arcs_;
    child->sp2_lower_bounds_ = parent->sp2_lower_bounds_;
    child->sp2_greater_arcs_ = parent->sp2_greater_arcs_;
    child->sp2_greater_bounds_ = parent->sp2_greater_bounds_;

    // 添加新的 Arc 约束 (Arc >= ceil)
    if (parent->branch_type_ == kBranchSP1Arc) {
        int bound = static_cast<int>(ceil(parent->branch_arc_flow_));
        child->sp1_greater_arcs_.push_back(parent->branch_arc_);
        child->sp1_greater_bounds_.push_back(bound);
        LOG_FMT("[Branch] 右子节点 %d: SP1 Arc[%d,%d] >= %d\n",
            new_id, parent->branch_arc_[0], parent->branch_arc_[1], bound);
    } else if (parent->branch_type_ == kBranchSP2Arc) {
        int strip_type = parent->branch_arc_strip_type_;
        int bound = static_cast<int>(ceil(parent->branch_arc_flow_));
        child->sp2_greater_arcs_[strip_type].push_back(parent->branch_arc_);
        child->sp2_greater_bounds_[strip_type].push_back(bound);
        LOG_FMT("[Branch] 右子节点 %d: SP2 Arc[%d,%d] 条带%d >= %d\n",
            new_id, parent->branch_arc_[0], parent->branch_arc_[1], strip_type, bound);
    }
}

// 选择待分支节点 (从链表中选择下界最小的未剪枝节点)
BPNode* SelectBranchNode(BPNode* head) {
    BPNode* best = nullptr;
    double best_lb = INFINITY;

    BPNode* curr = head;
    while (curr != nullptr) {
        if (curr->prune_flag_ == 0 && curr->branched_flag_ == 0) {
            if (curr->lower_bound_ < best_lb) {
                best_lb = curr->lower_bound_;
                best = curr;
            }
        }
        curr = curr->next_;
    }

    return best;
}

// 分支定价主循环
int RunBranchAndPrice(ProblemParams& params, ProblemData& data, BPNode* root) {
    LOG("[BP] 分支定价开始 (Arc 分支策略)");

    // 确保 Arc Flow 网络已生成
    if (data.sp1_arc_data_.arc_list_.empty()) {
        GenerateAllArcs(data, params);
    }

    // 为根节点解生成 Arc 集合
    GenerateYArcSetMatrix(*root, data.strip_widths_);
    for (int j = 0; j < params.num_strip_types_; j++) {
        GenerateXArcSetMatrix(*root, data.item_lengths_, j);
    }

    // 初始化节点链表
    BPNode* head = root;
    BPNode* tail = root;
    int node_count = 1;

    // 检查根节点是否为整数解 (使用 Arc 分支判断)
    int branch_type = SelectBranchArc(params, data, root);
    if (branch_type == kBranchNone) {
        params.global_best_int_ = root->solution_.obj_val_;
        params.global_best_y_cols_ = root->solution_.y_columns_;
        params.global_best_x_cols_ = root->solution_.x_columns_;
        LOG("[BP] 根节点 Arc 流量全整数, 即为最优解");
        return 0;
    }

    // 分支定价主循环
    while (true) {
        // 选择待分支节点
        BPNode* parent = SelectBranchNode(head);

        if (parent == nullptr) {
            LOG("[BP] 无可分支节点, 搜索完成");
            break;
        }

        LOG_FMT("[BP] 选择节点 %d 进行分支 (LB=%.4f)\n",
            parent->id_, parent->lower_bound_);

        // 创建左子节点
        BPNode* left = new BPNode();
        node_count++;
        params.node_counter_++;
        CreateLeftChild(parent, params.node_counter_, left);

        // 求解左子节点
        SolveNodeCG(params, data, left);

        // 将左子节点加入链表
        tail->next_ = left;
        tail = left;

        // 检查左子节点
        if (left->prune_flag_ == 0) {
            // 为解生成 Arc 集合
            GenerateYArcSetMatrix(*left, data.strip_widths_);
            for (int j = 0; j < params.num_strip_types_; j++) {
                GenerateXArcSetMatrix(*left, data.item_lengths_, j);
            }

            // 使用 Arc 分支判断是否为整数解
            int left_branch_type = SelectBranchArc(params, data, left);
            if (left_branch_type == kBranchNone) {
                // Arc 流量全整数, 找到整数解
                if (left->solution_.obj_val_ < params.global_best_int_) {
                    params.global_best_int_ = left->solution_.obj_val_;
                    params.global_best_y_cols_ = left->solution_.y_columns_;
                    params.global_best_x_cols_ = left->solution_.x_columns_;
                    LOG_FMT("[BP] 找到新整数解, 目标值=%.4f\n", params.global_best_int_);
                }
                left->branched_flag_ = 1;  // 无需再分支
            }
            // 否则 left->branch_type_ 已由 SelectBranchArc 设置
        }

        // 创建右子节点
        BPNode* right = new BPNode();
        node_count++;
        params.node_counter_++;
        CreateRightChild(parent, params.node_counter_, right);

        // 求解右子节点
        SolveNodeCG(params, data, right);

        // 将右子节点加入链表
        tail->next_ = right;
        tail = right;

        // 检查右子节点
        if (right->prune_flag_ == 0) {
            // 为解生成 Arc 集合
            GenerateYArcSetMatrix(*right, data.strip_widths_);
            for (int j = 0; j < params.num_strip_types_; j++) {
                GenerateXArcSetMatrix(*right, data.item_lengths_, j);
            }

            // 使用 Arc 分支判断是否为整数解
            int right_branch_type = SelectBranchArc(params, data, right);
            if (right_branch_type == kBranchNone) {
                // Arc 流量全整数, 找到整数解
                if (right->solution_.obj_val_ < params.global_best_int_) {
                    params.global_best_int_ = right->solution_.obj_val_;
                    params.global_best_y_cols_ = right->solution_.y_columns_;
                    params.global_best_x_cols_ = right->solution_.x_columns_;
                    LOG_FMT("[BP] 找到新整数解, 目标值=%.4f\n", params.global_best_int_);
                }
                right->branched_flag_ = 1;
            }
            // 否则 right->branch_type_ 已由 SelectBranchArc 设置
        }

        // 标记父节点已分支
        parent->branched_flag_ = 1;

        // 剪枝: 下界 >= 全局最优整数解
        BPNode* curr = head;
        while (curr != nullptr) {
            if (curr->prune_flag_ == 0 && curr->branched_flag_ == 0) {
                if (curr->lower_bound_ >= params.global_best_int_ - kZeroTolerance) {
                    curr->prune_flag_ = 1;
                    LOG_FMT("[BP] 节点 %d 被剪枝 (LB=%.4f >= UB=%.4f)\n",
                        curr->id_, curr->lower_bound_, params.global_best_int_);
                }
            }
            curr = curr->next_;
        }

        // 安全检查
        if (node_count > 100) {
            LOG("[BP] 达到最大节点数, 强制终止");
            break;
        }
    }

    // 计算最优性间隙
    double best_lb = INFINITY;
    BPNode* curr = head;
    while (curr != nullptr) {
        if (curr->prune_flag_ == 0 && curr->lower_bound_ < best_lb) {
            best_lb = curr->lower_bound_;
        }
        curr = curr->next_;
    }

    if (params.global_best_int_ < INFINITY && best_lb < INFINITY) {
        params.gap_ = (params.global_best_int_ - best_lb) / params.global_best_int_;
    }

    LOG_FMT("[BP] 分支定价结束, 最优解=%.4f, 间隙=%.2f%%\n",
        params.global_best_int_, params.gap_ * 100);

    return 0;
}
