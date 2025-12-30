// branch_and_price.cpp - 分支定价算法主循环
//
// 分支定价 (Branch and Price) = 列生成 (Column Generation) + 分支定界 (Branch and Bound)
// 算法流程:
//   1. 在根节点运行列生成，得到 LP 松弛解
//   2. 若 LP 解为整数解，则为最优解，算法结束
//   3. 若 LP 解包含分数值，选择分支变量/Arc 进行分支
//   4. 创建左右子节点，对每个子节点运行列生成
//   5. 更新全局最优整数解和剪枝条件
//   6. 重复步骤 3-5 直到所有节点都被处理或剪枝
//
// 本实现采用 Arc 流量分支策略:
//   将 LP 解转换为 Arc 流量，若某 Arc 流量为分数则对其分支
//   左分支: Arc 流量 <= floor(流量)
//   右分支: Arc 流量 >= ceil(流量)

#include "2DBP.h"

#include <chrono>

using namespace std;

// 检查 LP 解是否为整数解
// 遍历所有 Y 列和 X 列，检查解值是否都接近整数
// 返回: true = 整数解，false = 存在分数解
bool IsIntegerSolution(NodeSolution& solution) {
    // 检查 Y 列 (母板切割方案)
    for (int i = 0; i < (int)solution.y_columns_.size(); i++) {
        double val = solution.y_columns_[i].value_;

        // 只检查非零解值
        if (val > kZeroTolerance) {
            double frac = val - floor(val);

            // 判断是否为分数: 小数部分不接近 0 且不接近 1
            if (frac > kZeroTolerance && frac < 1 - kZeroTolerance) {
                return false;
            }
        }
    }

    // 检查 X 列 (条带切割方案)
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

// 选择分支变量 (变量分支策略，已废弃)
// 选择分数部分最大的变量进行分支
// 返回: 待分支变量索引，-1 表示无需分支 (整数解)
int SelectBranchVar(BPNode* node) {
    double max_frac = 0;
    int branch_idx = -1;

    // 检查 Y 列
    int y_count = static_cast<int>(node->solution_.y_columns_.size());
    for (int i = 0; i < y_count; i++) {
        double val = node->solution_.y_columns_[i].value_;

        if (val > kZeroTolerance) {
            double frac = val - floor(val);

            // 找分数部分最大的变量
            if (frac > kZeroTolerance && frac < 1 - kZeroTolerance) {
                if (frac > max_frac) {
                    max_frac = frac;
                    branch_idx = i;
                    node->branch_var_val_ = val;
                }
            }
        }
    }

    // 检查 X 列
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

    // 记录分支信息
    if (branch_idx >= 0) {
        node->branch_var_id_ = branch_idx;
        node->branch_floor_ = floor(node->branch_var_val_);
        node->branch_ceil_ = ceil(node->branch_var_val_);
    }

    return branch_idx;
}

// 选择分支 Arc (Arc 流量分支策略)
// 分支优先级: SP1 Arc > SP2 Arc
// 策略: 将 LP 解转换为 Arc 流量，选择流量最接近 0.5 的分数 Arc
// 返回: 分支类型 (kBranchNone / kBranchSP1Arc / kBranchSP2Arc)
int SelectBranchArc(ProblemParams& params, ProblemData& data, BPNode* node) {
    // 重置分支信息
    node->branch_type_ = kBranchNone;
    node->branch_arc_ = {-1, -1};
    node->branch_arc_flow_ = -1;
    node->branch_arc_strip_type_ = -1;

    // 步骤 1: 检查 SP1 Arc (宽度方向)
    // 将所有 Y 列的 LP 解转换为 SP1 Arc 流量
    map<int, tuple<int, int, double>> sp1_arc_flow;
    ConvertYColsToSP1ArcFlow(node->solution_.y_columns_, data, sp1_arc_flow);

    array<int, 2> branch_arc;
    double branch_flow;

    // 在 SP1 Arc 中寻找分数流量
    if (FindBranchArcSP1(sp1_arc_flow, branch_arc, branch_flow)) {
        // 找到分数 Arc，设置分支信息
        node->branch_type_ = kBranchSP1Arc;
        node->branch_arc_ = branch_arc;
        node->branch_arc_flow_ = branch_flow;
        LOG_FMT("[分支] 选择 SP1 Arc [%d,%d] 流量=%.4f\n",
            branch_arc[0], branch_arc[1], branch_flow);
        return kBranchSP1Arc;
    }

    // 步骤 2: SP1 全整数，检查 SP2 Arc (长度方向)
    // 需要遍历所有条带类型，因为每种条带有独立的 SP2 网络
    for (int j = 0; j < params.num_strip_types_; j++) {
        map<int, tuple<int, int, double>> sp2_arc_flow;
        ConvertXColsToSP2ArcFlow(node->solution_.x_columns_, j, data, sp2_arc_flow);

        if (FindBranchArcSP2(sp2_arc_flow, branch_arc, branch_flow)) {
            // 找到分数 Arc，记录条带类型
            node->branch_type_ = kBranchSP2Arc;
            node->branch_arc_ = branch_arc;
            node->branch_arc_flow_ = branch_flow;
            node->branch_arc_strip_type_ = j;
            LOG_FMT("[分支] 选择 SP2 Arc [%d,%d] 条带类型=%d 流量=%.4f\n",
                branch_arc[0], branch_arc[1], j, branch_flow);
            return kBranchSP2Arc;
        }
    }

    // 步骤 3: 所有 Arc 流量都是整数，无需分支
    LOG("[分支] Arc 流量全部整数, 无需分支");
    return kBranchNone;
}

// 创建左子节点
// 分支约束: Arc <= floor(流量)
// 若 floor(流量) = 0，则禁用该 Arc (设置为不可用)
void CreateLeftChild(BPNode* parent, int new_id, BPNode* child) {
    // 复制基本信息
    child->id_ = new_id;
    child->parent_id_ = parent->id_;
    child->branch_dir_ = 1;  // 标记为左分支
    child->sp1_method_ = parent->sp1_method_;
    child->sp2_method_ = parent->sp2_method_;

    // 复制父节点的列集合作为初始解
    child->y_columns_ = parent->y_columns_;
    child->x_columns_ = parent->x_columns_;

    // 继承父节点的所有 Arc 约束
    // 这是分支定价的关键: 子节点必须满足祖先节点的所有约束
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

    // 添加新的分支约束 (Arc <= floor)
    if (parent->branch_type_ == kBranchSP1Arc) {
        // SP1 分支: 限制宽度方向的 Arc
        int bound = static_cast<int>(floor(parent->branch_arc_flow_));

        if (bound == 0) {
            // 特殊情况: floor(流量) = 0 意味着禁用该 Arc
            child->sp1_zero_arcs_.insert(parent->branch_arc_);
            LOG_FMT("[Branch] 左子节点 %d: SP1 Arc[%d,%d] = 0 (禁用)\n",
                new_id, parent->branch_arc_[0], parent->branch_arc_[1]);
        } else {
            // 添加上界约束: Arc <= bound
            child->sp1_lower_arcs_.push_back(parent->branch_arc_);
            child->sp1_lower_bounds_.push_back(bound);
            LOG_FMT("[Branch] 左子节点 %d: SP1 Arc[%d,%d] <= %d\n",
                new_id, parent->branch_arc_[0], parent->branch_arc_[1], bound);
        }
    } else if (parent->branch_type_ == kBranchSP2Arc) {
        // SP2 分支: 限制长度方向的 Arc (需要指定条带类型)
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

// 创建右子节点
// 分支约束: Arc >= ceil(流量)
void CreateRightChild(BPNode* parent, int new_id, BPNode* child) {
    // 复制基本信息
    child->id_ = new_id;
    child->parent_id_ = parent->id_;
    child->branch_dir_ = 2;  // 标记为右分支
    child->sp1_method_ = parent->sp1_method_;
    child->sp2_method_ = parent->sp2_method_;

    // 复制父节点的列集合
    child->y_columns_ = parent->y_columns_;
    child->x_columns_ = parent->x_columns_;

    // 继承父节点的所有 Arc 约束
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

    // 添加新的分支约束 (Arc >= ceil)
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

// 选择待分支节点
// 策略: 选择下界最小的未剪枝未分支节点 (Best-First Search)
// 这种策略有助于更快地找到最优解并剪枝
BPNode* SelectBranchNode(BPNode* head) {
    BPNode* best = nullptr;
    double best_lb = INFINITY;

    // 遍历节点链表
    BPNode* curr = head;
    while (curr != nullptr) {
        // 条件: 未剪枝 且 未分支
        if (curr->prune_flag_ == 0 && curr->branched_flag_ == 0) {
            // 选择下界最小的节点
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
// 输入: 根节点 (已完成列生成)
// 输出: 最优整数解存储在 params 中
int RunBranchAndPrice(ProblemParams& params, ProblemData& data, BPNode* root) {
    LOG("[BP] 分支定价开始 (Arc 分支策略)");
    LOG_FMT("[BP] 时间限制: %d 秒\n", kMaxBPTimeSec);

    // 记录开始时间
    auto bp_start_time = chrono::high_resolution_clock::now();

    // 确保 Arc Flow 网络已生成
    // 分支定价需要将解转换为 Arc 流量
    if (data.sp1_arc_data_.arc_list_.empty()) {
        GenerateAllArcs(data, params);
    }

    // 为根节点解生成 Arc 集合
    // 用于后续的 Arc 流量分析
    GenerateYArcSetMatrix(*root, data.strip_widths_);
    for (int j = 0; j < params.num_strip_types_; j++) {
        GenerateXArcSetMatrix(*root, data.item_lengths_, j);
    }

    // 初始化节点链表 (用于存储所有节点)
    BPNode* head = root;
    BPNode* tail = root;
    int node_count = 1;

    // 检查根节点是否已经是整数解
    int branch_type = SelectBranchArc(params, data, root);
    if (branch_type == kBranchNone) {
        // Arc 流量全整数，根节点即为最优解
        params.global_best_int_ = root->solution_.obj_val_;
        params.global_best_y_cols_ = root->solution_.y_columns_;
        params.global_best_x_cols_ = root->solution_.x_columns_;
        LOG("[BP] 根节点 Arc 流量全整数, 即为最优解");
        CONSOLE_FMT("[BP] 根节点即整数解 obj=%.0f\n", params.global_best_int_);
        return 0;
    }

    // 分支定价主循环
    int loop_iter = 0;
    while (true) {
        loop_iter++;

        // 超时检查
        if (IsTimeUp(params)) {
            params.is_timeout_ = true;
            LOG_FMT("[BP] 达到时间限制 (%d秒), 终止搜索\n", params.time_limit_);
            LOG_FMT("[BP] 当前状态: 已探索 %d 个节点\n", params.node_counter_);
            if (params.global_best_int_ < INFINITY) {
                LOG_FMT("[BP] 当前最优整数解: %.0f\n", params.global_best_int_);
            } else {
                LOG("[BP] 尚未找到整数解");
            }
            break;
        }

        // 选择待分支节点 (下界最小的未处理节点)
        BPNode* parent = SelectBranchNode(head);

        // 检查终止条件: 无可分支节点
        if (parent == nullptr) {
            LOG("[BP] 无可分支节点, 搜索完成");
            break;
        }

        LOG_FMT("[BP] 选择节点 %d 进行分支 (LB=%.4f)\n",
            parent->id_, parent->lower_bound_);

        // 控制台进度: 每个节点都输出
        double lb = parent->lower_bound_;
        double ub = params.global_best_int_;
        double gap = (ub < INFINITY && lb > 0) ? (ub - lb) / ub * 100 : 0;
        CONSOLE_FMT("[BP] n=%d | LB=%.2f UB=%.0f Gap=%.1f%%\n",
            node_count, lb, ub, gap);

        // 创建并求解左子节点
        BPNode* left = new BPNode();
        node_count++;
        params.node_counter_++;
        CreateLeftChild(parent, params.node_counter_, left);

        // 求解左子节点的列生成
        // 子问题会应用该节点累积的 Arc 约束
        SolveNodeCG(params, data, left);

        // 将左子节点加入链表
        tail->next_ = left;
        tail = left;

        // 检查左子节点
        if (left->prune_flag_ == 0) {
            // 节点可行，为解生成 Arc 集合
            GenerateYArcSetMatrix(*left, data.strip_widths_);
            for (int j = 0; j < params.num_strip_types_; j++) {
                GenerateXArcSetMatrix(*left, data.item_lengths_, j);
            }

            // 检查是否为整数解
            int left_branch_type = SelectBranchArc(params, data, left);
            if (left_branch_type == kBranchNone) {
                // Arc 流量全整数，找到一个整数解
                if (left->solution_.obj_val_ < params.global_best_int_) {
                    // 更新全局最优整数解
                    params.global_best_int_ = left->solution_.obj_val_;
                    params.global_best_y_cols_ = left->solution_.y_columns_;
                    params.global_best_x_cols_ = left->solution_.x_columns_;
                    LOG_FMT("[BP] 找到新整数解, 目标值=%.4f\n", params.global_best_int_);
                }
                left->branched_flag_ = 1;  // 整数解无需再分支
            }
            // 否则 left->branch_type_ 已由 SelectBranchArc 设置，等待后续分支
        }

        // 创建并求解右子节点
        BPNode* right = new BPNode();
        node_count++;
        params.node_counter_++;
        CreateRightChild(parent, params.node_counter_, right);

        SolveNodeCG(params, data, right);

        // 将右子节点加入链表
        tail->next_ = right;
        tail = right;

        // 检查右子节点
        if (right->prune_flag_ == 0) {
            GenerateYArcSetMatrix(*right, data.strip_widths_);
            for (int j = 0; j < params.num_strip_types_; j++) {
                GenerateXArcSetMatrix(*right, data.item_lengths_, j);
            }

            int right_branch_type = SelectBranchArc(params, data, right);
            if (right_branch_type == kBranchNone) {
                if (right->solution_.obj_val_ < params.global_best_int_) {
                    params.global_best_int_ = right->solution_.obj_val_;
                    params.global_best_y_cols_ = right->solution_.y_columns_;
                    params.global_best_x_cols_ = right->solution_.x_columns_;
                    LOG_FMT("[BP] 找到新整数解, 目标值=%.4f\n", params.global_best_int_);
                }
                right->branched_flag_ = 1;
            }
        }

        // 标记父节点已完成分支
        parent->branched_flag_ = 1;

        // 剪枝: 遍历所有节点，剪掉下界 >= 全局最优整数解的节点
        // 这些节点不可能产生更优的整数解
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

        // 检查时间限制 (主要控制手段)
        auto current_time = chrono::high_resolution_clock::now();
        auto elapsed = chrono::duration_cast<chrono::seconds>(
            current_time - bp_start_time).count();
        if (elapsed >= kMaxBPTimeSec) {
            LOG_FMT("[BP] 达到时间限制 (%d 秒), 终止搜索\n", kMaxBPTimeSec);
            break;
        }

        // 检查节点数限制 (可选的安全阀)
        if (kMaxBPNodes > 0 && node_count > kMaxBPNodes) {
            LOG_FMT("[BP] 达到最大节点数 (%d), 强制终止\n", kMaxBPNodes);
            break;
        }
    }

    // 如果未找到整数解，使用根节点 LP 解的向上取整作为可行解
    if (params.global_best_int_ >= INFINITY) {
        LOG("[BP] 未找到整数解, 使用根节点 LP 解向上取整");

        // 对 Y 列取整 (向上取整确保可行性)
        params.global_best_y_cols_ = root->solution_.y_columns_;
        double rounded_obj = 0.0;
        for (auto& y_col : params.global_best_y_cols_) {
            if (y_col.value_ > kZeroTolerance) {
                y_col.value_ = ceil(y_col.value_);
                rounded_obj += y_col.value_;
            }
        }

        // 对 X 列取整
        params.global_best_x_cols_ = root->solution_.x_columns_;
        for (auto& x_col : params.global_best_x_cols_) {
            if (x_col.value_ > kZeroTolerance) {
                x_col.value_ = ceil(x_col.value_);
            }
        }

        params.global_best_int_ = rounded_obj;
        LOG_FMT("[BP] 取整后目标值: %.4f (非最优)\n", params.global_best_int_);
    }

    // 计算最优性间隙
    // gap = (UB - LB) / UB，其中 LB 是所有未剪枝节点下界的最小值
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
