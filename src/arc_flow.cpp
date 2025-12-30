// arc_flow.cpp - Arc Flow 网络生成与解转换
//
// Arc Flow 模型将背包问题建模为网络流问题:
// - 节点表示已使用的容量 (0 到 capacity)
// - Arc (i, j) 表示在位置 i 放置一个尺寸为 (j-i) 的物品
// - 从节点 0 到节点 capacity 的路径对应一个可行的装载方案
//
// 优点: 便于添加 Arc 约束实现分支，如禁用某些 Arc 或限制 Arc 流量

#include "2DBP.h"

#include <climits>  // for INT_MAX

using namespace std;

// 生成 SP1 的 Arc Flow 网络 (宽度方向)
// 网络结构 (符合数学模型 Section 7):
//   节点: 母板宽度方向上的位置 (0 到 stock_width)
//   物品弧 A^{item}: 放置一种条带，Arc 长度等于条带宽度
//   损耗弧 A^{loss}: (t, t+1) 表示浪费1单位宽度，保证与背包等价
// 例如: Arc (10, 30) 表示在位置 10 放置一个宽度为 20 的条带
//       Arc (10, 11) 表示在位置 10 浪费 1 单位宽度
void GenerateSP1Arcs(ProblemData& data, ProblemParams& params) {
    LOG("[Arc Flow] 生成SP1网络 (宽度方向, 含loss arcs)");

    SP1ArcFlowData& arc_data = data.sp1_arc_data_;

    // 清空现有数据，准备重新生成
    arc_data.arc_list_.clear();
    arc_data.arc_to_index_.clear();
    arc_data.begin_nodes_.clear();
    arc_data.end_nodes_.clear();
    arc_data.mid_nodes_.clear();
    arc_data.begin_arc_indices_.clear();
    arc_data.end_arc_indices_.clear();
    arc_data.mid_in_arcs_.clear();
    arc_data.mid_out_arcs_.clear();

    int stock_width = params.stock_width_;
    int num_strip_types = params.num_strip_types_;

    // 使用 set 收集所有可能的节点位置
    // 有了 loss arcs 后，所有位置 0..stock_width 都是有效节点
    set<int> node_set;
    for (int t = 0; t <= stock_width; t++) {
        node_set.insert(t);
    }

    // 1. 生成物品弧 A^{item}
    // 对于每个起点位置 start，尝试放置每种条带
    int num_item_arcs = 0;
    for (int start = 0; start < stock_width; start++) {
        for (int j = 0; j < num_strip_types; j++) {
            int strip_width = data.strip_types_[j].width_;
            int end = start + strip_width;

            // 检查终点是否超出母板宽度
            if (end <= stock_width) {
                array<int, 2> arc = {start, end};

                // 避免重复添加相同的 Arc
                // 不同条带类型可能有相同宽度，会产生相同的 Arc
                if (arc_data.arc_to_index_.find(arc) == arc_data.arc_to_index_.end()) {
                    int idx = static_cast<int>(arc_data.arc_list_.size());
                    arc_data.arc_to_index_[arc] = idx;
                    arc_data.arc_list_.push_back(arc);
                    num_item_arcs++;
                }
            }
        }
    }

    // 2. 生成损耗弧 A^{loss} = {(t, t+1) : t = 0, ..., W-1}
    // 损耗弧保证 Arc-Flow 与 "<=" 背包严格等价
    int num_loss_arcs = 0;
    for (int t = 0; t < stock_width; t++) {
        array<int, 2> loss_arc = {t, t + 1};

        // 损耗弧可能与某些长度为1的物品弧重合，需检查
        if (arc_data.arc_to_index_.find(loss_arc) == arc_data.arc_to_index_.end()) {
            int idx = static_cast<int>(arc_data.arc_list_.size());
            arc_data.arc_to_index_[loss_arc] = idx;
            arc_data.arc_list_.push_back(loss_arc);
            num_loss_arcs++;
        }
    }

    // 将节点分为三类: 起点、终点、中间节点
    arc_data.begin_nodes_.push_back(0);
    arc_data.end_nodes_.push_back(stock_width);

    for (int node : node_set) {
        if (node != 0 && node != stock_width) {
            arc_data.mid_nodes_.push_back(node);
        }
    }

    // 为每个中间节点建立入弧和出弧列表
    // 用于后续构建流量守恒约束: 入弧流量之和 = 出弧流量之和
    int num_mid = static_cast<int>(arc_data.mid_nodes_.size());
    arc_data.mid_in_arcs_.resize(num_mid);
    arc_data.mid_out_arcs_.resize(num_mid);

    // 遍历所有 Arc，将其分类到对应的列表中
    for (int idx = 0; idx < static_cast<int>(arc_data.arc_list_.size()); idx++) {
        int start = arc_data.arc_list_[idx][0];
        int end = arc_data.arc_list_[idx][1];

        // 从起点 (位置 0) 出发的 Arc
        if (start == 0) {
            arc_data.begin_arc_indices_.push_back(idx);
        }

        // 到达终点 (位置 stock_width) 的 Arc
        if (end == stock_width) {
            arc_data.end_arc_indices_.push_back(idx);
        }

        // 对于中间节点，记录其入弧和出弧
        for (int i = 0; i < num_mid; i++) {
            int mid_node = arc_data.mid_nodes_[i];
            if (end == mid_node) {
                arc_data.mid_in_arcs_[i].push_back(idx);  // 该 Arc 进入中间节点 i
            }
            if (start == mid_node) {
                arc_data.mid_out_arcs_[i].push_back(idx); // 该 Arc 离开中间节点 i
            }
        }
    }

    LOG_FMT("  节点数: %d (起点1, 终点1, 中间%d)\n",
        (int)node_set.size(), num_mid);
    LOG_FMT("  Arc数: %d (物品弧%d, 损耗弧%d)\n",
        (int)arc_data.arc_list_.size(), num_item_arcs, num_loss_arcs);
}

// 生成 SP2 的 Arc Flow 网络 (长度方向)
// 每种条带类型有独立的 SP2 网络，因为不同条带可放置不同的子板
// 网络结构 (符合数学模型 Section 7.4):
//   节点: 条带长度方向上的位置 (0 到 stock_length)
//   物品弧 A^{item}: 放置一种子板，Arc 长度等于子板长度
//   损耗弧 A^{loss}: (t, t+1) 表示浪费1单位长度，保证与背包等价
// 约束: 只有宽度不超过条带宽度的子板才能放入该条带
void GenerateSP2Arcs(ProblemData& data, ProblemParams& params, int strip_type_id) {
    LOG_FMT("[Arc Flow] 生成SP2网络 (条带类型%d, 含loss arcs)\n", strip_type_id);

    // 确保 sp2_arc_data_ 数组足够大
    while ((int)data.sp2_arc_data_.size() <= strip_type_id) {
        data.sp2_arc_data_.push_back(SP2ArcFlowData());
    }

    SP2ArcFlowData& arc_data = data.sp2_arc_data_[strip_type_id];
    arc_data.strip_type_id_ = strip_type_id;

    // 清空现有数据
    arc_data.arc_list_.clear();
    arc_data.arc_to_index_.clear();
    arc_data.begin_nodes_.clear();
    arc_data.end_nodes_.clear();
    arc_data.mid_nodes_.clear();
    arc_data.begin_arc_indices_.clear();
    arc_data.end_arc_indices_.clear();
    arc_data.mid_in_arcs_.clear();
    arc_data.mid_out_arcs_.clear();

    int stock_length = params.stock_length_;
    int strip_width = data.strip_types_[strip_type_id].width_;
    int num_item_types = params.num_item_types_;

    // 收集所有可能的节点位置
    // 有了 loss arcs 后，所有位置 0..stock_length 都是有效节点
    set<int> node_set;
    for (int t = 0; t <= stock_length; t++) {
        node_set.insert(t);
    }

    // 1. 生成物品弧 A^{item} (仅考虑宽度匹配的子板)
    int num_item_arcs = 0;
    for (int start = 0; start < stock_length; start++) {
        for (int i = 0; i < num_item_types; i++) {
            int item_width = data.item_types_[i].width_;
            int item_length = data.item_types_[i].length_;

            // 关键条件: 子板宽度必须不超过条带宽度
            // 这是两阶段切割的约束，子板只能放入宽度匹配的条带
            if (item_width <= strip_width) {
                int end = start + item_length;

                // 检查终点是否超出条带长度
                if (end <= stock_length) {
                    array<int, 2> arc = {start, end};

                    // 避免重复添加相同的 Arc
                    if (arc_data.arc_to_index_.find(arc) == arc_data.arc_to_index_.end()) {
                        int idx = static_cast<int>(arc_data.arc_list_.size());
                        arc_data.arc_to_index_[arc] = idx;
                        arc_data.arc_list_.push_back(arc);
                        num_item_arcs++;
                    }
                }
            }
        }
    }

    // 2. 生成损耗弧 A^{loss} = {(t, t+1) : t = 0, ..., L-1}
    // 损耗弧保证 Arc-Flow 与 "<=" 背包严格等价
    int num_loss_arcs = 0;
    for (int t = 0; t < stock_length; t++) {
        array<int, 2> loss_arc = {t, t + 1};

        // 损耗弧可能与某些长度为1的物品弧重合，需检查
        if (arc_data.arc_to_index_.find(loss_arc) == arc_data.arc_to_index_.end()) {
            int idx = static_cast<int>(arc_data.arc_list_.size());
            arc_data.arc_to_index_[loss_arc] = idx;
            arc_data.arc_list_.push_back(loss_arc);
            num_loss_arcs++;
        }
    }

    // 分类节点
    arc_data.begin_nodes_.push_back(0);
    arc_data.end_nodes_.push_back(stock_length);

    for (int node : node_set) {
        if (node != 0 && node != stock_length) {
            arc_data.mid_nodes_.push_back(node);
        }
    }

    // 为中间节点建立入弧和出弧列表
    int num_mid = static_cast<int>(arc_data.mid_nodes_.size());
    arc_data.mid_in_arcs_.resize(num_mid);
    arc_data.mid_out_arcs_.resize(num_mid);

    for (int idx = 0; idx < static_cast<int>(arc_data.arc_list_.size()); idx++) {
        int start = arc_data.arc_list_[idx][0];
        int end = arc_data.arc_list_[idx][1];

        if (start == 0) {
            arc_data.begin_arc_indices_.push_back(idx);
        }

        if (end == stock_length) {
            arc_data.end_arc_indices_.push_back(idx);
        }

        for (int i = 0; i < num_mid; i++) {
            int mid_node = arc_data.mid_nodes_[i];
            if (end == mid_node) {
                arc_data.mid_in_arcs_[i].push_back(idx);
            }
            if (start == mid_node) {
                arc_data.mid_out_arcs_[i].push_back(idx);
            }
        }
    }

    LOG_FMT("  节点数: %d, Arc数: %d (物品弧%d, 损耗弧%d)\n",
        (int)node_set.size(), (int)arc_data.arc_list_.size(),
        num_item_arcs, num_loss_arcs);
}

// 生成所有 Arc Flow 网络
// 包括一个 SP1 网络和多个 SP2 网络 (每种条带类型一个)
void GenerateAllArcs(ProblemData& data, ProblemParams& params) {
    LOG("[Arc Flow] 生成所有网络");

    // SP1 网络 (宽度方向，母板 -> 条带)
    GenerateSP1Arcs(data, params);

    // SP2 网络 (长度方向，条带 -> 子板)
    // 每种条带类型有独立的网络
    for (int j = 0; j < params.num_strip_types_; j++) {
        GenerateSP2Arcs(data, params, j);
    }

    LOG("[Arc Flow] 网络生成完成");
}

// 将切割方案 (pattern) 转换为 Arc 集合
// pattern: 每种类型的数量，如 [2, 0, 1] 表示类型0放2个，类型1放0个，类型2放1个
// sizes: 对应的尺寸列表，如 [100, 80, 60]
// arc_set: 输出的 Arc 集合
// 转换逻辑: 按顺序放置物品，记录每个物品的起点和终点
void ConvertPatternToArcSet(vector<int>& pattern, vector<int>& sizes,
    set<array<int, 2>>& arc_set) {

    arc_set.clear();
    int pos = 0;  // 当前放置位置

    // 遍历每种类型
    for (int i = 0; i < (int)pattern.size(); i++) {
        // 放置 pattern[i] 个该类型的物品
        for (int k = 0; k < pattern[i]; k++) {
            int end = pos + sizes[i];
            arc_set.insert({pos, end});
            pos = end;
        }
    }
}

// 生成节点 Y 列的 Arc 集合矩阵
// 将每个 Y 列的 pattern 转换为对应的 SP1 Arc 集合
void GenerateYArcSetMatrix(BPNode& node, vector<int>& strip_widths) {
    node.y_arc_sets_.clear();

    for (int col = 0; col < (int)node.y_columns_.size(); col++) {
        set<array<int, 2>> arc_set;
        ConvertPatternToArcSet(node.y_columns_[col].pattern_, strip_widths, arc_set);
        node.y_arc_sets_.push_back(arc_set);
        node.y_columns_[col].arc_set_ = arc_set;
    }
}

// 生成节点 X 列的 Arc 集合矩阵
// 将指定条带类型的 X 列的 pattern 转换为对应的 SP2 Arc 集合
void GenerateXArcSetMatrix(BPNode& node, vector<int>& item_lengths, int strip_type) {
    for (int col = 0; col < (int)node.x_columns_.size(); col++) {
        // 只处理指定条带类型的 X 列
        if (node.x_columns_[col].strip_type_id_ == strip_type) {
            set<array<int, 2>> arc_set;
            ConvertPatternToArcSet(node.x_columns_[col].pattern_, item_lengths, arc_set);
            node.x_arc_sets_.push_back(arc_set);
            node.x_columns_[col].arc_set_ = arc_set;
        }
    }
}

// 将 Y 列 LP 解转换为 SP1 Arc 流量
// 输入: Y 列集合，每列有 pattern (切割方案) 和 value (LP 解值)
// 输出: arc_flow_solution[arc_idx] = (起点, 终点, 总流量)
// 转换逻辑: 每个 Y 列的解值贡献到其包含的所有 Arc
// 例如: 若 Y 列解值为 2.5，包含 Arc (0,30)，则该 Arc 流量增加 2.5
void ConvertYColsToSP1ArcFlow(vector<YColumn>& y_columns, ProblemData& data,
    map<int, tuple<int, int, double>>& arc_flow_solution) {

    arc_flow_solution.clear();
    SP1ArcFlowData& arc_data = data.sp1_arc_data_;

    for (int col = 0; col < (int)y_columns.size(); col++) {
        double col_value = y_columns[col].value_;

        // 跳过解值为 0 的列
        if (col_value < kZeroTolerance) continue;

        // 获取该列的 Arc 集合
        set<array<int, 2>>& arc_set = y_columns[col].arc_set_;

        // 如果 arc_set 为空，需要根据 pattern 生成
        if (arc_set.empty()) {
            ConvertPatternToArcSet(y_columns[col].pattern_,
                data.strip_widths_, arc_set);
        }

        // 将该列的解值累加到每个 Arc 的流量
        for (const auto& arc : arc_set) {
            // 检查该 Arc 是否在网络中
            if (arc_data.arc_to_index_.count(arc) == 0) continue;
            int arc_idx = arc_data.arc_to_index_[arc];

            if (arc_flow_solution.find(arc_idx) == arc_flow_solution.end()) {
                // 第一次遇到该 Arc，初始化流量
                arc_flow_solution[arc_idx] = make_tuple(arc[0], arc[1], col_value);
            } else {
                // 累加流量
                get<2>(arc_flow_solution[arc_idx]) += col_value;
            }
        }
    }
}

// 将 X 列 LP 解转换为 SP2 Arc 流量 (指定条带类型)
// 输入: X 列集合，条带类型 ID
// 输出: arc_flow_solution[arc_idx] = (起点, 终点, 总流量)
// 只处理属于指定条带类型的 X 列
void ConvertXColsToSP2ArcFlow(vector<XColumn>& x_columns, int strip_type_id,
    ProblemData& data, map<int, tuple<int, int, double>>& arc_flow_solution) {

    arc_flow_solution.clear();

    // 确保该条带类型的 Arc 网络已生成
    if ((int)data.sp2_arc_data_.size() <= strip_type_id) {
        return;
    }
    SP2ArcFlowData& arc_data = data.sp2_arc_data_[strip_type_id];

    for (int col = 0; col < (int)x_columns.size(); col++) {
        // 只处理该条带类型的 X 列
        if (x_columns[col].strip_type_id_ != strip_type_id) continue;

        double col_value = x_columns[col].value_;
        if (col_value < kZeroTolerance) continue;

        // 获取该列的 Arc 集合
        set<array<int, 2>>& arc_set = x_columns[col].arc_set_;

        // 如果 arc_set 为空，需要根据 pattern 生成
        if (arc_set.empty()) {
            ConvertPatternToArcSet(x_columns[col].pattern_,
                data.item_lengths_, arc_set);
        }

        // 累加每个 Arc 的流量
        for (const auto& arc : arc_set) {
            if (arc_data.arc_to_index_.count(arc) == 0) continue;
            int arc_idx = arc_data.arc_to_index_[arc];

            if (arc_flow_solution.find(arc_idx) == arc_flow_solution.end()) {
                arc_flow_solution[arc_idx] = make_tuple(arc[0], arc[1], col_value);
            } else {
                get<2>(arc_flow_solution[arc_idx]) += col_value;
            }
        }
    }
}

// 在 SP1 Arc 流量中寻找非整数流量的 Arc (用于分支)
// 候选弧选择准则 (符合数学模型 Section 9.2):
//   1. |frac(F_a) - 0.5| 最小 (最接近0.5)
//   2. |F_a| 最大 (tie-break)
//   3. 弧编号最小 (固定tie-break)
// 不对损耗弧分支 (损耗弧长度为1)
// 返回: true = 找到非整数 Arc，false = 所有 Arc 流量都是整数
bool FindBranchArcSP1(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow) {

    bool found = false;
    double best_dist_to_half = 1.0;  // 越小越好
    double best_flow_abs = 0.0;      // tie-break: 越大越好
    int best_arc_idx = INT_MAX;      // tie-break: 越小越好

    for (const auto& item : arc_flow_solution) {
        int arc_idx = item.first;
        int arc_start = get<0>(item.second);
        int arc_end = get<1>(item.second);
        double flow = get<2>(item.second);

        // 不对损耗弧分支 (损耗弧长度为1)
        int arc_length = arc_end - arc_start;
        if (arc_length == 1) continue;

        // 整数判别: |x - round(x)| <= kIntTolerance
        double rounded = round(flow);
        double frac_part = fabs(flow - rounded);

        // 检查是否为非整数流量
        if (frac_part > kIntTolerance) {
            // 计算小数部分 (用于0.5距离计算)
            double frac = flow - floor(flow);
            double dist_to_half = fabs(frac - 0.5);
            double flow_abs = fabs(flow);

            // 应用选择准则
            bool is_better = false;
            if (!found) {
                is_better = true;
            } else if (dist_to_half < best_dist_to_half - kZeroTolerance) {
                // 规则1: 更接近0.5
                is_better = true;
            } else if (fabs(dist_to_half - best_dist_to_half) <= kZeroTolerance) {
                // 规则2: 流量绝对值更大
                if (flow_abs > best_flow_abs + kZeroTolerance) {
                    is_better = true;
                } else if (fabs(flow_abs - best_flow_abs) <= kZeroTolerance) {
                    // 规则3: 弧编号更小
                    if (arc_idx < best_arc_idx) {
                        is_better = true;
                    }
                }
            }

            if (is_better) {
                found = true;
                best_dist_to_half = dist_to_half;
                best_flow_abs = flow_abs;
                best_arc_idx = arc_idx;
                branch_arc[0] = arc_start;
                branch_arc[1] = arc_end;
                branch_flow = flow;
            }
        }
    }

    return found;
}

// 在 SP2 Arc 流量中寻找非整数流量的 Arc
// 候选弧选择准则与 SP1 相同 (符合数学模型 Section 9.2)
// 不对损耗弧分支
bool FindBranchArcSP2(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow) {
    // 复用 SP1 的选择逻辑
    return FindBranchArcSP1(arc_flow_solution, branch_arc, branch_flow);
}

// 打印 SP1 Arc Flow 解 (调试用)
void PrintSP1ArcFlowSolution(map<int, tuple<int, int, double>>& solution) {
    LOG("[SP1 Arc Flow 解]");
    for (const auto& item : solution) {
        int arc_idx = item.first;
        auto arc_info = item.second;
        double flow = get<2>(arc_info);

        // 只打印非零流量的 Arc
        if (flow > kZeroTolerance) {
            LOG_FMT("  Arc %d: [%d,%d] 流量=%.4f\n",
                arc_idx, get<0>(arc_info), get<1>(arc_info), flow);
        }
    }
}

// 打印 SP2 Arc Flow 解 (调试用)
void PrintSP2ArcFlowSolution(map<int, tuple<int, int, double>>& solution, int strip_type) {
    LOG_FMT("[SP2 Arc Flow 解] 条带类型 %d\n", strip_type);
    for (const auto& item : solution) {
        int arc_idx = item.first;
        auto arc_info = item.second;
        double flow = get<2>(arc_info);

        if (flow > kZeroTolerance) {
            LOG_FMT("  Arc %d: [%d,%d] 流量=%.4f\n",
                arc_idx, get<0>(arc_info), get<1>(arc_info), flow);
        }
    }
}
