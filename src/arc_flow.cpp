// =============================================================================
// arc_flow.cpp - Arc Flow网络生成
// =============================================================================

#include "2DBP.h"

using namespace std;

// 生成SP1的Arc Flow网络 (宽度方向)
// 节点: 母板宽度方向上的位置 (0到stock_width)
// Arc: 放置一种条带类型, Arc长度等于条带宽度
void GenerateSP1Arcs(ProblemData& data, ProblemParams& params) {
    LOG("[Arc Flow] 生成SP1网络 (宽度方向)");

    SP1ArcFlowData& arc_data = data.sp1_arc_data_;
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

    // 收集所有可能的节点位置
    set<int> node_set;
    node_set.insert(0);
    node_set.insert(stock_width);

    // 生成所有可能的Arc
    for (int start = 0; start <= stock_width; start++) {
        for (int j = 0; j < num_strip_types; j++) {
            int strip_width = data.strip_types_[j].width_;
            int end = start + strip_width;

            if (end <= stock_width) {
                array<int, 2> arc = {start, end};
                if (arc_data.arc_to_index_.find(arc) == arc_data.arc_to_index_.end()) {
                    int idx = static_cast<int>(arc_data.arc_list_.size());
                    arc_data.arc_to_index_[arc] = idx;
                    arc_data.arc_list_.push_back(arc);
                    node_set.insert(start);
                    node_set.insert(end);
                }
            }
        }
    }

    // 分类节点
    arc_data.begin_nodes_.push_back(0);
    arc_data.end_nodes_.push_back(stock_width);

    for (int node : node_set) {
        if (node != 0 && node != stock_width) {
            arc_data.mid_nodes_.push_back(node);
        }
    }

    // 分类Arc
    int num_mid = static_cast<int>(arc_data.mid_nodes_.size());
    arc_data.mid_in_arcs_.resize(num_mid);
    arc_data.mid_out_arcs_.resize(num_mid);

    for (int idx = 0; idx < static_cast<int>(arc_data.arc_list_.size()); idx++) {
        int start = arc_data.arc_list_[idx][0];
        int end = arc_data.arc_list_[idx][1];

        // 起点Arc
        if (start == 0) {
            arc_data.begin_arc_indices_.push_back(idx);
        }

        // 终点Arc
        if (end == stock_width) {
            arc_data.end_arc_indices_.push_back(idx);
        }

        // 中间节点的入弧和出弧
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

    LOG_FMT("  节点数: %d (起点1, 终点1, 中间%d)\n",
        (int)node_set.size(), num_mid);
    LOG_FMT("  Arc数: %d\n", (int)arc_data.arc_list_.size());
}

// 生成SP2的Arc Flow网络 (长度方向)
// 节点: 条带长度方向上的位置 (0到stock_length)
// Arc: 放置一种子件, Arc长度等于子件长度
// 仅考虑宽度匹配的子件类型
void GenerateSP2Arcs(ProblemData& data, ProblemParams& params, int strip_type_id) {
    LOG_FMT("[Arc Flow] 生成SP2网络 (条带类型%d)\n", strip_type_id);

    // 确保sp2_arc_data_有足够空间
    while ((int)data.sp2_arc_data_.size() <= strip_type_id) {
        data.sp2_arc_data_.push_back(SP2ArcFlowData());
    }

    SP2ArcFlowData& arc_data = data.sp2_arc_data_[strip_type_id];
    arc_data.strip_type_id_ = strip_type_id;
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
    set<int> node_set;
    node_set.insert(0);
    node_set.insert(stock_length);

    // 生成所有可能的Arc (仅考虑宽度匹配的子件)
    for (int start = 0; start <= stock_length; start++) {
        for (int i = 0; i < num_item_types; i++) {
            int item_width = data.item_types_[i].width_;
            int item_length = data.item_types_[i].length_;

            // 子件宽度必须小于等于条带宽度
            if (item_width <= strip_width) {
                int end = start + item_length;

                if (end <= stock_length) {
                    array<int, 2> arc = {start, end};
                    if (arc_data.arc_to_index_.find(arc) == arc_data.arc_to_index_.end()) {
                        int idx = static_cast<int>(arc_data.arc_list_.size());
                        arc_data.arc_to_index_[arc] = idx;
                        arc_data.arc_list_.push_back(arc);
                        node_set.insert(start);
                        node_set.insert(end);
                    }
                }
            }
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

    // 分类Arc
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

    LOG_FMT("  节点数: %d, Arc数: %d\n",
        (int)node_set.size(), (int)arc_data.arc_list_.size());
}

// 生成所有Arc Flow网络
void GenerateAllArcs(ProblemData& data, ProblemParams& params) {
    LOG("[Arc Flow] 生成所有网络");

    // SP1网络 (宽度方向)
    GenerateSP1Arcs(data, params);

    // SP2网络 (每种条带类型一个)
    for (int j = 0; j < params.num_strip_types_; j++) {
        GenerateSP2Arcs(data, params, j);
    }

    LOG("[Arc Flow] 网络生成完成");
}

// 将切割方案转换为Arc集合
// pattern: 每种类型的数量
// sizes: 对应的尺寸列表 (长度或宽度)
// arc_set: 输出的Arc集合
void ConvertPatternToArcSet(vector<int>& pattern, vector<int>& sizes,
    set<array<int, 2>>& arc_set) {

    arc_set.clear();
    int pos = 0;

    for (int i = 0; i < (int)pattern.size(); i++) {
        for (int k = 0; k < pattern[i]; k++) {
            int end = pos + sizes[i];
            arc_set.insert({pos, end});
            pos = end;
        }
    }
}

// 生成Y列的Arc集合矩阵
void GenerateYArcSetMatrix(BPNode& node, vector<int>& strip_widths) {
    node.y_arc_sets_.clear();

    for (int col = 0; col < (int)node.y_columns_.size(); col++) {
        set<array<int, 2>> arc_set;
        ConvertPatternToArcSet(node.y_columns_[col].pattern_, strip_widths, arc_set);
        node.y_arc_sets_.push_back(arc_set);
        node.y_columns_[col].arc_set_ = arc_set;
    }
}

// 生成X列的Arc集合矩阵
void GenerateXArcSetMatrix(BPNode& node, vector<int>& item_lengths, int strip_type) {
    for (int col = 0; col < (int)node.x_columns_.size(); col++) {
        if (node.x_columns_[col].strip_type_id_ == strip_type) {
            set<array<int, 2>> arc_set;
            ConvertPatternToArcSet(node.x_columns_[col].pattern_, item_lengths, arc_set);
            node.x_arc_sets_.push_back(arc_set);
            node.x_columns_[col].arc_set_ = arc_set;
        }
    }
}

// =============================================================================
// Arc Flow 解转换函数
// =============================================================================

// 将 Y 列解转换为 SP1 Arc 流量 (宽度方向)
// 输入: Y 列集合 (每列有 pattern 和 value)
// 输出: arc_flow_solution[arc_idx] = (起点, 终点, 总流量)
void ConvertYColsToSP1ArcFlow(vector<YColumn>& y_columns, ProblemData& data,
    map<int, tuple<int, int, double>>& arc_flow_solution) {

    arc_flow_solution.clear();
    SP1ArcFlowData& arc_data = data.sp1_arc_data_;

    for (int col = 0; col < (int)y_columns.size(); col++) {
        double col_value = y_columns[col].value_;
        if (col_value < kZeroTolerance) continue;

        // 将该列的 pattern 转换为 Arc 集合
        set<array<int, 2>>& arc_set = y_columns[col].arc_set_;

        // 如果 arc_set 为空, 根据 pattern 生成
        if (arc_set.empty()) {
            ConvertPatternToArcSet(y_columns[col].pattern_,
                data.strip_widths_, arc_set);
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

// 将 X 列解转换为 SP2 Arc 流量 (长度方向, 指定条带类型)
// 输入: X 列集合, 条带类型 ID
// 输出: arc_flow_solution[arc_idx] = (起点, 终点, 总流量)
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

        // 将该列的 pattern 转换为 Arc 集合
        set<array<int, 2>>& arc_set = x_columns[col].arc_set_;

        // 如果 arc_set 为空, 根据 pattern 生成
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

// =============================================================================
// 分支 Arc 选择函数
// =============================================================================

// 在 SP1 Arc 流量中找非整数 Arc
// 返回: true = 找到非整数 Arc, false = 全部整数
bool FindBranchArcSP1(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow) {

    double max_frac = 0.0;
    bool found = false;

    for (const auto& item : arc_flow_solution) {
        double flow = get<2>(item.second);

        // 精度处理: 接近整数的流量视为整数
        double frac = flow - floor(flow);
        if (frac > 0.9999) frac = 0.0;
        if (frac < 0.0001) frac = 0.0;

        // 找分数部分最大的 Arc (最远离整数)
        if (frac > kZeroTolerance && frac < 1 - kZeroTolerance) {
            double dist_to_half = fabs(frac - 0.5);
            double score = 0.5 - dist_to_half;  // 越接近 0.5 得分越高

            if (score > max_frac || !found) {
                max_frac = score;
                branch_arc[0] = get<0>(item.second);
                branch_arc[1] = get<1>(item.second);
                branch_flow = flow;
                found = true;
            }
        }
    }

    return found;
}

// 在 SP2 Arc 流量中找非整数 Arc
// 返回: true = 找到非整数 Arc, false = 全部整数
bool FindBranchArcSP2(map<int, tuple<int, int, double>>& arc_flow_solution,
    array<int, 2>& branch_arc, double& branch_flow) {

    // 逻辑与 SP1 相同
    return FindBranchArcSP1(arc_flow_solution, branch_arc, branch_flow);
}

// =============================================================================
// 打印函数
// =============================================================================

// 打印 SP1 Arc Flow 解 (宽度方向)
void PrintSP1ArcFlowSolution(map<int, tuple<int, int, double>>& solution) {
    LOG("[SP1 Arc Flow 解]");
    for (const auto& item : solution) {
        int arc_idx = item.first;
        auto arc_info = item.second;
        double flow = get<2>(arc_info);

        // 只打印非零流量
        if (flow > kZeroTolerance) {
            LOG_FMT("  Arc %d: [%d,%d] 流量=%.4f\n",
                arc_idx, get<0>(arc_info), get<1>(arc_info), flow);
        }
    }
}

// 打印 SP2 Arc Flow 解 (长度方向)
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
