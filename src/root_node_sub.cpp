// root_node_sub.cpp - 根节点子问题求解函数
//
// 本文件实现根节点的子问题求解, 用于列生成中寻找改进列 (reduced cost > 0)
//
// 二维下料问题的子问题分为两个层次:
// - SP1 (宽度方向): 选择条带放置在母板上, 最大化条带价值总和
//   数学模型: max sum(v_j * G_j), s.t. sum(w_j * G_j) <= W
//   其中 v_j 是条带平衡约束的对偶价格, w_j 是条带宽度, W 是母板宽度
//
// - SP2 (长度方向): 选择子板放置在条带上, 最大化子板价值总和
//   数学模型: max sum(pi_i * D_i), s.t. sum(l_i * D_i) <= L
//   其中 pi_i 是需求约束的对偶价格, l_i 是子板长度, L 是条带长度(=母板长度)
//
// 每个子问题支持三种求解方法:
// - CPLEX IP (Knapsack): 直接建模为整数背包问题
// - Arc Flow: 建模为网络流, 利用路径结构
// - DP: 动态规划, 适用于离散容量较小的情况
//
// 列生成收敛条件:
// - SP1: reduced_cost <= 1 (Y列系数为1)
// - SP2: reduced_cost <= v_j (X列系数为v_j, 条带的对偶价格)

#include "2DBP.h"

using namespace std;

// SP1子问题: 宽度方向背包问题 - CPLEX整数规划求解
// 功能: 寻找能改进主问题目标值的新Y列 (母板切割模式)
// 数学模型:
//   max sum_{j} v_j * G_j           (最大化条带价值, v_j为对偶价格)
//   s.t. sum_{j} w_j * G_j <= W     (条带宽度之和不超过母板宽度)
//        G_j >= 0, 整数             (G_j为j型条带的切割数量)
// 列生成判断:
//   - reduced_cost = obj_value, Y列在目标函数系数为1
//   - 若 reduced_cost > 1, 说明该列能改进目标, 应加入主问题
//   - 若 reduced_cost <= 1, 说明无改进列, 列生成收敛
// 返回值: true=列生成收敛, false=找到改进列
bool SolveRootSP1Knapsack(ProblemParams& params, ProblemData& data, BPNode& node) {
    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);
    int num_strip_types = params.num_strip_types_;

    // 构建目标函数: max sum(v_j * G_j)
    // v_j 是第j条条带平衡约束的对偶价格, 反映条带的边际价值
    IloExpr obj_expr(env);
    for (int j = 0; j < num_strip_types; j++) {
        // G_j: 第j型条带在该母板模式中的切割数量
        string var_name = "G_" + to_string(j + 1);
        IloNumVar var(env, 0, IloInfinity, ILOINT, var_name.c_str());
        vars.add(var);

        double dual = node.duals_[j];  // 取条带平衡约束对偶价格
        if (dual != 0.0) {             // 跳过零系数项以提高求解效率
            obj_expr += vars[j] * dual;
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    // 宽度约束: sum(w_j * G_j) <= W
    // 所有切割条带的宽度总和不能超过母板宽度
    IloExpr wid_expr(env);
    for (int j = 0; j < num_strip_types; j++) {
        wid_expr += data.strip_types_[j].width_ * vars[j];
    }
    model.add(wid_expr <= params.stock_width_);
    wid_expr.end();

    // 调用CPLEX求解背包问题
    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (背包)\n", node.iter_, node.id_);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());  // 关闭CPLEX输出
    bool feasible = cplex.solve();

    bool cg_converged = true;  // 默认假设收敛

    if (feasible) {
        double rc = cplex.getObjValue();  // reduced cost = 目标值
        LOG_FMT("  [SP1] Reduced Cost: %.4f\n", rc);

        // 提取解向量, 构建新Y列的pattern
        // pattern[j] 表示该母板模式切割j型条带的数量
        node.new_y_col_.pattern_.clear();
        for (int j = 0; j < num_strip_types; j++) {
            double val = cplex.getValue(vars[j]);
            // 浮点取整: 处理0.99999这类数值误差
            int int_val = (val - (int)val > 0.99999) ? (int)val + 1 : (int)val;
            node.new_y_col_.pattern_.push_back(int_val);
        }

        // 判断是否找到改进列
        // Y列的目标函数系数为1 (每使用一个母板, 目标值+1)
        // 若 rc > 1, 说明该列的对偶收益超过成本, 能改进目标
        if (rc > 1 + kRcTolerance) {
            cg_converged = false;  // 找到改进列, 继续迭代
            LOG("  [SP1] 找到改进列");
        } else {
            cg_converged = true;   // 无改进列, 收敛
            node.new_y_col_.pattern_.clear();  // 清空无效的pattern
            LOG("  [SP1] 收敛");
        }
    } else {
        // 背包问题不可行通常不应发生 (空模式总是可行的)
        LOG("  [SP1] 子问题不可行");
    }

    // 释放CPLEX资源
    obj_expr.end();
    obj.end();
    vars.end();
    cplex.end();
    model.end();
    env.end();

    return cg_converged;
}

// SP1子问题: 宽度方向背包问题 - Arc Flow网络流模型求解
// 功能: 将背包问题建模为网络流, 利用Arc流量进行分支
// Arc Flow网络结构:
//   - 节点: 0, 1, 2, ..., W (表示母板宽度的使用位置)
//   - Arc (i, j): 在位置i放置宽度为(j-i)的条带, j-i对应某种条带宽度
//   - 起点节点0表示母板起始位置, 终点节点W表示母板末端
//   - 选中的Arc形成一条从0到某节点<=W的路径, 代表一种切割模式
// Arc变量与分支:
//   - 每条Arc对应一个0-1变量, 表示是否在该位置切割该条带
//   - 分支时可对某Arc的流量添加约束 (禁用/上界/下界)
// 返回值: true=列生成收敛, false=找到改进列
bool SolveRootSP1ArcFlow(ProblemParams& params, ProblemData& data, BPNode& node) {
    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);

    SP1ArcFlowData& arc_data = data.sp1_arc_data_;
    int num_arcs = static_cast<int>(arc_data.arc_list_.size());
    int num_strip_types = params.num_strip_types_;

    // 为每个Arc创建0-1整数变量
    // 目标函数: max sum(v_j * a_i), v_j为Arc对应条带的对偶价格
    IloExpr obj_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        string var_name = "a_" + to_string(i + 1);
        IloNumVar var(env, 0, 1, ILOINT, var_name.c_str());
        vars.add(var);

        // Arc的长度(终点-起点)对应条带宽度
        int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        int strip_idx = data.width_to_strip_index_[arc_width];  // 宽度映射到条带索引
        double dual = node.duals_[strip_idx];  // 该条带的对偶价格

        if (dual != 0.0) {  // 跳过零系数项
            obj_expr += vars[i] * dual;
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    // 宽度约束: 选中Arc的总长度不超过母板宽度
    // 这是一个冗余约束 (网络结构已保证), 但可加速求解
    IloExpr wid_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        wid_expr += arc_width * vars[i];
    }
    model.add(wid_expr <= params.stock_width_);
    wid_expr.end();

    // 起点约束: 从节点0出发的流量 = 1
    // 确保路径从节点0开始
    IloExpr begin_expr(env);
    for (int idx : arc_data.begin_arc_indices_) {
        begin_expr += vars[idx];
    }
    model.add(begin_expr == 1);
    begin_expr.end();

    // 终点约束: 进入终点节点的流量 = 1
    // 确保路径到达某个终点节点 (允许浪费部分宽度)
    IloExpr end_expr(env);
    for (int idx : arc_data.end_arc_indices_) {
        end_expr += vars[idx];
    }
    model.add(end_expr == 1);
    end_expr.end();

    // 中间节点流量守恒: 流入量 = 流出量
    // 确保形成一条连通的路径
    int num_mid = static_cast<int>(arc_data.mid_nodes_.size());
    for (int i = 0; i < num_mid; i++) {
        IloExpr in_expr(env);   // 流入该节点的Arc
        IloExpr out_expr(env);  // 流出该节点的Arc
        for (int idx : arc_data.mid_in_arcs_[i]) {
            in_expr += vars[idx];
        }
        for (int idx : arc_data.mid_out_arcs_[i]) {
            out_expr += vars[idx];
        }
        model.add(in_expr == out_expr);
        in_expr.end();
        out_expr.end();
    }

    // 添加分支约束: 这是Arc Flow分支策略的核心
    // 从父节点继承的Arc约束会限制子问题的可行域

    // 禁用Arc约束: 设置上界为0, 完全禁止使用该Arc
    // 通常来自左分支 (floor(v)=0的情况)
    for (const auto& arc : node.sp1_zero_arcs_) {
        if (arc_data.arc_to_index_.count(arc)) {  // 确保Arc存在于网络中
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setUB(0);  // 禁用该Arc
        }
    }

    // Arc上界约束: Arc流量 <= N (左分支, floor(v) > 0)
    for (size_t i = 0; i < node.sp1_lower_arcs_.size(); i++) {
        const auto& arc = node.sp1_lower_arcs_[i];
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setUB(node.sp1_lower_bounds_[i]);
        }
    }

    // Arc下界约束: Arc流量 >= N (右分支, ceil(v))
    for (size_t i = 0; i < node.sp1_greater_arcs_.size(); i++) {
        const auto& arc = node.sp1_greater_arcs_[i];
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setLB(node.sp1_greater_bounds_[i]);
        }
    }

    // 求解Arc Flow子问题
    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (Arc Flow)\n", node.iter_, node.id_);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        LOG_FMT("  [SP1] Reduced Cost: %.4f\n", rc);

        // 根据选中的Arc构建切割模式
        // 遍历所有Arc, 统计每种条带被选中的次数
        vector<int> pattern(num_strip_types, 0);
        for (int i = 0; i < num_arcs; i++) {
            double val = cplex.getValue(vars[i]);
            if (val > 0.5) {  // Arc被选中 (二值变量, 用0.5判断)
                int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
                int strip_idx = data.width_to_strip_index_[arc_width];
                pattern[strip_idx]++;  // 该条带类型数量+1
            }
        }

        // 判断是否找到改进列
        if (rc > 1 + kRcTolerance) {
            cg_converged = false;
            node.new_y_col_.pattern_ = pattern;  // 保存新Y列
            LOG("  [SP1] 找到改进列");
        } else {
            cg_converged = true;
            LOG("  [SP1] 收敛");
        }
    } else {
        // Arc Flow不可行可能由于分支约束过紧
        LOG("  [SP1] 子问题不可行");
    }

    // 释放CPLEX资源
    obj_expr.end();
    obj.end();
    vars.end();
    cplex.end();
    model.end();
    env.end();

    return cg_converged;
}

// SP1: 宽度背包问题 - 动态规划求解
bool SolveRootSP1DP(ProblemParams& params, ProblemData& data, BPNode& node) {
    int num_strip_types = params.num_strip_types_;
    int W = params.stock_width_;

    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (DP)\n", node.iter_, node.id_);

    // dp[w] = (最大价值, 方案)
    vector<double> dp(W + 1, 0.0);
    vector<vector<int>> choice(W + 1, vector<int>(num_strip_types, 0));

    // 完全背包DP
    for (int j = 0; j < num_strip_types; j++) {
        int wid = data.strip_types_[j].width_;
        double val = node.duals_[j];

        if (val <= 0) continue;

        for (int w = wid; w <= W; w++) {
            if (dp[w - wid] + val > dp[w]) {
                dp[w] = dp[w - wid] + val;
                choice[w] = choice[w - wid];
                choice[w][j]++;
            }
        }
    }

    double rc = dp[W];
    LOG_FMT("  [SP1] Reduced Cost: %.4f\n", rc);

    if (rc > 1 + kRcTolerance) {
        node.new_y_col_.pattern_ = choice[W];
        LOG("  [SP1] 找到改进列");
        return false;
    } else {
        LOG("  [SP1] 收敛");
        return true;
    }
}

// SP2: 长度背包问题 - CPLEX整数规划求解
// 目标: max sum(pi_i * D_i), 约束: sum(l_i * D_i) <= L
bool SolveRootSP2Knapsack(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id) {

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);
    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;
    int strip_width = data.strip_types_[strip_type_id].width_;

    // 构建目标函数: max sum(pi_i * D_i)
    IloExpr obj_expr(env);
    double sum_coef = 0;

    for (int i = 0; i < num_item_types; i++) {
        string var_name = "D_" + to_string(i + 1);
        IloNumVar var(env, 0, IloInfinity, ILOINT, var_name.c_str());
        vars.add(var);

        // 只考虑宽度匹配的子件
        int item_width = data.item_types_[i].width_;
        if (item_width <= strip_width) {
            double dual = node.duals_[num_strip_types + i];
            if (dual > 0) {
                obj_expr += vars[i] * dual;
                sum_coef += dual;
            }
        }
    }

    // 无正系数子件, 无需求解
    if (sum_coef <= 0) {
        obj_expr.end();
        vars.end();
        model.end();
        env.end();
        return true;
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    // 长度约束: sum(l_i * D_i) <= L
    IloExpr len_expr(env);
    for (int i = 0; i < num_item_types; i++) {
        if (data.item_types_[i].width_ <= strip_width) {
            len_expr += data.item_types_[i].length_ * vars[i];
        }
    }
    model.add(len_expr <= params.stock_length_);
    len_expr.end();

    // 求解
    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (背包)\n", node.iter_, strip_type_id);

    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        double dual_v = node.duals_[strip_type_id];
        LOG_FMT("  [SP2] Reduced Cost: %.4f (v_j=%.4f)\n", rc - dual_v, dual_v);

        if (rc > dual_v + kRcTolerance) {
            cg_converged = false;

            // 提取解
            // 只对宽度匹配的变量获取值, 其他变量的值为0
            // 宽度不匹配的变量不在模型中, 调用getValue会崩溃
            node.new_x_col_.pattern_.clear();
            for (int i = 0; i < num_item_types; i++) {
                int int_val = 0;
                if (data.item_types_[i].width_ <= strip_width) {
                    double val = cplex.getValue(vars[i]);
                    int_val = (val - (int)val > 0.99999) ? (int)val + 1 : (int)val;
                }
                node.new_x_col_.pattern_.push_back(int_val);
            }
            node.new_strip_type_ = strip_type_id;
            LOG("  [SP2] 找到改进列");
        } else {
            cg_converged = true;
            LOG("  [SP2] 收敛");
        }
    }

    obj_expr.end();
    obj.end();
    vars.end();
    cplex.end();
    model.end();
    env.end();

    return cg_converged;
}

// SP2: 长度背包问题 - Arc Flow模型求解
bool SolveRootSP2ArcFlow(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id) {

    // 确保Arc网络已生成
    if ((int)data.sp2_arc_data_.size() <= strip_type_id) {
        GenerateSP2Arcs(data, params, strip_type_id);
    }

    SP2ArcFlowData& arc_data = data.sp2_arc_data_[strip_type_id];
    int num_arcs = static_cast<int>(arc_data.arc_list_.size());
    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;

    if (num_arcs == 0) {
        return true;  // 无可用Arc, 视为收敛
    }

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);

    // 为每个Arc创建0-1变量
    IloExpr obj_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        string var_name = "a_" + to_string(i + 1);
        IloNumVar var(env, 0, 1, ILOINT, var_name.c_str());
        vars.add(var);

        // Arc长度对应子件长度
        int arc_len = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        if (data.length_to_item_index_.count(arc_len)) {
            int item_idx = data.length_to_item_index_[arc_len];
            double dual = node.duals_[num_strip_types + item_idx];
            if (dual > 0) {
                obj_expr += vars[i] * dual;
            }
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    // 长度约束
    IloExpr len_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        int arc_len = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        len_expr += arc_len * vars[i];
    }
    model.add(len_expr <= params.stock_length_);
    len_expr.end();

    // 起点约束
    IloExpr begin_expr(env);
    for (int idx : arc_data.begin_arc_indices_) {
        begin_expr += vars[idx];
    }
    model.add(begin_expr == 1);
    begin_expr.end();

    // 终点约束
    IloExpr end_expr(env);
    for (int idx : arc_data.end_arc_indices_) {
        end_expr += vars[idx];
    }
    model.add(end_expr == 1);
    end_expr.end();

    // 中间节点流量守恒
    int num_mid = static_cast<int>(arc_data.mid_nodes_.size());
    for (int i = 0; i < num_mid; i++) {
        IloExpr in_expr(env);
        IloExpr out_expr(env);
        for (int idx : arc_data.mid_in_arcs_[i]) {
            in_expr += vars[idx];
        }
        for (int idx : arc_data.mid_out_arcs_[i]) {
            out_expr += vars[idx];
        }
        model.add(in_expr == out_expr);
        in_expr.end();
        out_expr.end();
    }

    // 添加 SP2 Arc 约束 (针对该条带类型)
    // 禁用 Arc
    if (node.sp2_zero_arcs_.count(strip_type_id)) {
        for (const auto& arc : node.sp2_zero_arcs_.at(strip_type_id)) {
            if (arc_data.arc_to_index_.count(arc)) {
                int idx = arc_data.arc_to_index_.at(arc);
                vars[idx].setUB(0);
            }
        }
    }
    // Arc <= N 约束
    if (node.sp2_lower_arcs_.count(strip_type_id)) {
        const auto& arcs = node.sp2_lower_arcs_.at(strip_type_id);
        const auto& bounds = node.sp2_lower_bounds_.at(strip_type_id);
        for (size_t i = 0; i < arcs.size(); i++) {
            if (arc_data.arc_to_index_.count(arcs[i])) {
                int idx = arc_data.arc_to_index_.at(arcs[i]);
                vars[idx].setUB(bounds[i]);
            }
        }
    }
    // Arc >= N 约束
    if (node.sp2_greater_arcs_.count(strip_type_id)) {
        const auto& arcs = node.sp2_greater_arcs_.at(strip_type_id);
        const auto& bounds = node.sp2_greater_bounds_.at(strip_type_id);
        for (size_t i = 0; i < arcs.size(); i++) {
            if (arc_data.arc_to_index_.count(arcs[i])) {
                int idx = arc_data.arc_to_index_.at(arcs[i]);
                vars[idx].setLB(bounds[i]);
            }
        }
    }

    // 求解
    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (Arc Flow)\n", node.iter_, strip_type_id);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        double dual_v = node.duals_[strip_type_id];

        if (rc > dual_v + kRcTolerance) {
            cg_converged = false;

            // 根据选中的Arc生成pattern
            vector<int> pattern(num_item_types, 0);
            for (int i = 0; i < num_arcs; i++) {
                double val = cplex.getValue(vars[i]);
                if (val > 0.5) {
                    int arc_len = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
                    if (data.length_to_item_index_.count(arc_len)) {
                        int item_idx = data.length_to_item_index_[arc_len];
                        pattern[item_idx]++;
                    }
                }
            }

            node.new_x_col_.pattern_ = pattern;
            node.new_strip_type_ = strip_type_id;
            LOG("  [SP2] 找到改进列");
        } else {
            cg_converged = true;
            LOG("  [SP2] 收敛");
        }
    }

    obj_expr.end();
    obj.end();
    vars.end();
    cplex.end();
    model.end();
    env.end();

    return cg_converged;
}

// SP2: 长度背包问题 - 动态规划求解
bool SolveRootSP2DP(ProblemParams& params, ProblemData& data,
    BPNode& node, int strip_type_id) {

    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;
    int L = params.stock_length_;
    int strip_width = data.strip_types_[strip_type_id].width_;

    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (DP)\n", node.iter_, strip_type_id);

    // dp[l] = (最大价值, 方案)
    vector<double> dp(L + 1, 0.0);
    vector<vector<int>> choice(L + 1, vector<int>(num_item_types, 0));

    // 完全背包DP
    for (int i = 0; i < num_item_types; i++) {
        // 只考虑宽度匹配的子件
        if (data.item_types_[i].width_ > strip_width) continue;

        int len = data.item_types_[i].length_;
        double val = node.duals_[num_strip_types + i];

        if (val <= 0) continue;

        for (int l = len; l <= L; l++) {
            if (dp[l - len] + val > dp[l]) {
                dp[l] = dp[l - len] + val;
                choice[l] = choice[l - len];
                choice[l][i]++;
            }
        }
    }

    double rc = dp[L];
    double dual_v = node.duals_[strip_type_id];
    LOG_FMT("  [SP2] Reduced Cost: %.4f (v_j=%.4f)\n", rc - dual_v, dual_v);

    if (rc > dual_v + kRcTolerance) {
        node.new_x_col_.pattern_ = choice[L];
        node.new_strip_type_ = strip_type_id;
        LOG("  [SP2] 找到改进列");
        return false;
    } else {
        LOG("  [SP2] 收敛");
        return true;
    }
}
