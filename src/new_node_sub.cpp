// new_node_sub.cpp - 非根节点子问题求解函数
//
// 本文件实现非根节点(分支节点)的子问题求解, 与根节点版本的主要区别:
// 1. 使用指针BPNode*而非引用, 便于树结构的管理
// 2. 继承父节点的Arc约束, 限制子问题的可行域
//
// 在Branch and Price过程中:
// - 根节点求解无约束的子问题, 生成初始列池
// - 分支后的子节点会累积Arc约束:
//   - sp1_zero_arcs_: 禁用的SP1 Arc (流量设为0)
//   - sp1_lower_arcs_/bounds_: SP1 Arc上界约束 (流量 <= N)
//   - sp1_greater_arcs_/bounds_: SP1 Arc下界约束 (流量 >= N)
//   - sp2_*: SP2子问题的对应约束 (按条带类型索引)
//
// 这些约束通过Arc分支策略从父节点继承, 逐步缩小可行域直至获得整数解

#include "2DBP.h"

using namespace std;

// 非根节点SP1子问题: 宽度方向背包问题 - CPLEX整数规划求解
// 功能: 为分支节点寻找能改进主问题目标值的新Y列
// 与根节点版本相同的数学模型:
//   max sum_{j} v_j * G_j  s.t. sum_{j} w_j * G_j <= W
// 注意: 背包模型不包含Arc约束, 因此只适用于无Arc约束的情况
// 返回值: true=列生成收敛, false=找到改进列
bool SolveNodeSP1Knapsack(ProblemParams& params, ProblemData& data, BPNode* node) {
    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);
    int num_strip_types = params.num_strip_types_;

    // 构建目标函数: max sum(v_j * G_j)
    // v_j: 条带平衡约束的对偶价格, 反映条带j的边际价值
    IloExpr obj_expr(env);
    for (int j = 0; j < num_strip_types; j++) {
        string var_name = "G_" + to_string(j + 1);
        IloNumVar var(env, 0, IloInfinity, ILOINT, var_name.c_str());
        vars.add(var);

        double dual = node->duals_[j];  // 指针访问
        if (dual != 0.0) {              // 跳过零系数项
            obj_expr += vars[j] * dual;
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    // 宽度约束: sum(w_j * G_j) <= W
    // 条带宽度总和不能超过母板宽度
    IloExpr wid_expr(env);
    for (int j = 0; j < num_strip_types; j++) {
        wid_expr += data.strip_types_[j].width_ * vars[j];
    }
    model.add(wid_expr <= params.stock_width_);
    wid_expr.end();

    // 求解背包问题
    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (背包)\n", node->iter_, node->id_);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());  // 关闭CPLEX输出
    bool feasible = cplex.solve();

    bool cg_converged = true;  // 默认假设收敛

    if (feasible) {
        double rc = cplex.getObjValue();  // reduced cost
        LOG_FMT("  [SP1] Reduced Cost: %.4f\n", rc);

        // 提取解向量, 构建新Y列的pattern
        node->new_y_col_.pattern_.clear();
        for (int j = 0; j < num_strip_types; j++) {
            double val = cplex.getValue(vars[j]);
            // 浮点取整: 处理0.99999这类数值误差
            int int_val = (val - (int)val > 0.99999) ? (int)val + 1 : (int)val;
            node->new_y_col_.pattern_.push_back(int_val);
        }

        // 判断是否找到改进列: rc > 1 表示该列能改进目标
        if (rc > 1 + kRcTolerance) {
            cg_converged = false;  // 找到改进列, 继续迭代
            LOG("  [SP1] 找到改进列");
        } else {
            cg_converged = true;   // 无改进列, 收敛
            node->new_y_col_.pattern_.clear();
            LOG("  [SP1] 收敛");
        }
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

// 非根节点SP1子问题: 宽度方向背包问题 - Arc Flow网络流模型求解
// 功能: 使用网络流模型求解背包问题, 支持Arc分支约束
// Arc Flow优势:
//   - 可在子问题中施加Arc约束, 保证分支切割的有效性
//   - 分支约束通过修改Arc变量的上下界实现
// 从父节点继承的Arc约束:
//   - sp1_zero_arcs_: 禁用Arc (setUB(0))
//   - sp1_lower_arcs_: Arc上界约束 (setUB(bound))
//   - sp1_greater_arcs_: Arc下界约束 (setLB(bound))
// 返回值: true=列生成收敛, false=找到改进列
bool SolveNodeSP1ArcFlow(ProblemParams& params, ProblemData& data, BPNode* node) {
    SP1ArcFlowData& arc_data = data.sp1_arc_data_;
    int num_arcs = static_cast<int>(arc_data.arc_list_.size());
    int num_strip_types = params.num_strip_types_;

    // 无Arc网络时视为收敛
    if (num_arcs == 0) {
        return true;
    }

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);

    // 为每个Arc创建0-1整数变量
    // 目标函数: max sum(v_j * a_i), v_j为Arc对应条带的对偶价格
    IloExpr obj_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        string var_name = "a_" + to_string(i + 1);
        IloNumVar var(env, 0, 1, ILOINT, var_name.c_str());
        vars.add(var);

        // Arc长度对应条带宽度, 映射到对偶价格
        int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        int strip_idx = data.width_to_strip_index_[arc_width];
        double dual = node->duals_[strip_idx];
        if (dual != 0.0) {
            obj_expr += vars[i] * dual;
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    // 宽度约束: 选中Arc总长度不超过母板宽度 (冗余约束, 可加速求解)
    IloExpr wid_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        wid_expr += arc_width * vars[i];
    }
    model.add(wid_expr <= params.stock_width_);
    wid_expr.end();

    // 起点约束: 从节点0出发的流量 = 1
    IloExpr begin_expr(env);
    for (int idx : arc_data.begin_arc_indices_) {
        begin_expr += vars[idx];
    }
    model.add(begin_expr == 1);
    begin_expr.end();

    // 终点约束: 进入终点节点的流量 = 1
    IloExpr end_expr(env);
    for (int idx : arc_data.end_arc_indices_) {
        end_expr += vars[idx];
    }
    model.add(end_expr == 1);
    end_expr.end();

    // 中间节点流量守恒: 流入量 = 流出量
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

    // 添加从父节点继承的SP1 Arc约束
    // 禁用Arc约束: 设置上界为0, 完全禁止使用该Arc
    for (const auto& arc : node->sp1_zero_arcs_) {
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setUB(0);  // 禁用该Arc
        }
    }

    // Arc上界约束: Arc流量 <= N (来自左分支)
    for (size_t i = 0; i < node->sp1_lower_arcs_.size(); i++) {
        const auto& arc = node->sp1_lower_arcs_[i];
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setUB(node->sp1_lower_bounds_[i]);
        }
    }

    // Arc下界约束: Arc流量 >= N (来自右分支)
    for (size_t i = 0; i < node->sp1_greater_arcs_.size(); i++) {
        const auto& arc = node->sp1_greater_arcs_[i];
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setLB(node->sp1_greater_bounds_[i]);
        }
    }

    // 求解Arc Flow子问题
    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (Arc Flow)\n", node->iter_, node->id_);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();

        // 判断是否找到改进列
        if (rc > 1 + kRcTolerance) {
            cg_converged = false;

            // 根据选中的Arc构建切割模式
            vector<int> pattern(num_strip_types, 0);
            for (int i = 0; i < num_arcs; i++) {
                double val = cplex.getValue(vars[i]);
                if (val > 0.5) {  // Arc被选中
                    int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
                    int strip_idx = data.width_to_strip_index_[arc_width];
                    pattern[strip_idx]++;
                }
            }
            node->new_y_col_.pattern_ = pattern;
        }
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

// 非根节点SP1子问题: 宽度方向背包问题 - 动态规划求解
// 功能: 使用完全背包DP求解SP1子问题
// 注意: DP方法不支持Arc约束, 仅适用于无约束或约束可忽略的情况
// 时间复杂度: O(num_strip_types * W)
// 返回值: true=列生成收敛, false=找到改进列
bool SolveNodeSP1DP(ProblemParams& params, ProblemData& data, BPNode* node) {
    int num_strip_types = params.num_strip_types_;
    int W = params.stock_width_;  // 母板宽度 = 背包容量

    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (DP)\n", node->iter_, node->id_);

    // dp[w]: 使用宽度w时能获得的最大价值
    // choice[w]: 达到最大价值时的切割方案
    vector<double> dp(W + 1, 0.0);
    vector<vector<int>> choice(W + 1, vector<int>(num_strip_types, 0));

    // 完全背包DP: 每种条带可使用任意次
    for (int j = 0; j < num_strip_types; j++) {
        int wid = data.strip_types_[j].width_;  // 条带宽度
        double val = node->duals_[j];           // 条带对偶价格
        if (val <= 0) continue;  // 跳过非正价值的条带

        // 从小到大遍历容量, 允许重复选择
        for (int w = wid; w <= W; w++) {
            if (dp[w - wid] + val > dp[w]) {
                dp[w] = dp[w - wid] + val;
                choice[w] = choice[w - wid];
                choice[w][j]++;  // 多选一个j型条带
            }
        }
    }

    double rc = dp[W];  // reduced cost = 最大价值
    if (rc > 1 + kRcTolerance) {
        node->new_y_col_.pattern_ = choice[W];  // 保存最优方案
        return false;  // 找到改进列
    }
    return true;  // 收敛
}

// 非根节点SP2子问题: 长度方向背包问题 - CPLEX整数规划求解
// 功能: 为指定条带类型寻找能改进主问题目标值的新X列
// 数学模型:
//   max sum_{i} pi_i * D_i  s.t. sum_{i} l_i * D_i <= L
//   其中 pi_i 是需求约束的对偶价格, l_i 是子板长度, L 是条带长度
// 列生成判断:
//   - reduced_cost = obj_value - v_j (X列在目标函数系数为-v_j)
//   - 若 reduced_cost > 0, 说明该列能改进目标
// 参数: strip_type_id - 条带类型索引
// 返回值: true=列生成收敛, false=找到改进列
bool SolveNodeSP2Knapsack(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id) {

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);
    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;
    int strip_width = data.strip_types_[strip_type_id].width_;  // 当前条带宽度

    // 构建目标函数: max sum(pi_i * D_i)
    IloExpr obj_expr(env);
    double sum_coef = 0;  // 用于检查是否有正系数

    for (int i = 0; i < num_item_types; i++) {
        string var_name = "D_" + to_string(i + 1);
        IloNumVar var(env, 0, IloInfinity, ILOINT, var_name.c_str());
        vars.add(var);

        // 只考虑宽度能装入当前条带的子板
        if (data.item_types_[i].width_ <= strip_width) {
            double dual = node->duals_[num_strip_types + i];  // pi_i
            if (dual > 0) {
                obj_expr += vars[i] * dual;
                sum_coef += dual;
            }
        }
    }

    // 无正系数子板, 无需求解 (不可能找到改进列)
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
    // 子板长度总和不能超过条带长度(=母板长度)
    IloExpr len_expr(env);
    for (int i = 0; i < num_item_types; i++) {
        if (data.item_types_[i].width_ <= strip_width) {
            len_expr += data.item_types_[i].length_ * vars[i];
        }
    }
    model.add(len_expr <= params.stock_length_);
    len_expr.end();

    // 求解背包问题
    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (背包)\n", node->iter_, strip_type_id);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        double dual_v = node->duals_[strip_type_id];  // 条带的对偶价格

        // 判断改进条件: rc > v_j
        if (rc > dual_v + kRcTolerance) {
            cg_converged = false;

            // 提取解向量
            node->new_x_col_.pattern_.clear();
            for (int i = 0; i < num_item_types; i++) {
                double val = cplex.getValue(vars[i]);
                int int_val = (val - (int)val > 0.99999) ? (int)val + 1 : (int)val;
                node->new_x_col_.pattern_.push_back(int_val);
            }
            node->new_strip_type_ = strip_type_id;  // 记录条带类型
        }
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

// 非根节点SP2子问题: 长度方向背包问题 - Arc Flow网络流模型求解
// 功能: 使用网络流模型求解SP2子问题, 支持Arc分支约束
// Arc Flow网络结构:
//   - 节点: 0, 1, 2, ..., L (表示条带长度的使用位置)
//   - Arc (i, j): 在位置i放置长度为(j-i)的子板
//   - 每种条带类型有独立的Arc网络 (因宽度不同, 可装入的子板不同)
// 从父节点继承的Arc约束 (按条带类型索引):
//   - sp2_zero_arcs_[strip_type_id]: 禁用的Arc
//   - sp2_lower_arcs_[strip_type_id]: Arc上界约束
//   - sp2_greater_arcs_[strip_type_id]: Arc下界约束
// 返回值: true=列生成收敛, false=找到改进列
bool SolveNodeSP2ArcFlow(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id) {

    // 确保该条带类型的Arc网络已生成
    if ((int)data.sp2_arc_data_.size() <= strip_type_id) {
        GenerateSP2Arcs(data, params, strip_type_id);
    }

    SP2ArcFlowData& arc_data = data.sp2_arc_data_[strip_type_id];
    int num_arcs = static_cast<int>(arc_data.arc_list_.size());
    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;

    // 无Arc网络时视为收敛
    if (num_arcs == 0) return true;

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);

    // 为每个Arc创建0-1整数变量
    // 目标函数: max sum(pi_i * a_k), pi_i为Arc对应子板的对偶价格
    IloExpr obj_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        string var_name = "a_" + to_string(i + 1);
        IloNumVar var(env, 0, 1, ILOINT, var_name.c_str());
        vars.add(var);

        // Arc长度对应子板长度, 映射到对偶价格
        int arc_len = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        if (data.length_to_item_index_.count(arc_len)) {
            int item_idx = data.length_to_item_index_[arc_len];
            double dual = node->duals_[num_strip_types + item_idx];
            if (dual > 0) {
                obj_expr += vars[i] * dual;
            }
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    // 长度约束: 选中Arc总长度不超过条带长度 (冗余约束)
    IloExpr len_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        int arc_len = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        len_expr += arc_len * vars[i];
    }
    model.add(len_expr <= params.stock_length_);
    len_expr.end();

    // 起点约束: 从节点0出发的流量 = 1
    IloExpr begin_expr(env);
    for (int idx : arc_data.begin_arc_indices_) {
        begin_expr += vars[idx];
    }
    model.add(begin_expr == 1);
    begin_expr.end();

    // 终点约束: 进入终点节点的流量 = 1
    IloExpr end_expr(env);
    for (int idx : arc_data.end_arc_indices_) {
        end_expr += vars[idx];
    }
    model.add(end_expr == 1);
    end_expr.end();

    // 中间节点流量守恒: 流入量 = 流出量
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

    // 添加从父节点继承的SP2 Arc约束 (针对当前条带类型)
    // 禁用Arc约束: 设置上界为0
    if (node->sp2_zero_arcs_.count(strip_type_id)) {
        for (const auto& arc : node->sp2_zero_arcs_.at(strip_type_id)) {
            if (arc_data.arc_to_index_.count(arc)) {
                int idx = arc_data.arc_to_index_.at(arc);
                vars[idx].setUB(0);
            }
        }
    }

    // Arc上界约束: Arc流量 <= N (来自左分支)
    if (node->sp2_lower_arcs_.count(strip_type_id)) {
        const auto& arcs = node->sp2_lower_arcs_.at(strip_type_id);
        const auto& bounds = node->sp2_lower_bounds_.at(strip_type_id);
        for (size_t i = 0; i < arcs.size(); i++) {
            if (arc_data.arc_to_index_.count(arcs[i])) {
                int idx = arc_data.arc_to_index_.at(arcs[i]);
                vars[idx].setUB(bounds[i]);
            }
        }
    }

    // Arc下界约束: Arc流量 >= N (来自右分支)
    if (node->sp2_greater_arcs_.count(strip_type_id)) {
        const auto& arcs = node->sp2_greater_arcs_.at(strip_type_id);
        const auto& bounds = node->sp2_greater_bounds_.at(strip_type_id);
        for (size_t i = 0; i < arcs.size(); i++) {
            if (arc_data.arc_to_index_.count(arcs[i])) {
                int idx = arc_data.arc_to_index_.at(arcs[i]);
                vars[idx].setLB(bounds[i]);
            }
        }
    }

    // 求解Arc Flow子问题
    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (Arc Flow)\n", node->iter_, strip_type_id);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        double dual_v = node->duals_[strip_type_id];  // 条带的对偶价格

        // 判断改进条件: rc > v_j
        if (rc > dual_v + kRcTolerance) {
            cg_converged = false;

            // 根据选中的Arc构建切割模式
            vector<int> pattern(num_item_types, 0);
            for (int i = 0; i < num_arcs; i++) {
                double val = cplex.getValue(vars[i]);
                if (val > 0.5) {  // Arc被选中
                    int arc_len = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
                    if (data.length_to_item_index_.count(arc_len)) {
                        int item_idx = data.length_to_item_index_[arc_len];
                        pattern[item_idx]++;
                    }
                }
            }
            node->new_x_col_.pattern_ = pattern;
            node->new_strip_type_ = strip_type_id;
        }
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

// 非根节点SP2子问题: 长度方向背包问题 - 动态规划求解
// 功能: 使用完全背包DP求解SP2子问题
// 注意: DP方法不支持Arc约束, 仅适用于无约束或约束可忽略的情况
// 时间复杂度: O(num_item_types * L)
// 参数: strip_type_id - 条带类型索引
// 返回值: true=列生成收敛, false=找到改进列
bool SolveNodeSP2DP(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id) {

    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;
    int L = params.stock_length_;  // 条带长度 = 背包容量
    int strip_width = data.strip_types_[strip_type_id].width_;

    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (DP)\n", node->iter_, strip_type_id);

    // dp[l]: 使用长度l时能获得的最大价值
    // choice[l]: 达到最大价值时的切割方案
    vector<double> dp(L + 1, 0.0);
    vector<vector<int>> choice(L + 1, vector<int>(num_item_types, 0));

    // 完全背包DP: 每种子板可使用任意次
    for (int i = 0; i < num_item_types; i++) {
        // 只考虑宽度能装入当前条带的子板
        if (data.item_types_[i].width_ > strip_width) continue;

        int len = data.item_types_[i].length_;   // 子板长度
        double val = node->duals_[num_strip_types + i];  // 子板对偶价格
        if (val <= 0) continue;  // 跳过非正价值的子板

        // 从小到大遍历容量, 允许重复选择
        for (int l = len; l <= L; l++) {
            if (dp[l - len] + val > dp[l]) {
                dp[l] = dp[l - len] + val;
                choice[l] = choice[l - len];
                choice[l][i]++;  // 多选一个i型子板
            }
        }
    }

    double rc = dp[L];  // 目标值
    double dual_v = node->duals_[strip_type_id];  // 条带对偶价格

    // 判断改进条件: rc > v_j
    if (rc > dual_v + kRcTolerance) {
        node->new_x_col_.pattern_ = choice[L];  // 保存最优方案
        node->new_strip_type_ = strip_type_id;
        return false;  // 找到改进列
    }
    return true;  // 收敛
}
