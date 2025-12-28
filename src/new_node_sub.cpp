// =============================================================================
// new_node_sub.cpp - 非根节点子问题函数
// =============================================================================

#include "2DBP.h"

using namespace std;

// 非根节点SP1: CPLEX整数规划求解
bool SolveNodeSP1Knapsack(ProblemParams& params, ProblemData& data, BPNode* node) {
    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);
    int num_strip_types = params.num_strip_types_;

    IloExpr obj_expr(env);
    for (int j = 0; j < num_strip_types; j++) {
        string var_name = "G_" + to_string(j + 1);
        IloNumVar var(env, 0, IloInfinity, ILOINT, var_name.c_str());
        vars.add(var);

        double dual = node->duals_[j];
        if (dual != 0.0) {
            obj_expr += vars[j] * dual;
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    IloExpr wid_expr(env);
    for (int j = 0; j < num_strip_types; j++) {
        wid_expr += data.strip_types_[j].width_ * vars[j];
    }
    model.add(wid_expr <= params.stock_width_);
    wid_expr.end();

    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (背包)\n", node->iter_, node->id_);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        LOG_FMT("  [SP1] Reduced Cost: %.4f\n", rc);

        node->new_y_col_.pattern_.clear();
        for (int j = 0; j < num_strip_types; j++) {
            double val = cplex.getValue(vars[j]);
            int int_val = (val - (int)val > 0.99999) ? (int)val + 1 : (int)val;
            node->new_y_col_.pattern_.push_back(int_val);
        }

        if (rc > 1 + kRcTolerance) {
            cg_converged = false;
            LOG("  [SP1] 找到改进列");
        } else {
            cg_converged = true;
            node->new_y_col_.pattern_.clear();
            LOG("  [SP1] 收敛");
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

// 非根节点SP1: Arc Flow求解
bool SolveNodeSP1ArcFlow(ProblemParams& params, ProblemData& data, BPNode* node) {
    // 与根节点类似, 调用Arc Flow模型
    SP1ArcFlowData& arc_data = data.sp1_arc_data_;
    int num_arcs = static_cast<int>(arc_data.arc_list_.size());
    int num_strip_types = params.num_strip_types_;

    if (num_arcs == 0) {
        return true;
    }

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);

    IloExpr obj_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        string var_name = "a_" + to_string(i + 1);
        IloNumVar var(env, 0, 1, ILOINT, var_name.c_str());
        vars.add(var);

        int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        int strip_idx = data.width_to_strip_index_[arc_width];
        double dual = node->duals_[strip_idx];
        if (dual != 0.0) {
            obj_expr += vars[i] * dual;
        }
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    IloExpr wid_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        wid_expr += arc_width * vars[i];
    }
    model.add(wid_expr <= params.stock_width_);
    wid_expr.end();

    IloExpr begin_expr(env);
    for (int idx : arc_data.begin_arc_indices_) {
        begin_expr += vars[idx];
    }
    model.add(begin_expr == 1);
    begin_expr.end();

    IloExpr end_expr(env);
    for (int idx : arc_data.end_arc_indices_) {
        end_expr += vars[idx];
    }
    model.add(end_expr == 1);
    end_expr.end();

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

    // 添加 SP1 Arc 约束 (从父节点继承)
    // 禁用 Arc
    for (const auto& arc : node->sp1_zero_arcs_) {
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setUB(0);
        }
    }
    // Arc <= N 约束
    for (size_t i = 0; i < node->sp1_lower_arcs_.size(); i++) {
        const auto& arc = node->sp1_lower_arcs_[i];
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setUB(node->sp1_lower_bounds_[i]);
        }
    }
    // Arc >= N 约束
    for (size_t i = 0; i < node->sp1_greater_arcs_.size(); i++) {
        const auto& arc = node->sp1_greater_arcs_[i];
        if (arc_data.arc_to_index_.count(arc)) {
            int idx = arc_data.arc_to_index_.at(arc);
            vars[idx].setLB(node->sp1_greater_bounds_[i]);
        }
    }

    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (Arc Flow)\n", node->iter_, node->id_);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();

        if (rc > 1 + kRcTolerance) {
            cg_converged = false;
            vector<int> pattern(num_strip_types, 0);
            for (int i = 0; i < num_arcs; i++) {
                double val = cplex.getValue(vars[i]);
                if (val > 0.5) {
                    int arc_width = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
                    int strip_idx = data.width_to_strip_index_[arc_width];
                    pattern[strip_idx]++;
                }
            }
            node->new_y_col_.pattern_ = pattern;
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

// 非根节点SP1: DP求解
bool SolveNodeSP1DP(ProblemParams& params, ProblemData& data, BPNode* node) {
    int num_strip_types = params.num_strip_types_;
    int W = params.stock_width_;

    LOG_FMT("[SP1-%d] 节点%d 求解SP1 (DP)\n", node->iter_, node->id_);

    vector<double> dp(W + 1, 0.0);
    vector<vector<int>> choice(W + 1, vector<int>(num_strip_types, 0));

    for (int j = 0; j < num_strip_types; j++) {
        int wid = data.strip_types_[j].width_;
        double val = node->duals_[j];
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
    if (rc > 1 + kRcTolerance) {
        node->new_y_col_.pattern_ = choice[W];
        return false;
    }
    return true;
}

// 非根节点SP2: CPLEX整数规划求解
bool SolveNodeSP2Knapsack(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id) {

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);
    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;
    int strip_width = data.strip_types_[strip_type_id].width_;

    IloExpr obj_expr(env);
    double sum_coef = 0;

    for (int i = 0; i < num_item_types; i++) {
        string var_name = "D_" + to_string(i + 1);
        IloNumVar var(env, 0, IloInfinity, ILOINT, var_name.c_str());
        vars.add(var);

        if (data.item_types_[i].width_ <= strip_width) {
            double dual = node->duals_[num_strip_types + i];
            if (dual > 0) {
                obj_expr += vars[i] * dual;
                sum_coef += dual;
            }
        }
    }

    if (sum_coef <= 0) {
        obj_expr.end();
        vars.end();
        model.end();
        env.end();
        return true;
    }

    IloObjective obj = IloMaximize(env, obj_expr);
    model.add(obj);

    IloExpr len_expr(env);
    for (int i = 0; i < num_item_types; i++) {
        if (data.item_types_[i].width_ <= strip_width) {
            len_expr += data.item_types_[i].length_ * vars[i];
        }
    }
    model.add(len_expr <= params.stock_length_);
    len_expr.end();

    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (背包)\n", node->iter_, strip_type_id);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        double dual_v = node->duals_[strip_type_id];

        if (rc > dual_v + kRcTolerance) {
            cg_converged = false;
            node->new_x_col_.pattern_.clear();
            for (int i = 0; i < num_item_types; i++) {
                double val = cplex.getValue(vars[i]);
                int int_val = (val - (int)val > 0.99999) ? (int)val + 1 : (int)val;
                node->new_x_col_.pattern_.push_back(int_val);
            }
            node->new_strip_type_ = strip_type_id;
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

// 非根节点SP2: Arc Flow求解
bool SolveNodeSP2ArcFlow(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id) {

    if ((int)data.sp2_arc_data_.size() <= strip_type_id) {
        GenerateSP2Arcs(data, params, strip_type_id);
    }

    SP2ArcFlowData& arc_data = data.sp2_arc_data_[strip_type_id];
    int num_arcs = static_cast<int>(arc_data.arc_list_.size());
    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;

    if (num_arcs == 0) return true;

    IloEnv env;
    IloModel model(env);
    IloNumVarArray vars(env);

    IloExpr obj_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        string var_name = "a_" + to_string(i + 1);
        IloNumVar var(env, 0, 1, ILOINT, var_name.c_str());
        vars.add(var);

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

    IloExpr len_expr(env);
    for (int i = 0; i < num_arcs; i++) {
        int arc_len = arc_data.arc_list_[i][1] - arc_data.arc_list_[i][0];
        len_expr += arc_len * vars[i];
    }
    model.add(len_expr <= params.stock_length_);
    len_expr.end();

    IloExpr begin_expr(env);
    for (int idx : arc_data.begin_arc_indices_) {
        begin_expr += vars[idx];
    }
    model.add(begin_expr == 1);
    begin_expr.end();

    IloExpr end_expr(env);
    for (int idx : arc_data.end_arc_indices_) {
        end_expr += vars[idx];
    }
    model.add(end_expr == 1);
    end_expr.end();

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

    // 添加 SP2 Arc 约束 (针对该条带类型, 从父节点继承)
    // 禁用 Arc
    if (node->sp2_zero_arcs_.count(strip_type_id)) {
        for (const auto& arc : node->sp2_zero_arcs_.at(strip_type_id)) {
            if (arc_data.arc_to_index_.count(arc)) {
                int idx = arc_data.arc_to_index_.at(arc);
                vars[idx].setUB(0);
            }
        }
    }
    // Arc <= N 约束
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
    // Arc >= N 约束
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

    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (Arc Flow)\n", node->iter_, strip_type_id);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    bool cg_converged = true;

    if (feasible) {
        double rc = cplex.getObjValue();
        double dual_v = node->duals_[strip_type_id];

        if (rc > dual_v + kRcTolerance) {
            cg_converged = false;
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
            node->new_x_col_.pattern_ = pattern;
            node->new_strip_type_ = strip_type_id;
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

// 非根节点SP2: DP求解
bool SolveNodeSP2DP(ProblemParams& params, ProblemData& data,
    BPNode* node, int strip_type_id) {

    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;
    int L = params.stock_length_;
    int strip_width = data.strip_types_[strip_type_id].width_;

    LOG_FMT("[SP2-%d] 条带类型%d 求解SP2 (DP)\n", node->iter_, strip_type_id);

    vector<double> dp(L + 1, 0.0);
    vector<vector<int>> choice(L + 1, vector<int>(num_item_types, 0));

    for (int i = 0; i < num_item_types; i++) {
        if (data.item_types_[i].width_ > strip_width) continue;

        int len = data.item_types_[i].length_;
        double val = node->duals_[num_strip_types + i];
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
    double dual_v = node->duals_[strip_type_id];

    if (rc > dual_v + kRcTolerance) {
        node->new_x_col_.pattern_ = choice[L];
        node->new_strip_type_ = strip_type_id;
        return false;
    }
    return true;
}
