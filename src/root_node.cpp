// =============================================================================
// root_node.cpp - 根节点列生成和主问题
// =============================================================================

#include "2DBP.h"

using namespace std;

// 根节点列生成主循环
void SolveRootCG(ProblemParams& params, ProblemData& data, BPNode& root_node) {
    LOG("[CG] 根节点列生成开始");

    // 继承全局SP方法设置
    root_node.sp1_method_ = params.sp1_method_;
    root_node.sp2_method_ = params.sp2_method_;

    // 初始化CPLEX环境
    IloEnv env;
    IloModel model(env);
    IloObjective obj = IloAdd(model, IloMinimize(env));
    IloNumVarArray vars(env);
    IloRangeArray cons(env);

    root_node.iter_ = 0;

    // 求解初始主问题
    bool feasible = SolveRootInitMP(params, data, env, model, obj, cons, vars, root_node);

    if (feasible) {
        // 列生成主循环
        while (true) {
            root_node.iter_++;

            if (root_node.iter_ >= kMaxCgIter) {
                LOG_FMT("[CG] 达到最大迭代次数 %d, 终止\n", kMaxCgIter);
                break;
            }

            // 求解子问题SP1 (宽度背包)
            bool sp1_converged = SolveRootSP1(params, data, root_node);

            if (sp1_converged) {
                // SP1收敛, 尝试SP2
                bool all_sp2_converged = true;

                for (int j = 0; j < params.num_strip_types_; j++) {
                    bool sp2_converged = SolveRootSP2(params, data, root_node, j);

                    if (!sp2_converged) {
                        all_sp2_converged = false;
                        // 添加新X列
                        SolveRootUpdateMP(params, data, env, model, obj, cons, vars, root_node);
                    }
                }

                if (all_sp2_converged) {
                    LOG_FMT("[CG] 列生成收敛, 迭代%d次\n", root_node.iter_);
                    break;
                }
            } else {
                // SP1找到改进列, 添加新Y列
                SolveRootUpdateMP(params, data, env, model, obj, cons, vars, root_node);
            }
        }

        // 求解最终主问题
        SolveRootFinalMP(params, data, env, model, obj, cons, vars, root_node);
    }

    // 释放CPLEX资源
    obj.end();
    vars.end();
    cons.end();
    model.end();
    env.end();

    LOG("[CG] 根节点列生成结束");
}

// 求解根节点初始主问题
// 主问题结构:
//   min sum(y_k)                         (最小化母板使用量)
//   s.t. sum(C_jk*y_k) - sum(x_p) >= 0   (条带平衡约束)
//        sum(B_ip*x_p) >= d_i            (子件需求约束)
bool SolveRootInitMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& root_node) {

    int num_y_cols = static_cast<int>(root_node.y_columns_.size());
    int num_x_cols = static_cast<int>(root_node.x_columns_.size());
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_rows = num_strip_types + num_item_types;

    LOG_FMT("[MP-0] 构建初始主问题 (Y=%d, X=%d)\n", num_y_cols, num_x_cols);

    // 构建约束
    IloNumArray con_min(env);
    IloNumArray con_max(env);

    // 条带平衡约束: sum(C_jk*y_k) - sum(x_p) >= 0
    for (int j = 0; j < num_strip_types; j++) {
        con_min.add(0);
        con_max.add(IloInfinity);
    }

    // 子件需求约束: sum(B_ip*x_p) >= d_i
    for (int i = 0; i < num_item_types; i++) {
        con_min.add(data.item_types_[i].demand_);
        con_max.add(IloInfinity);
    }

    cons = IloRangeArray(env, con_min, con_max);
    model.add(cons);
    con_min.end();
    con_max.end();

    // 添加Y变量 (目标系数=1)
    for (int col = 0; col < num_y_cols; col++) {
        IloNumColumn cplex_col = obj(1.0);

        // 条带产出部分
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j](root_node.y_columns_[col].pattern_[j]);
        }
        // 子件产出部分 (0)
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](0);
        }

        string var_name = "Y_" + to_string(col + 1);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();
    }

    // 添加X变量 (目标系数=0)
    for (int col = 0; col < num_x_cols; col++) {
        IloNumColumn cplex_col = obj(0.0);
        int strip_type = root_node.x_columns_[col].strip_type_id_;

        // 条带消耗部分 (-1在对应位置)
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j]((j == strip_type) ? -1 : 0);
        }
        // 子件产出部分
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](root_node.x_columns_[col].pattern_[i]);
        }

        string var_name = "X_" + to_string(col + 1);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();
    }

    // 求解
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());

    // 导出LP文件 (调试用)
    if (kExportLp) {
        string lp_file = kLpDir + "root_init_mp.lp";
        cplex.exportModel(lp_file.c_str());
    }

    bool feasible = cplex.solve();

    if (!feasible) {
        LOG("[MP] 初始主问题不可行");
        cplex.end();
        return false;
    }

    double obj_val = cplex.getValue(obj);
    LOG_FMT("[MP] 目标值: %.4f\n", obj_val);

    // 提取对偶价格
    root_node.duals_.clear();
    for (int row = 0; row < num_rows; row++) {
        double dual = cplex.getDual(cons[row]);
        if (dual == -0.0) dual = 0.0;
        root_node.duals_.push_back(dual);
    }

    cplex.end();
    return true;
}

// 更新主问题 (添加新列)
bool SolveRootUpdateMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& node) {

    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;

    // 添加新Y列
    if (!node.new_y_col_.pattern_.empty()) {
        IloNumColumn cplex_col = obj(1.0);

        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j](node.new_y_col_.pattern_[j]);
        }
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](0);
        }

        int col_id = static_cast<int>(node.y_columns_.size()) + 1;
        string var_name = "Y_" + to_string(col_id);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();

        // 保存新列
        YColumn y_col;
        y_col.pattern_ = node.new_y_col_.pattern_;
        y_col.arc_set_ = node.new_y_col_.arc_set_;
        node.y_columns_.push_back(y_col);

        node.new_y_col_.pattern_.clear();
        node.new_y_col_.arc_set_.clear();
    }

    // 添加新X列
    if (!node.new_x_col_.pattern_.empty()) {
        int strip_type = node.new_strip_type_;
        IloNumColumn cplex_col = obj(0.0);

        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j]((j == strip_type) ? -1 : 0);
        }
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](node.new_x_col_.pattern_[i]);
        }

        int col_id = static_cast<int>(node.x_columns_.size()) + 1;
        string var_name = "X_" + to_string(col_id);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();

        // 保存新列
        XColumn x_col;
        x_col.strip_type_id_ = strip_type;
        x_col.pattern_ = node.new_x_col_.pattern_;
        x_col.arc_set_ = node.new_x_col_.arc_set_;
        node.x_columns_.push_back(x_col);

        node.new_x_col_.pattern_.clear();
        node.new_x_col_.arc_set_.clear();
    }

    // 求解更新后的主问题
    LOG_FMT("[MP-%d] 更新并求解主问题\n", node.iter_);
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    if (!feasible) {
        LOG("[MP] 更新后主问题不可行");
        cplex.end();
        return false;
    }

    double obj_val = cplex.getValue(obj);
    LOG_FMT("[MP] 目标值: %.4f\n", obj_val);

    // 提取对偶价格
    node.duals_.clear();
    int num_rows = num_strip_types + num_item_types;
    for (int row = 0; row < num_rows; row++) {
        double dual = cplex.getDual(cons[row]);
        if (dual == -0.0) dual = 0.0;
        node.duals_.push_back(dual);
    }

    cplex.end();
    return true;
}

// 求解最终主问题
bool SolveRootFinalMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& node) {

    LOG_FMT("[MP-Final] 节点%d求解最终主问题\n", node.id_);

    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());

    // 导出LP文件 (调试用)
    if (kExportLp) {
        string lp_file = kLpDir + "root_final_mp.lp";
        cplex.exportModel(lp_file.c_str());
    }

    bool feasible = cplex.solve();

    if (!feasible) {
        LOG("[MP] 最终主问题不可行");
        cplex.end();
        return false;
    }

    double obj_val = cplex.getValue(obj);
    node.lower_bound_ = obj_val;
    node.solution_.obj_val_ = obj_val;

    LOG_FMT("[MP] 最终目标值: %.4f\n", obj_val);

    // 提取Y列解值
    node.solution_.y_columns_.clear();
    for (int col = 0; col < (int)node.y_columns_.size(); col++) {
        double val = cplex.getValue(vars[col]);
        if (fabs(val) < kZeroTolerance) val = 0;

        YColumn y_col = node.y_columns_[col];
        y_col.value_ = val;
        node.solution_.y_columns_.push_back(y_col);

        if (val > kZeroTolerance) {
            LOG_FMT("  Y_%d = %.4f\n", col + 1, val);
        }
    }

    // 提取X列解值
    node.solution_.x_columns_.clear();
    int y_count = static_cast<int>(node.y_columns_.size());
    for (int col = 0; col < (int)node.x_columns_.size(); col++) {
        double val = cplex.getValue(vars[y_count + col]);
        if (fabs(val) < kZeroTolerance) val = 0;

        XColumn x_col = node.x_columns_[col];
        x_col.value_ = val;
        node.solution_.x_columns_.push_back(x_col);

        if (val > kZeroTolerance) {
            LOG_FMT("  X_%d = %.4f\n", col + 1, val);
        }
    }

    cplex.end();
    return true;
}
