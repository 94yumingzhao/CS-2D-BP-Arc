// =============================================================================
// update_master_problem.cpp - 主问题更新模块
// =============================================================================
//
// 功能: 在列生成过程中动态更新主问题
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// UpdateMP - 添加新列并重新求解主问题
// =============================================================================
void UpdateMP(
    ProblemParams& params,
    ProblemData& data,
    IloEnv& mp_env,
    IloModel& mp_model,
    IloObjective& mp_obj,
    IloRangeArray& mp_cons,
    IloNumVarArray& mp_vars,
    BPNode& node) {

    // =========================================================================
    // 问题规模
    // =========================================================================
    int num_y_cols = node.y_cols_.size();
    int num_x_cols = node.x_cols_.size();
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_rows = num_strip_types + num_item_types;

    // =========================================================================
    // 情况 1: 添加新的 Y 列 (母板切割模式)
    // =========================================================================
    if (node.col_type_flag_ == 1) {
        IloNum obj_coef = 1;
        IloNumColumn cplex_col = mp_obj(obj_coef);

        for (int row = 0; row < num_rows; row++) {
            IloNum row_coef = node.new_y_col_[row];
            cplex_col += mp_cons[row](row_coef);
        }

        int cols_num = node.y_cols_.size();
        string var_name = "Y_" + to_string(cols_num + 1);
        IloNum var_lb = 0;
        IloNum var_ub = IloInfinity;
        IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
        mp_vars.add(var_y);
        cplex_col.end();

        vector<double> new_col;
        for (int row = 0; row < num_rows; row++) {
            new_col.push_back(node.new_y_col_[row]);
        }

        node.y_cols_.push_back(new_col);
        node.matrix_.insert(node.matrix_.begin() + node.y_cols_.size(), new_col);
        node.new_y_col_.clear();
    }

    // =========================================================================
    // 情况 2: 添加新的 X 列 (条带切割模式)
    // =========================================================================
    if (node.col_type_flag_ == 0) {
        int num_new_cols = node.new_x_cols_.size();

        for (int col = 0; col < num_new_cols; col++) {
            IloNum obj_coef = 0;
            IloNumColumn cplex_col = mp_obj(obj_coef);

            for (int row = 0; row < num_rows; row++) {
                IloNum row_coef = node.new_x_cols_[col][row];
                cplex_col += mp_cons[row](row_coef);
            }

            int old_cols_num = node.x_cols_.size();
            string var_name = "X_" + to_string(old_cols_num + 1);
            IloNum var_lb = 0;
            IloNum var_ub = IloInfinity;
            IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
            mp_vars.add(var_x);
            cplex_col.end();

            vector<double> temp_col;
            for (int row = 0; row < num_rows; row++) {
                temp_col.push_back(node.new_x_cols_[col][row]);
            }

            node.x_cols_.push_back(temp_col);
            node.matrix_.insert(node.matrix_.end(), temp_col);
        }

        node.new_x_cols_.clear();
    }

    // =========================================================================
    // 求解更新后的主问题
    // =========================================================================
    IloCplex mp_cplex(mp_model);
    mp_cplex.extract(mp_model);
    mp_cplex.setOut(mp_env.getNullStream());
    mp_cplex.solve();

    cout << "[主问题] 迭代 " << node.iter_ << ", 目标值 = "
         << fixed << setprecision(4) << mp_cplex.getValue(mp_obj) << "\n";
    cout.unsetf(ios::fixed);

    // =========================================================================
    // 提取对偶价格
    // =========================================================================
    node.duals_.clear();

    for (int row = 0; row < num_strip_types; row++) {
        double dual_val = mp_cplex.getDual(mp_cons[row]);
        if (std::abs(dual_val) < kZeroTolerance) dual_val = 0;
        node.duals_.push_back(dual_val);
    }

    for (int row = num_strip_types; row < num_strip_types + num_item_types; row++) {
        double dual_val = mp_cplex.getDual(mp_cons[row]);
        if (std::abs(dual_val) < kZeroTolerance) dual_val = 0;
        node.duals_.push_back(dual_val);
    }

    mp_cplex.end();
}


// =============================================================================
// SolveFinalMP - 求解列生成收敛后的最终主问题
// =============================================================================
void SolveFinalMP(
    ProblemParams& params,
    ProblemData& data,
    IloEnv& mp_env,
    IloModel& mp_model,
    IloObjective& mp_obj,
    IloRangeArray& mp_cons,
    IloNumVarArray& mp_vars,
    BPNode& node) {

    // =========================================================================
    // 问题规模
    // =========================================================================
    int num_y_cols = node.y_cols_.size();
    int num_x_cols = node.x_cols_.size();
    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;
    int num_rows = num_item_types + num_strip_types;
    int num_cols = num_y_cols + num_x_cols;

    // =========================================================================
    // 求解最终主问题
    // =========================================================================
    IloCplex mp_cplex(mp_model);
    mp_cplex.extract(mp_model);
    mp_cplex.setOut(mp_env.getNullStream());
    mp_cplex.exportModel("Final Master Problem.lp");
    mp_cplex.solve();

    node.lower_bound_ = mp_cplex.getValue(mp_obj);

    cout << "[主问题] 最终目标值 = " << fixed << setprecision(4) << node.lower_bound_ << "\n";
    cout.unsetf(ios::fixed);

    // =========================================================================
    // 保存所有变量的解值
    // =========================================================================
    for (int col = 0; col < num_cols; col++) {
        IloNum sol_val = mp_cplex.getValue(mp_vars[col]);
        if (std::abs(sol_val) < kZeroTolerance) sol_val = 0;
        node.solution_.push_back(sol_val);
    }

    // =========================================================================
    // 统计非零解
    // =========================================================================
    int num_y_nonzero = 0;
    int num_x_nonzero = 0;

    for (int col = 0; col < num_y_cols; col++) {
        double sol_val = mp_cplex.getValue(mp_vars[col]);
        if (sol_val > 0) num_y_nonzero++;
    }

    for (int col = num_y_cols; col < num_y_cols + num_x_cols; col++) {
        double sol_val = mp_cplex.getValue(mp_vars[col]);
        if (sol_val > 0) num_x_nonzero++;
    }

    cout << "[主问题] 非零解: Y=" << num_y_nonzero << ", X=" << num_x_nonzero
         << " (总变量: " << num_cols << ")\n";

    mp_cplex.end();
}
