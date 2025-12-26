// =============================================================================
// root_node_first_master_problem.cpp - 根节点初始主问题
// =============================================================================
//
// 功能: 构建并求解分支定价树根节点的初始受限主问题 (RMP)
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// SolveRootInitMP - 求解根节点初始主问题
// =============================================================================
bool SolveRootInitMP(
    ProblemParams& params,
    ProblemData& data,
    IloEnv& mp_env,
    IloModel& mp_model,
    IloObjective& mp_obj,
    IloRangeArray& mp_cons,
    IloNumVarArray& mp_vars,
    BPNode& root_node) {

    // =========================================================================
    // 问题规模
    // =========================================================================
    int num_y_cols = root_node.y_cols_.size();
    int num_x_cols = root_node.x_cols_.size();
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_cols = num_y_cols + num_x_cols;
    int num_rows = num_strip_types + num_item_types;

    cout << "[主问题] 构建初始主问题 (Y=" << num_y_cols << ", X=" << num_x_cols << ")\n";

    // =========================================================================
    // 第一步: 构建约束
    // =========================================================================
    IloNumArray con_min(mp_env);
    IloNumArray con_max(mp_env);

    for (int row = 0; row < num_strip_types + num_item_types; row++) {
        if (row < num_strip_types) {
            // ----- 条带平衡约束 -----
            con_min.add(IloNum(0));
            con_max.add(IloNum(IloInfinity));
        }
        if (row >= num_strip_types) {
            // ----- 子件需求约束 -----
            int row_idx = row - num_strip_types;
            double demand_val = data.item_types_[row_idx].demand_;
            con_min.add(IloNum(demand_val));
            con_max.add(IloNum(IloInfinity));
        }
    }

    mp_cons = IloRangeArray(mp_env, con_min, con_max);
    mp_model.add(mp_cons);
    con_min.end();
    con_max.end();

    // =========================================================================
    // 第二步: 构建 Y 变量 (母板切割模式)
    // =========================================================================
    for (int col = 0; col < num_y_cols; col++) {
        IloNum obj_coef = 1;
        IloNumColumn cplex_col = mp_obj(obj_coef);

        for (int row = 0; row < num_strip_types + num_item_types; row++) {
            IloNum row_coef = root_node.matrix_[col][row];
            cplex_col += mp_cons[row](row_coef);
        }

        string var_name = "Y_" + to_string(col + 1);
        IloNum var_lb = 0;
        IloNum var_ub = IloInfinity;
        IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
        mp_vars.add(var_y);
        cplex_col.end();
    }

    // =========================================================================
    // 第三步: 构建 X 变量 (条带切割模式)
    // =========================================================================
    for (int col = num_y_cols; col < num_y_cols + num_x_cols; col++) {
        IloNum obj_coef = 0;
        IloNumColumn cplex_col = mp_obj(obj_coef);

        for (int row = 0; row < num_strip_types + num_item_types; row++) {
            IloNum row_coef = root_node.matrix_[col][row];
            cplex_col += mp_cons[row](row_coef);
        }

        string var_name = "X_" + to_string(col + 1 - num_y_cols);
        IloNum var_lb = 0;
        IloNum var_ub = IloInfinity;
        IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
        mp_vars.add(var_x);
        cplex_col.end();
    }

    // =========================================================================
    // 第四步: 求解主问题
    // =========================================================================
    cout << "[主问题] 求解初始主问题...\n";
    IloCplex mp_cplex(mp_model);
    mp_cplex.extract(mp_model);
    mp_cplex.setOut(mp_env.getNullStream());
    mp_cplex.exportModel("The First Master Problem.lp");
    bool mp_feasible = mp_cplex.solve();

    if (mp_feasible == 0) {
        cout << "[主问题] 初始主问题不可行\n";
    } else {
        cout << "[主问题] 初始主问题可行, 目标值 = "
             << fixed << setprecision(4) << mp_cplex.getValue(mp_obj) << "\n";
        cout.unsetf(ios::fixed);

        // ----- 统计非零解 -----
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

        cout << "[主问题] 非零解: Y=" << num_y_nonzero << ", X=" << num_x_nonzero << "\n";

        // =====================================================================
        // 提取对偶价格
        // =====================================================================
        root_node.duals_.clear();

        for (int row = 0; row < num_strip_types; row++) {
            double dual_val = mp_cplex.getDual(mp_cons[row]);
            if (dual_val == -0) dual_val = 0;
            root_node.duals_.push_back(dual_val);
        }

        for (int row = num_strip_types; row < num_strip_types + num_item_types; row++) {
            double dual_val = mp_cplex.getDual(mp_cons[row]);
            if (dual_val == -0) dual_val = 0;
            root_node.duals_.push_back(dual_val);
        }
    }

    return mp_feasible;
}
