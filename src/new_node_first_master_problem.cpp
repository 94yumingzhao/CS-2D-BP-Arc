// =============================================================================
// new_node_first_master_problem.cpp - 新节点初始主问题
// =============================================================================
//
// 功能: 构建并求解分支定界树中新节点的初始受限主问题
//
// 与根节点的区别:
//   根节点: 所有变量边界为 [0, +inf)
//   新节点: 部分变量被固定为整数值
//
// 变量边界处理逻辑:
//
// 数学模型:
//
//   min  sum_{k} y_k
//
//   s.t. sum_{k} c_{jk} * y_k - sum_{p} d_{jp} * x_p >= 0  (条带平衡)
//        sum_{p} b_{ip} * x_p >= demand_i                  (子件需求)
//        y_k = fixed_val  (对于已分支变量)
//        y_k >= 0         (对于未分支变量)
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SolveNodeInitMP - 求解新节点初始主问题
// -----------------------------------------------------------------------------
// 功能: 构建考虑分支约束的初始主问题并求解
//
// 参数:
//   params      - 全局参数
//   data        - 全局列表
//   mp_env      - CPLEX 环境
//   mp_model    - 主问题模型
//   mp_obj      - 目标函数对象
//   mp_cons     - 约束数组
//   mp_vars     - 变量数组
//   cur_node    - 当前新节点
//   parent_node - 父节点 (用于获取分支变量信息)
//
// 返回值:
//   true  = 问题可行
//   false = 问题不可行 (节点需剪枝)
// -----------------------------------------------------------------------------
bool SolveNodeInitMP(
    ProblemParams& params,
    ProblemData& data,
    IloEnv& mp_env,
    IloModel& mp_model,
    IloObjective& mp_obj,
    IloRangeArray& mp_cons,
    IloNumVarArray& mp_vars,
    BPNode& cur_node,
    BPNode& parent_node) {

    // =========================================================================
    // 问题规模
    // =========================================================================
    int num_y_cols = cur_node.y_cols_.size();
    int num_x_cols = cur_node.x_cols_.size();
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_cols = num_y_cols + num_x_cols;
    int num_rows = num_strip_types + num_item_types;

    // =========================================================================
    // 第一步: 构建约束
    // =========================================================================
    IloNumArray con_min(mp_env);
    IloNumArray con_max(mp_env);

    for (int row = 0; row < num_strip_types + num_item_types; row++) {
        if (row < num_strip_types) {
            // ----- 条带平衡约束: >= 0 -----
            con_min.add(IloNum(0));
            con_max.add(IloNum(IloInfinity));
        }
        if (row >= num_strip_types) {
            // ----- 子件需求约束: >= demand -----
            int row_pos = row - num_strip_types;
            double demand_val = data.item_types_[row_pos].demand_;
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
        // ----- 设置目标函数系数 -----
        IloNum obj_coef = 1;
        IloNumColumn cplex_col = mp_obj(obj_coef);

        // ----- 设置约束系数 -----
        for (int row = 0; row < num_rows; row++) {
            IloNum row_coef = cur_node.y_cols_[col][row];
            cplex_col += mp_cons[row](row_coef);
        }

        string var_name = "Y_" + to_string(col + 1);

        // =====================================================================
        // Case 1: 当前分支变量
        // =====================================================================
        // 固定为 floor (左子节点) 或 ceil (右子节点)
        // =====================================================================
        if (col == parent_node.branch_var_id_) {
            IloNum var_lb = cur_node.branch_bound_;
            IloNum var_ub = cur_node.branch_bound_;
            IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
            mp_vars.add(var_y);
        }
        // =====================================================================
        // Case 2: 非当前分支变量
        // =====================================================================
        else {
            int num_branched = parent_node.branched_vals_.size();
            bool find_flag = 0;

            // ----- Case 2.1: 检查是否为已分支变量 -----
            for (int index = 0; index < num_branched; index++) {
                int branched_idx = parent_node.branched_var_ids_[index];
                if (col == branched_idx) {
                    // 固定为之前确定的整数值
                    IloNum var_lb = parent_node.branched_vals_[index];
                    IloNum var_ub = parent_node.branched_vals_[index];
                    IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                    mp_vars.add(var_y);

                    find_flag = 1;
                    break;
                }
            }

            // ----- Case 2.2: 未分支变量 -----
            if (find_flag == 0) {
                // 保持 [0, +inf) 范围
                IloNum var_lb = 0;
                IloNum var_ub = IloInfinity;
                IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                mp_vars.add(var_y);
            }
        }

        cplex_col.end();
    }

    // =========================================================================
    // 第三步: 构建 X 变量 (条带切割模式)
    // =========================================================================
    for (int col = 0; col < num_x_cols; col++) {
        // ----- 设置目标函数系数 -----
        IloNum obj_coef = 0;
        IloNumColumn cplex_col = mp_obj(obj_coef);

        // ----- 设置约束系数 -----
        for (int row = 0; row < num_rows; row++) {
            IloNum row_coef = cur_node.x_cols_[col][row];
            cplex_col += mp_cons[row](row_coef);
        }

        string var_name = "X_" + to_string(col + 1);

        // =====================================================================
        // Case 1: 当前分支变量
        // =====================================================================
        if (col + num_y_cols == parent_node.branch_var_id_) {
            IloNum var_lb = cur_node.branch_bound_;
            IloNum var_ub = cur_node.branch_bound_;
            IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
            mp_vars.add(var_x);
        }
        // =====================================================================
        // Case 2: 非当前分支变量
        // =====================================================================
        else {
            int num_branched = parent_node.branched_vals_.size();
            bool find_flag = 0;

            // ----- Case 2.1: 检查是否为已分支变量 -----
            for (int pos = 0; pos < num_branched; pos++) {
                int branched_idx = parent_node.branched_var_ids_[pos];
                if (col == branched_idx) {
                    IloNum var_lb = parent_node.branched_vals_[pos];
                    IloNum var_ub = parent_node.branched_vals_[pos];
                    IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                    mp_vars.add(var_x);

                    find_flag = 1;
                    break;
                }
            }

            // ----- Case 2.2: 未分支变量 -----
            if (find_flag == 0) {
                IloNum var_lb = 0;
                IloNum var_ub = IloInfinity;
                IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                mp_vars.add(var_x);
            }
        }

        cplex_col.end();
    }

    // =========================================================================
    // 第四步: 求解主问题
    // =========================================================================
    cout << "[节点_" << cur_node.id_ << "] 求解初始主问题...\n";
    IloCplex mp_cplex(mp_env);
    mp_cplex.extract(mp_model);
    mp_cplex.setOut(mp_env.getNullStream());
    mp_cplex.exportModel("New Node First Master Problem.lp");
    bool mp_feasible = mp_cplex.solve();

    if (mp_feasible == 0) {
        // ----- 不可行: 标记剪枝 -----
        cur_node.prune_flag_ = 1;
        cout << "[节点_" << cur_node.id_ << "] 初始主问题不可行, 需剪枝\n";
    }
    else {
        // ----- 可行: 输出结果并提取对偶价格 -----
        cout << "[节点_" << cur_node.id_ << "] 初始主问题可行, 目标值 = "
             << fixed << setprecision(4) << mp_cplex.getValue(mp_obj) << "\n";
        cout.unsetf(ios::fixed);

        // ----- 统计非零解 -----
        int num_y_nonzero = 0;
        int num_x_nonzero = 0;
        double sol_val = -1;

        for (int col = 0; col < num_y_cols; col++) {
            sol_val = mp_cplex.getValue(mp_vars[col]);
            if (sol_val > 0) num_y_nonzero++;
        }

        for (int col = num_y_cols; col < num_y_cols + num_x_cols; col++) {
            sol_val = mp_cplex.getValue(mp_vars[col]);
            if (sol_val > 0) num_x_nonzero++;
        }

        cout << "[节点_" << cur_node.id_ << "] 非零解: Y=" << num_y_nonzero << ", X=" << num_x_nonzero << "\n";

        // =====================================================================
        // 提取对偶价格 (用于子问题定价)
        // =====================================================================
        cur_node.duals_.clear();

        double dual_val = -1;
        for (int row = 0; row < num_strip_types; row++) {
            dual_val = mp_cplex.getDual(mp_cons[row]);
            if (dual_val == -0) {
                dual_val = 0;
            }
            cur_node.duals_.push_back(dual_val);
        }

        for (int row = num_strip_types; row < num_strip_types + num_item_types; row++) {
            dual_val = mp_cplex.getDual(mp_cons[row]);
            if (dual_val == -0) {
                dual_val = 0;
            }
            cur_node.duals_.push_back(dual_val);
        }
    }

    mp_cplex.end();
    return mp_feasible;
}
