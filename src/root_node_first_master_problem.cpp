// =============================================================================
// root_node_first_master_problem.cpp - 根节点初始主问题
// =============================================================================
// 功能: 构建并求解分支定价树根节点的初始受限主问题 (RMP)
//
// 主问题结构:
//   min  sum(y_k)                          (最小化母板使用数量)
//   s.t. sum(C_jk * y_k) - sum(x_p) >= 0   (条带平衡约束, j=1..J)
//        sum(B_ip * x_p) >= d_i            (子件需求约束, i=1..I)
//        y_k >= 0, x_p >= 0
//
// 矩阵含义:
//   C_jk: 母板模式 k 产出条带类型 j 的数量
//   B_ip: 条带模式 p 产出子件类型 i 的数量
//   d_i:  子件类型 i 的需求量
// =============================================================================

#include "2DBP.h"

using namespace std;


// 求解根节点初始主问题
// 返回: true=可行, false=不可行
bool SolveRootInitMP(
    ProblemParams& params,
    ProblemData& data,
    IloEnv& mp_env,
    IloModel& mp_model,
    IloObjective& mp_obj,
    IloRangeArray& mp_cons,
    IloNumVarArray& mp_vars,
    BPNode& root_node) {
    // 问题规模
    int num_y_cols = root_node.y_cols_.size();   // Y 列数量 (母板模式数)
    int num_x_cols = root_node.x_cols_.size();   // X 列数量 (条带模式数)
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_cols = num_y_cols + num_x_cols;
    int num_rows = num_strip_types + num_item_types;

    cout << "[主问题] 构建初始主问题 (Y=" << num_y_cols << ", X=" << num_x_cols << ")\n";

    // 第一步: 构建约束
    // 约束分为两类: 条带平衡约束 (前 J 行) 和 子件需求约束 (后 I 行)
    IloNumArray con_min(mp_env);
    IloNumArray con_max(mp_env);

    for (int row = 0; row < num_strip_types + num_item_types; row++) {
        // 前 J 行: 条带平衡约束 sum(C_jk * y_k) - sum(x_p) >= 0
        // 含义: 母板产出的条带数量 >= 切割消耗的条带数量
        if (row < num_strip_types) {
            con_min.add(IloNum(0));           // 下界 = 0
            con_max.add(IloNum(IloInfinity)); // 上界 = +inf
        }
        // 后 I 行: 子件需求约束 sum(B_ip * x_p) >= d_i
        // 含义: 条带产出的子件数量 >= 子件需求量
        if (row >= num_strip_types) {
            int row_idx = row - num_strip_types;
            double demand_val = data.item_types_[row_idx].demand_;
            con_min.add(IloNum(demand_val));  // 下界 = 需求量
            con_max.add(IloNum(IloInfinity)); // 上界 = +inf
        }
    }

    mp_cons = IloRangeArray(mp_env, con_min, con_max);
    mp_model.add(mp_cons);
    con_min.end();
    con_max.end();

    // 第二步: 构建 Y 变量 (母板切割模式)
    // Y 变量的目标系数 = 1 (每使用一块母板, 目标值 +1)
    for (int col = 0; col < num_y_cols; col++) {
        IloNum obj_coef = 1;  // 目标系数: 母板使用数量
        IloNumColumn cplex_col = mp_obj(obj_coef);

        // 添加约束系数 (列向量)
        for (int row = 0; row < num_strip_types + num_item_types; row++) {
            IloNum row_coef = root_node.matrix_[col][row];
            cplex_col += mp_cons[row](row_coef);
        }

        string var_name = "Y_" + to_string(col + 1);
        IloNum var_lb = 0;          // 下界 = 0
        IloNum var_ub = IloInfinity; // 上界 = +inf (LP松弛)
        IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
        mp_vars.add(var_y);
        cplex_col.end();
    }

    // 第三步: 构建 X 变量 (条带切割模式)
    // X 变量的目标系数 = 0 (条带模式不直接计入目标)
    for (int col = num_y_cols; col < num_y_cols + num_x_cols; col++) {
        IloNum obj_coef = 0;  // 目标系数: 0
        IloNumColumn cplex_col = mp_obj(obj_coef);

        // 添加约束系数 (列向量)
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

    // 第四步: 求解主问题
    cout << "[主问题] 求解初始主问题...\n";
    IloCplex mp_cplex(mp_model);
    mp_cplex.extract(mp_model);
    mp_cplex.setOut(mp_env.getNullStream());
    mp_cplex.exportModel("The First Master Problem.lp");
    bool mp_feasible = mp_cplex.solve();

    // 不可行: 初始列集合无法满足所有需求约束
    if (mp_feasible == 0) {
        cout << "[主问题] 初始主问题不可行\n";
    }
    // 可行: 提取解和对偶价格
    else {
        cout << "[主问题] 初始主问题可行, 目标值 = "
             << fixed << setprecision(4) << mp_cplex.getValue(mp_obj) << "\n";
        cout.unsetf(ios::fixed);

        // 统计非零解 (用于调试)
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

        // 提取对偶价格 (用于子问题定价)
        // 对偶价格 = 约束的影子价格, 表示放松约束一单位能改善目标多少
        root_node.duals_.clear();

        // 前 J 个对偶价格: 条带平衡约束的对偶 (v_j)
        for (int row = 0; row < num_strip_types; row++) {
            double dual_val = mp_cplex.getDual(mp_cons[row]);
            if (std::abs(dual_val) < kZeroTolerance) dual_val = 0;
            root_node.duals_.push_back(dual_val);
        }

        // 后 I 个对偶价格: 子件需求约束的对偶 (w_i)
        for (int row = num_strip_types; row < num_strip_types + num_item_types; row++) {
            double dual_val = mp_cplex.getDual(mp_cons[row]);
            if (std::abs(dual_val) < kZeroTolerance) dual_val = 0;
            root_node.duals_.push_back(dual_val);
        }
    }

    mp_cplex.end();
    return mp_feasible;
}
