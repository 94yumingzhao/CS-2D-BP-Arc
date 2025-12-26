// =============================================================================
// new_node_first_master_problem.cpp - 新节点初始主问题
// =============================================================================
// 功能: 构建并求解分支定界树中新节点的初始受限主问题
//
// 与根节点的区别:
//   根节点: 所有变量边界为 [0, +inf), 是纯粹的 LP 松弛
//   新节点: 部分变量被固定为整数值 (分支约束)
//
// 分支约束处理:
//   - 当前分支变量: 固定为 floor (左子节点) 或 ceil (右子节点)
//   - 历史分支变量: 保持之前确定的固定值
//   - 未分支变量: 保持 [0, +inf) 范围
// =============================================================================

#include "2DBP.h"

using namespace std;


// 求解新节点初始主问题
// cur_node: 当前要求解的新节点
// parent_node: 父节点 (包含分支变量信息)
// 返回: true=可行, false=不可行(需剪枝)
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
    // 问题规模
    int num_y_cols = cur_node.y_cols_.size();
    int num_x_cols = cur_node.x_cols_.size();
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_cols = num_y_cols + num_x_cols;
    int num_rows = num_strip_types + num_item_types;

    // 第一步: 构建约束 (与根节点相同)
    IloNumArray con_min(mp_env);
    IloNumArray con_max(mp_env);

    for (int row = 0; row < num_strip_types + num_item_types; row++) {
        // 前 J 行: 条带平衡约束 >= 0
        if (row < num_strip_types) {
            con_min.add(IloNum(0));
            con_max.add(IloNum(IloInfinity));
        }
        // 后 I 行: 子件需求约束 >= demand
        if (row >= num_strip_types) {
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

    // 第二步: 构建 Y 变量 (母板切割模式)
    // 需要根据分支信息设置变量边界
    for (int col = 0; col < num_y_cols; col++) {
        IloNum obj_coef = 1;  // 目标系数 = 1
        IloNumColumn cplex_col = mp_obj(obj_coef);

        // 设置约束系数
        for (int row = 0; row < num_rows; row++) {
            IloNum row_coef = cur_node.y_cols_[col][row];
            cplex_col += mp_cons[row](row_coef);
        }

        string var_name = "Y_" + to_string(col + 1);

        // 情况1: 当前列是本次分支的变量
        // branch_var_id_ 存储父节点选择的分支变量索引
        // branch_bound_ 存储当前节点的固定值 (左子=floor, 右子=ceil)
        if (col == parent_node.branch_var_id_) {
            IloNum var_lb = cur_node.branch_bound_;  // 固定为整数值
            IloNum var_ub = cur_node.branch_bound_;
            IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
            mp_vars.add(var_y);
        }
        // 情况2: 不是本次分支变量, 需要检查是否为历史分支变量
        else {
            int num_branched = parent_node.branched_vals_.size();
            bool find_flag = 0;

            // 遍历历史分支变量列表
            // branched_var_ids_: 存储所有已分支变量的索引
            // branched_vals_: 存储对应的固定值
            for (int index = 0; index < num_branched; index++) {
                int branched_idx = parent_node.branched_var_ids_[index];

                // 找到匹配: 该列是历史分支变量, 固定为之前确定的值
                if (col == branched_idx) {
                    IloNum var_lb = parent_node.branched_vals_[index];
                    IloNum var_ub = parent_node.branched_vals_[index];
                    IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                    mp_vars.add(var_y);

                    find_flag = 1;
                    break;
                }
            }

            // 未找到: 该列是未分支变量, 保持 [0, +inf) 范围
            if (find_flag == 0) {
                IloNum var_lb = 0;
                IloNum var_ub = IloInfinity;
                IloNumVar var_y(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                mp_vars.add(var_y);
            }
        }

        cplex_col.end();
    }

    // 第三步: 构建 X 变量 (条带切割模式)
    // 处理逻辑与 Y 变量类似
    for (int col = 0; col < num_x_cols; col++) {
        IloNum obj_coef = 0;  // 目标系数 = 0
        IloNumColumn cplex_col = mp_obj(obj_coef);

        // 设置约束系数
        for (int row = 0; row < num_rows; row++) {
            IloNum row_coef = cur_node.x_cols_[col][row];
            cplex_col += mp_cons[row](row_coef);
        }

        string var_name = "X_" + to_string(col + 1);

        // 情况1: 当前列是本次分支变量
        // 注意: X 变量的全局索引 = col + num_y_cols
        if (col + num_y_cols == parent_node.branch_var_id_) {
            IloNum var_lb = cur_node.branch_bound_;
            IloNum var_ub = cur_node.branch_bound_;
            IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
            mp_vars.add(var_x);
        }
        // 情况2: 不是本次分支变量
        else {
            int num_branched = parent_node.branched_vals_.size();
            bool find_flag = 0;

            // 遍历历史分支变量列表
            for (int pos = 0; pos < num_branched; pos++) {
                int branched_idx = parent_node.branched_var_ids_[pos];

                // 找到匹配: 历史分支变量
                if (col == branched_idx) {
                    IloNum var_lb = parent_node.branched_vals_[pos];
                    IloNum var_ub = parent_node.branched_vals_[pos];
                    IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                    mp_vars.add(var_x);

                    find_flag = 1;
                    break;
                }
            }

            // 未找到: 未分支变量
            if (find_flag == 0) {
                IloNum var_lb = 0;
                IloNum var_ub = IloInfinity;
                IloNumVar var_x(cplex_col, var_lb, var_ub, ILOFLOAT, var_name.c_str());
                mp_vars.add(var_x);
            }
        }

        cplex_col.end();
    }

    // 第四步: 求解主问题
    cout << "[节点_" << cur_node.id_ << "] 求解初始主问题...\n";
    IloCplex mp_cplex(mp_env);
    mp_cplex.extract(mp_model);
    mp_cplex.setOut(mp_env.getNullStream());
    mp_cplex.exportModel("New Node First Master Problem.lp");
    bool mp_feasible = mp_cplex.solve();

    // 不可行: 分支约束导致无可行解, 标记为剪枝
    // 这说明该分支方向无法产生可行的整数解
    if (mp_feasible == 0) {
        cur_node.prune_flag_ = 1;
        cout << "[节点_" << cur_node.id_ << "] 初始主问题不可行, 需剪枝\n";
    }
    // 可行: 提取解和对偶价格
    else {
        cout << "[节点_" << cur_node.id_ << "] 初始主问题可行, 目标值 = "
             << fixed << setprecision(4) << mp_cplex.getValue(mp_obj) << "\n";
        cout.unsetf(ios::fixed);

        // 统计非零解
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

        // 提取对偶价格 (用于子问题定价)
        cur_node.duals_.clear();

        double dual_val = -1;
        // 前 J 个: 条带平衡约束的对偶价格
        for (int row = 0; row < num_strip_types; row++) {
            dual_val = mp_cplex.getDual(mp_cons[row]);
            if (std::abs(dual_val) < kZeroTolerance) {
                dual_val = 0;
            }
            cur_node.duals_.push_back(dual_val);
        }

        // 后 I 个: 子件需求约束的对偶价格
        for (int row = num_strip_types; row < num_strip_types + num_item_types; row++) {
            dual_val = mp_cplex.getDual(mp_cons[row]);
            if (std::abs(dual_val) < kZeroTolerance) {
                dual_val = 0;
            }
            cur_node.duals_.push_back(dual_val);
        }
    }

    mp_cplex.end();
    return mp_feasible;
}
