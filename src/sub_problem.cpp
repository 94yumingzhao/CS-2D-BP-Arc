// =============================================================================
// sub_problem.cpp - 子问题求解模块
// =============================================================================
// 功能: 实现列生成算法中的定价子问题求解
//
// 子问题结构:
//   SP1 (第一阶段): 宽度背包问题, 决定母板上放置哪些条带类型
//   SP2 (第二阶段): 长度背包问题, 决定条带上放置哪些子件类型
//
// 定价原理:
//   - 列生成通过求解子问题寻找 reduced cost > 0 的新列
//   - reduced cost = 子问题目标值 - 对应变量的目标系数
//   - Y变量: rc = SP1目标值 - 1, 若 > 0 则添加新 Y 列
//   - X变量: rc = SP2目标值 - v_j, 若 > 0 则添加新 X 列
// =============================================================================

#include "2DBP.h"

using namespace std;


// 求解第一阶段子问题 (宽度背包)
// 目标: max sum(v_j * G_j) - 1, 其中 v_j 是条带约束的对偶价格
// 约束: sum(w_j * G_j) <= W (母板宽度限制)
// 返回: 1=找到改进列, 0=无改进列
int SolveSP1(ProblemParams& params, ProblemData& data, BPNode& node) {
    int num_y_cols = node.y_cols_.size();
    int num_x_cols = node.x_cols_.size();
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int has_improvement = -1;

    // 构建 SP1 CPLEX 模型
    IloEnv sp1_env;
    IloModel sp1_model(sp1_env);
    IloNumVarArray g_vars(sp1_env);

    // 创建决策变量 G_j: 条带类型 j 在母板上的放置数量
    for (int j = 0; j < num_strip_types; j++) {
        IloNum var_lb = 0;
        IloNum var_ub = IloInfinity;
        string var_name = "G_" + to_string(j + 1);
        IloNumVar var_g(sp1_env, var_lb, var_ub, ILOINT, var_name.c_str());
        g_vars.add(var_g);
    }

    // 构建目标函数: max sum(v_j * G_j)
    // v_j = node.duals_[j] 是条带平衡约束的对偶价格
    IloExpr obj_expr(sp1_env);
    for (int j = 0; j < num_strip_types; j++) {
        double dual_price = node.duals_[j];
        obj_expr += dual_price * g_vars[j];
    }
    IloObjective sp1_obj = IloMaximize(sp1_env, obj_expr);
    sp1_model.add(sp1_obj);
    obj_expr.end();

    // 构建宽度约束: sum(w_j * G_j) <= W
    // w_j = 条带类型 j 的宽度, W = 母板宽度
    IloExpr con_expr(sp1_env);
    for (int j = 0; j < num_strip_types; j++) {
        double strip_wid = data.strip_types_[j].width_;
        con_expr += strip_wid * g_vars[j];
    }
    sp1_model.add(con_expr <= params.stock_width_);
    con_expr.end();

    // 求解 SP1
    IloCplex sp1_cplex(sp1_env);
    sp1_cplex.extract(sp1_model);
    sp1_cplex.setOut(sp1_env.getNullStream());
    bool sp1_solved = sp1_cplex.solve();

    // SP1 不可行 (理论上不应发生, 因为 G=0 总是可行的)
    if (sp1_solved == 0) {
        cout << "[子问题] SP1 不可行\n";
    }
    // SP1 可行, 检查是否找到改进列
    else {
        double sp1_obj_val = sp1_cplex.getValue(sp1_obj);

        // 提取 SP1 的解, 构建新的 Y 列系数向量
        // Y 列结构: [条带产出数量 G_1...G_J, 子件产出数量 0...0]
        node.new_y_col_.clear();
        for (int j = 0; j < num_strip_types; j++) {
            double sol_val = sp1_cplex.getValue(g_vars[j]);
            if (std::abs(sol_val) < kZeroTolerance) sol_val = 0;
            node.new_y_col_.push_back(sol_val);
        }
        // Y 列不直接产出子件, 填充 0
        for (int j = 0; j < num_item_types; j++) {
            node.new_y_col_.push_back(0);
        }

        // 定价判断: reduced cost = SP1目标值 - 1
        // 若 rc > 0, 说明新列可以改进目标函数
        if (sp1_obj_val > 1 + kRcTolerance) {
            // 找到改进的 Y 列, 添加到主问题
            cout << "[子问题] SP1 找到改进列 (rc=" << fixed << setprecision(4)
                 << (sp1_obj_val - 1) << ")\n";
            cout.unsetf(ios::fixed);
            node.col_type_flag_ = 1;  // 标记为 Y 列
            has_improvement = 1;
        }
        // SP1 无改进列 (rc <= 0), 继续求解 SP2
        else {
            node.new_y_col_.clear();
            node.new_x_cols_.clear();
            node.col_type_flag_ = 0;  // 标记为 X 列
            node.sp2_obj_ = -1;

            // 对每种条带类型 k 求解对应的 SP2
            // 因为 X 列与条带类型绑定, 每种条带需要单独求解
            int is_feasible = 0;
            int sp2_solved = -1;

            for (int k = 0; k < num_strip_types; k++) {
                sp2_solved = SolveSP2(params, data, node, k + 1);

                // SP2 有解且找到改进列
                if (sp2_solved == 1) {
                    // v_k = 条带类型 k 的对偶价格
                    double dual_val = node.duals_[k];

                    // 定价判断: reduced cost = SP2目标值 - v_k
                    // 若 rc > 0, 说明新的 X 列可以改进目标函数
                    if (node.sp2_obj_ > dual_val + kRcTolerance) {
                        is_feasible = 1;

                        // 构建新的 X 列系数向量
                        // X 列结构: [条带消耗 -1(仅位置k), 子件产出 D_1...D_I]
                        vector<double> temp_col;

                        // 条带消耗部分: 位置 k 为 -1 (消耗一个条带), 其余为 0
                        for (int j = 0; j < num_strip_types; j++) {
                            temp_col.push_back((k == j) ? -1 : 0);
                        }

                        // 子件产出部分: 来自 SP2 的解
                        for (int i = 0; i < num_item_types; i++) {
                            double sol_val = node.sp2_solution_[i];
                            if (std::abs(sol_val) < kZeroTolerance) sol_val = 0;
                            temp_col.push_back(sol_val);
                        }

                        node.new_x_cols_.push_back(temp_col);
                    }
                }
            }

            // 输出 SP2 求解结果
            if (is_feasible == 0) {
                // 所有条带类型都没有找到改进列, 列生成收敛
                cout << "[子问题] 未找到改进列, 列生成收敛\n";
            } else {
                cout << "[子问题] SP2 找到 " << node.new_x_cols_.size() << " 个改进列\n";
            }

            has_improvement = is_feasible;
        }
    }

    // 释放 CPLEX 资源
    sp1_obj.removeAllProperties();
    sp1_obj.end();
    g_vars.clear();
    g_vars.end();
    sp1_model.removeAllProperties();
    sp1_model.end();
    sp1_env.removeAllProperties();
    sp1_env.end();

    return has_improvement;
}


// 求解第二阶段子问题 (长度背包)
// 目标: max sum(w_i * D_i), 其中 w_i 是子件约束的对偶价格
// 约束: sum(l_i * D_i) <= L (母板长度限制)
//       仅考虑宽度 <= 条带宽度的子件类型
// 参数: strip_type_id = 当前条带类型编号 (1-based)
// 返回: 1=求解成功, 0=无可行子件或无需求解
int SolveSP2(ProblemParams& params, ProblemData& data, BPNode& node, int strip_type_id) {
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int result = -1;

    // 构建 SP2 CPLEX 模型
    IloEnv sp2_env;
    IloModel sp2_model(sp2_env);
    IloNumVarArray d_vars(sp2_env);

    // 创建决策变量 D_i: 子件类型 i 在条带上的放置数量
    for (int i = 0; i < num_item_types; i++) {
        IloNum var_lb = 0;
        IloNum var_ub = IloInfinity;
        string var_name = "D_" + to_string(i + 1);
        IloNumVar var_d(sp2_env, var_lb, var_ub, ILOINT, var_name.c_str());
        d_vars.add(var_d);
    }

    // 构建目标函数和长度约束
    IloObjective sp2_obj(sp2_env);
    IloExpr obj_expr(sp2_env);
    IloExpr len_expr(sp2_env);
    double sum_coef = 0;
    vector<int> feasible_ids;

    // 获取当前条带类型的宽度
    double strip_wid = data.strip_types_[strip_type_id - 1].width_;

    for (int i = 0; i < num_item_types; i++) {
        // 条件1: 子件宽度必须 <= 条带宽度 (几何约束)
        if (data.item_types_[i].width_ <= strip_wid) {
            // 获取子件约束的对偶价格 w_i
            // 对偶价格存储顺序: [v_1...v_J, w_1...w_I]
            int row_idx = i + num_strip_types;
            double dual_price = node.duals_[row_idx];

            // 条件2: 对偶价格必须 > 0 (否则该子件对目标无贡献)
            if (dual_price > 0) {
                obj_expr += dual_price * d_vars[i];
                sum_coef += dual_price;
                double item_len = data.item_types_[i].length_;
                len_expr += item_len * d_vars[i];
                feasible_ids.push_back(i);
            }
        }
    }

    // 判断是否需要求解
    // 若 sum_coef = 0, 说明没有子件能贡献正的目标值, 无需求解
    if (sum_coef > 0) {
        sp2_obj = IloMaximize(sp2_env, obj_expr);
        sp2_model.add(sp2_obj);
        sp2_model.add(len_expr <= params.stock_length_);
        obj_expr.end();
        len_expr.end();
        result = 1;
    }
    // 无可行子件, 直接返回
    else {
        len_expr.end();
        sp2_obj.removeAllProperties();
        sp2_obj.end();
        d_vars.clear();
        d_vars.end();
        sp2_model.removeAllProperties();
        sp2_model.end();
        sp2_env.removeAllProperties();
        sp2_env.end();
        return 0;
    }

    // 求解 SP2
    IloCplex sp2_cplex(sp2_env);
    sp2_cplex.extract(sp2_model);
    sp2_cplex.setOut(sp2_env.getNullStream());
    bool sp2_solved = sp2_cplex.solve();

    // SP2 不可行 (理论上不应发生)
    if (sp2_solved == 0) {
        // SP2 不可行
    }
    // SP2 可行, 提取解
    else {
        node.sp2_obj_ = sp2_cplex.getValue(sp2_obj);
        node.sp2_solution_.clear();

        int num_feasible = feasible_ids.size();

        // 构建完整的解向量 (包括不可行的子件类型, 其值为0)
        for (int i = 0; i < num_item_types; i++) {
            int is_feasible = -1;

            // 检查子件 i 是否在可行列表中
            for (int k = 0; k < num_feasible; k++) {
                if (i == feasible_ids[k]) {
                    is_feasible = 1;
                    double sol_val = sp2_cplex.getValue(d_vars[i]);
                    if (std::abs(sol_val) < kZeroTolerance) sol_val = 0;
                    node.sp2_solution_.push_back(sol_val);
                    break;
                }
            }

            // 不可行的子件类型, 填充 0
            if (is_feasible != 1) {
                node.sp2_solution_.push_back(0);
            }
        }
    }

    // 释放 CPLEX 资源
    sp2_obj.removeAllProperties();
    sp2_obj.end();
    d_vars.clear();
    d_vars.end();
    sp2_model.removeAllProperties();
    sp2_model.end();
    sp2_env.removeAllProperties();
    sp2_env.end();

    return result;
}
