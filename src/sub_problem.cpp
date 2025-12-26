// =============================================================================
// sub_problem.cpp - 子问题求解模块
// =============================================================================
//
// 功能: 实现列生成算法中的定价子问题求解
//
// -----------------------------------------------------------------------------
// 子问题概述
// -----------------------------------------------------------------------------
//
// 在列生成框架中, 子问题的作用是寻找能够改进当前解的新切割模式 (列)。
// 根据两阶段切割的特点, 有两个子问题:
//
// 1. 第一阶段子问题 (SP1) - 宽度方向背包:
//    - 决定在母板宽度方向上如何组合不同类型的条带
//    - 目标: 最大化条带类型的对偶价格和
//    - 约束: 条带宽度之和 <= 母板宽度
//
// 2. 第二阶段子问题 (SP2) - 长度方向背包:
//    - 决定在条带长度方向上如何组合不同类型的子件
//    - 目标: 最大化子件类型的对偶价格和
//    - 约束: 子件长度之和 <= 条带长度 (= 母板长度)
//
// -----------------------------------------------------------------------------
// 定价判断规则
// -----------------------------------------------------------------------------
//
// 对于主问题: min sum(Y_k), 其对偶问题为 max 形式
//
// SP1 的改进列判断:
//   - Y 变量的目标系数 = 1
//   - 若 SP1_obj > 1 + RC_EPS, 则找到改进的 Y 列
//
// SP2 的改进列判断:
//   - X 变量的目标系数 = 0
//   - 对偶约束为: sum(系数 * 对偶价格) <= 0
//   - 变形后: SP2_obj - lambda_j > RC_EPS 时, 找到改进的 X 列
//   - 其中 lambda_j 是条带类型 j 的对偶价格
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// SolveStageOneSubProblem - 求解第一阶段子问题 (宽度背包)
// =============================================================================
//
// 数学模型:
//
//   max  sum_{j=1}^{J} lambda_j * G_j
//
//   s.t. sum_{j=1}^{J} w_j * G_j <= W
//
//        G_j >= 0, 整数
//
// 其中:
//   - J = 条带类型数
//   - lambda_j = 条带类型 j 的对偶价格 (从主问题获取)
//   - w_j = 条带类型 j 的宽度
//   - W = 母板宽度
//   - G_j = 条带类型 j 的选取数量 (决策变量)
//
// 算法流程:
//   1. 构建 CPLEX 整数规划模型
//   2. 求解模型
//   3. 判断是否找到改进列:
//      a. 若 obj > 1 + RC_EPS, 生成新的 Y 列
//      b. 否则, 继续求解 SP2 寻找 X 列
//
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   this_node - 当前节点 (读取对偶价格, 输出新列)
//
// 返回值:
//   0 = 未找到改进列 (列生成收敛)
//   1 = 找到改进列 (需更新主问题)
//
// =============================================================================
int SolveStageOneSubProblem(All_Values& Values, All_Lists& Lists, Node& this_node) {

    // -------------------------------------------------------------------------
    // 获取问题规模参数
    // -------------------------------------------------------------------------
    int K_num = this_node.Y_cols_list.size();   // 当前 Y 变量数
    int P_num = this_node.X_cols_list.size();   // 当前 X 变量数
    int J_num = Values.strip_types_num;         // 条带类型数
    int N_num = Values.item_types_num;          // 子件类型数
    int all_cols_num = K_num + P_num;
    int all_rows_num = J_num + N_num;

    int loop_continue_flag = -1;  // 返回值: 是否找到改进列

    // =========================================================================
    // 构建 SP1 CPLEX 模型
    // =========================================================================
    IloEnv Env_SP1;
    IloModel Model_SP1(Env_SP1);
    IloNumVarArray Ga_Vars(Env_SP1);  // G 变量数组

    // -------------------------------------------------------------------------
    // 创建决策变量 G_j (j = 1, ..., J)
    // -------------------------------------------------------------------------
    for (int j = 0; j < J_num; j++) {
        IloNum var_min = 0;
        IloNum var_max = IloInfinity;
        string var_name = "G_" + to_string(j + 1);

        // 整数变量
        IloNumVar Var_Ga(Env_SP1, var_min, var_max, ILOINT, var_name.c_str());
        Ga_Vars.add(Var_Ga);
    }

    // -------------------------------------------------------------------------
    // 构建目标函数: max sum(lambda_j * G_j)
    // -------------------------------------------------------------------------
    // lambda_j = 条带约束的对偶价格 (存储在 dual_prices_list 的前 J 个元素)
    IloExpr obj_sum(Env_SP1);
    for (int j = 0; j < J_num; j++) {
        double val = this_node.dual_prices_list[j];  // lambda_j
        obj_sum += val * Ga_Vars[j];
    }
    IloObjective Obj_SP1 = IloMaximize(Env_SP1, obj_sum);
    Model_SP1.add(Obj_SP1);
    obj_sum.end();

    // -------------------------------------------------------------------------
    // 构建宽度约束: sum(w_j * G_j) <= W
    // -------------------------------------------------------------------------
    IloExpr con_sum(Env_SP1);
    for (int j = 0; j < J_num; j++) {
        double val = Lists.all_strip_types_list[j].width;  // w_j
        con_sum += val * Ga_Vars[j];
    }
    Model_SP1.add(con_sum <= Values.stock_width);  // <= W
    con_sum.end();

    // =========================================================================
    // 求解 SP1
    // =========================================================================
    IloCplex Cplex_SP1(Env_SP1);
    Cplex_SP1.extract(Model_SP1);
    Cplex_SP1.setOut(Env_SP1.getNullStream());  // 关闭 CPLEX 输出
    bool SP1_flag = Cplex_SP1.solve();

    if (SP1_flag == 0) {
        // SP1 不可行 (理论上不应发生)
        cout << "[子问题] SP1 不可行\n";
    } else {
        double SP1_obj_val = Cplex_SP1.getValue(Obj_SP1);

        // ---------------------------------------------------------------------
        // 提取 SP1 的解, 构建新的 Y 列
        // ---------------------------------------------------------------------
        this_node.new_Y_col.clear();

        // 前 J 行: 条带类型产出数量
        for (int j = 0; j < J_num; j++) {
            double soln_val = Cplex_SP1.getValue(Ga_Vars[j]);
            if (soln_val == -0) soln_val = 0;  // 处理 -0 的情况
            this_node.new_Y_col.push_back(soln_val);
        }

        // 后 N 行: 全为 0 (Y 列不直接产出子件)
        for (int j = 0; j < N_num; j++) {
            this_node.new_Y_col.push_back(0);
        }

        // ---------------------------------------------------------------------
        // 定价判断: 决定是否添加新列
        // ---------------------------------------------------------------------
        if (SP1_obj_val > 1 + RC_EPS) {
            // --------- 找到改进的 Y 列 ---------
            // 检验数 = SP1_obj - 1 > RC_EPS
            cout << "[子问题] SP1 找到改进列 (rc=" << fixed << setprecision(4)
                 << (SP1_obj_val - 1) << ")\n";
            cout.unsetf(ios::fixed);

            this_node.Y_col_flag = 1;     // 标记为 Y 列
            loop_continue_flag = 1;        // 继续迭代

        } else {
            // --------- SP1 无改进列, 继续求解 SP2 ---------
            // 清空 Y 列 (不使用)
            this_node.new_Y_col.clear();
            this_node.new_X_cols_list.clear();
            this_node.Y_col_flag = 0;      // 标记为 X 列
            this_node.SP2_obj_val = -1;

            // -----------------------------------------------------------------
            // 对每种条带类型求解 SP2
            // -----------------------------------------------------------------
            int feasible_flag = 0;  // 是否找到任何改进的 X 列
            int SP2_flag = -1;

            for (int k = 0; k < J_num; k++) {
                // 求解条带类型 k+1 的 SP2
                SP2_flag = SolveStageTwoSubProblem(Values, Lists, this_node, k + 1);

                if (SP2_flag == 1) {
                    // SP2 可行, 判断是否为改进列
                    double a_val = this_node.dual_prices_list[k];  // lambda_k

                    // 检验数 = SP2_obj - lambda_k
                    if (this_node.SP2_obj_val > a_val + RC_EPS) {
                        feasible_flag = 1;  // 找到改进列

                        // 构建新的 X 列
                        vector<double> temp_col;

                        // 前 J 行: D 矩阵部分 (-1 表示属于该条带类型)
                        for (int j = 0; j < J_num; j++) {
                            if (k == j) {
                                temp_col.push_back(-1);  // 属于条带类型 k
                            } else {
                                temp_col.push_back(0);   // 不属于
                            }
                        }

                        // 后 N 行: B 矩阵部分 (子件产出数量)
                        for (int i = 0; i < N_num; i++) {
                            double soln_val = this_node.SP2_solns_list[i];
                            if (soln_val == -0) soln_val = 0;
                            temp_col.push_back(soln_val);
                        }

                        this_node.new_X_cols_list.push_back(temp_col);
                    }
                }
            }

            // -----------------------------------------------------------------
            // 输出 SP2 求解结果
            // -----------------------------------------------------------------
            if (feasible_flag == 0) {
                cout << "[子问题] 未找到改进列, 列生成收敛\n";
            } else {
                cout << "[子问题] SP2 找到 " << this_node.new_X_cols_list.size()
                     << " 个改进列\n";
            }

            loop_continue_flag = feasible_flag;
        }
    }

    // =========================================================================
    // 释放 CPLEX 资源
    // =========================================================================
    Obj_SP1.removeAllProperties();
    Obj_SP1.end();
    Ga_Vars.clear();
    Ga_Vars.end();
    Model_SP1.removeAllProperties();
    Model_SP1.end();
    Env_SP1.removeAllProperties();
    Env_SP1.end();

    return loop_continue_flag;
}


// =============================================================================
// SolveStageTwoSubProblem - 求解第二阶段子问题 (长度背包)
// =============================================================================
//
// 数学模型:
//
//   max  sum_{i: w_i <= w_j} mu_i * D_i
//
//   s.t. sum_{i: w_i <= w_j} l_i * D_i <= L
//
//        D_i >= 0, 整数
//
// 其中:
//   - N = 子件类型数
//   - mu_i = 子件类型 i 的对偶价格 (从主问题获取)
//   - l_i = 子件类型 i 的长度
//   - w_i = 子件类型 i 的宽度
//   - w_j = 条带类型 j 的宽度
//   - L = 母板长度 (= 条带长度)
//   - D_i = 子件类型 i 的选取数量 (决策变量)
//
// 宽度可行性:
//   - 只有宽度 <= 条带宽度的子件才能放入该类型条带
//   - 这通过只包含满足 w_i <= w_j 的子件来实现
//
// 参数:
//   Values         - 全局参数
//   Lists          - 全局列表
//   this_node      - 当前节点 (读取对偶价格, 输出解)
//   strip_type_idx - 条带类型索引 (1-based)
//
// 返回值:
//   0 = SP2 不可行或无正对偶价格的子件
//   1 = SP2 求解成功
//
// =============================================================================
int SolveStageTwoSubProblem(All_Values& Values, All_Lists& Lists, Node& this_node, int strip_type_idx) {

    // -------------------------------------------------------------------------
    // 获取问题规模参数
    // -------------------------------------------------------------------------
    int all_cols_num = this_node.model_matrix.size();
    int strip_types_num = Values.strip_types_num;
    int item_types_num = Values.item_types_num;
    int J_num = strip_types_num;
    int N_num = item_types_num;

    int final_return = -1;  // 返回值

    // =========================================================================
    // 构建 SP2 CPLEX 模型
    // =========================================================================
    IloEnv Env_SP2;
    IloModel Model_SP2(Env_SP2);
    IloNumVarArray De_Vars(Env_SP2);  // D 变量数组

    // -------------------------------------------------------------------------
    // 创建决策变量 D_i (i = 1, ..., N)
    // -------------------------------------------------------------------------
    for (int i = 0; i < N_num; i++) {
        IloNum var_min = 0;
        IloNum var_max = IloInfinity;
        string var_name = "D_" + to_string(i + 1);

        // 整数变量
        IloNumVar De_Var(Env_SP2, var_min, var_max, ILOINT, var_name.c_str());
        De_Vars.add(De_Var);
    }

    // -------------------------------------------------------------------------
    // 构建目标函数和长度约束
    // -------------------------------------------------------------------------
    // 只考虑宽度可行的子件 (w_i <= w_j) 且对偶价格 > 0

    IloObjective Obj_SP2(Env_SP2);
    IloExpr obj_sum(Env_SP2);   // 目标函数表达式
    IloExpr sum_1(Env_SP2);     // 长度约束表达式
    double sum_val = 0;         // 目标系数之和 (用于判断是否有正价格子件)
    vector<int> fsb_idx;        // 可行子件的索引列表

    // 获取当前条带类型的宽度
    double strip_width = Lists.all_strip_types_list[strip_type_idx - 1].width;

    for (int i = 0; i < N_num; i++) {
        // 检查宽度可行性: w_i <= w_j
        if (Lists.all_item_types_list[i].width <= strip_width) {

            // 获取子件类型 i 的对偶价格
            // 注意: 子件约束的对偶价格存储在 dual_prices_list 的后 N 个元素
            int row_pos = i + J_num;  // 跳过前 J 个条带约束
            double b_val = this_node.dual_prices_list[row_pos];  // mu_i

            // 只考虑对偶价格 > 0 的子件
            if (b_val > 0) {
                // 添加到目标函数
                obj_sum += b_val * De_Vars[i];
                sum_val += b_val;

                // 添加到长度约束
                double l_val = Lists.all_item_types_list[i].length;  // l_i
                sum_1 += l_val * De_Vars[i];

                // 记录可行子件索引
                fsb_idx.push_back(i);
            }
        }
    }

    // -------------------------------------------------------------------------
    // 判断是否需要求解
    // -------------------------------------------------------------------------
    if (sum_val > 0) {
        // 存在正对偶价格的可行子件, 构建并求解模型
        Obj_SP2 = IloMaximize(Env_SP2, obj_sum);
        Model_SP2.add(Obj_SP2);
        Model_SP2.add(sum_1 <= Values.stock_length);  // <= L
        obj_sum.end();
        sum_1.end();
        final_return = 1;

    } else {
        // 没有正对偶价格的可行子件, 无需求解
        sum_1.end();
        Obj_SP2.removeAllProperties();
        Obj_SP2.end();
        De_Vars.clear();
        De_Vars.end();
        Model_SP2.removeAllProperties();
        Model_SP2.end();
        Env_SP2.removeAllProperties();
        Env_SP2.end();
        return 0;  // 直接返回
    }

    // =========================================================================
    // 求解 SP2
    // =========================================================================
    IloCplex Cplex_SP2(Env_SP2);
    Cplex_SP2.extract(Model_SP2);
    Cplex_SP2.setOut(Env_SP2.getNullStream());  // 关闭 CPLEX 输出
    bool SP2_flag = Cplex_SP2.solve();

    if (SP2_flag == 0) {
        // SP2 不可行 (理论上不应发生)
        // 不做处理
    } else {
        // ---------------------------------------------------------------------
        // 提取 SP2 的解
        // ---------------------------------------------------------------------
        this_node.SP2_obj_val = Cplex_SP2.getValue(Obj_SP2);
        this_node.SP2_solns_list.clear();

        int fsb_num = fsb_idx.size();

        // 构建完整的解向量 (包括不可行子件, 其值为 0)
        for (int i = 0; i < N_num; i++) {
            int fsb_flag = -1;

            // 检查子件 i 是否在可行列表中
            for (int k = 0; k < fsb_num; k++) {
                if (i == fsb_idx[k]) {
                    fsb_flag = 1;

                    // 从 CPLEX 获取解值
                    double soln_val = Cplex_SP2.getValue(De_Vars[i]);
                    if (soln_val == -0) soln_val = 0;
                    this_node.SP2_solns_list.push_back(soln_val);
                    break;
                }
            }

            // 不可行子件的解值设为 0
            if (fsb_flag != 1) {
                this_node.SP2_solns_list.push_back(0);
            }
        }
    }

    // =========================================================================
    // 释放 CPLEX 资源
    // =========================================================================
    Obj_SP2.removeAllProperties();
    Obj_SP2.end();
    De_Vars.clear();
    De_Vars.end();
    Model_SP2.removeAllProperties();
    Model_SP2.end();
    Env_SP2.removeAllProperties();
    Env_SP2.end();

    return final_return;
}
