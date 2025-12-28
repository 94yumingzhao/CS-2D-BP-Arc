// root_node.cpp - 根节点列生成和主问题求解
//
// 本文件实现根节点的列生成(Column Generation)过程, 包括:
// - 主问题(Master Problem)的构建、更新和求解
// - 列生成主循环的迭代控制
// - 最终LP解的提取和存储
//
// 二维下料问题的主问题模型:
// 决策变量:
//   - Y_k: 第k种母板切割模式的使用次数
//   - X_p: 第p种条带切割模式的使用次数
// 目标函数:
//   min sum_{k} Y_k  (最小化母板使用数量)
// 约束条件:
//   (1) 条带平衡约束: sum_{k} C_{jk}*Y_k - sum_{p} A_{jp}*X_p >= 0
//       其中 C_{jk} 表示母板模式k产出j型条带的数量
//       A_{jp} = 1 当且仅当条带模式p属于j型条带
//   (2) 需求约束: sum_{p} B_{ip}*X_p >= d_i
//       其中 B_{ip} 表示条带模式p产出i型子板的数量
//
// 列生成收敛判断:
//   - SP1找不到 reduced_cost > 1 的Y列
//   - 所有SP2都找不到 reduced_cost > v_j 的X列 (v_j为条带j的对偶价格)

#include "2DBP.h"

using namespace std;

// 根节点列生成主循环
// 功能: 通过交替求解主问题和子问题, 逐步生成列直至收敛
// 流程:
//   1. 初始化主问题 (使用启发式生成的初始列)
//   2. 迭代: 求解SP1 -> (若收敛) 求解所有SP2 -> 更新主问题
//   3. 直到SP1和所有SP2都收敛, 或达到最大迭代次数
// 输出: 列生成收敛后的LP最优解存储在root_node中
void SolveRootCG(ProblemParams& params, ProblemData& data, BPNode& root_node) {
    LOG("[CG] 根节点列生成开始");

    // 继承全局SP方法设置
    // sp1_method_/sp2_method_: 子问题求解方法 (Knapsack/ArcFlow/DP)
    root_node.sp1_method_ = params.sp1_method_;
    root_node.sp2_method_ = params.sp2_method_;

    // 初始化CPLEX环境
    // model: 主问题模型, vars: 所有列变量, cons: 所有约束
    IloEnv env;
    IloModel model(env);
    IloObjective obj = IloAdd(model, IloMinimize(env));  // 最小化目标
    IloNumVarArray vars(env);
    IloRangeArray cons(env);

    root_node.iter_ = 0;  // 迭代计数器

    // 构建并求解初始主问题
    // 初始列来自启发式解 (heuristic.cpp中生成)
    bool feasible = SolveRootInitMP(params, data, env, model, obj, cons, vars, root_node);

    if (feasible) {
        // 列生成主循环
        while (true) {
            root_node.iter_++;

            // 检查最大迭代次数限制
            if (root_node.iter_ >= kMaxCgIter) {
                LOG_FMT("[CG] 达到最大迭代次数 %d, 终止\n", kMaxCgIter);
                break;
            }

            // 步骤1: 求解SP1子问题 (宽度方向背包)
            // 寻找能改进目标的新Y列 (母板切割模式)
            bool sp1_converged = SolveRootSP1(params, data, root_node);

            if (sp1_converged) {
                // SP1收敛: 没有新的Y列能改进目标
                // 步骤2: 对每种条带类型求解SP2子问题 (长度方向背包)
                bool all_sp2_converged = true;

                for (int j = 0; j < params.num_strip_types_; j++) {
                    // 为条带类型j寻找能改进目标的新X列
                    bool sp2_converged = SolveRootSP2(params, data, root_node, j);

                    if (!sp2_converged) {
                        all_sp2_converged = false;
                        // 找到改进列, 立即添加到主问题
                        SolveRootUpdateMP(params, data, env, model, obj, cons, vars, root_node);
                    }
                }

                // 检查是否完全收敛
                if (all_sp2_converged) {
                    LOG_FMT("[CG] 列生成收敛, 迭代%d次\n", root_node.iter_);
                    break;  // SP1和所有SP2都收敛, 退出循环
                }
            } else {
                // SP1找到改进列, 添加新Y列并继续迭代
                SolveRootUpdateMP(params, data, env, model, obj, cons, vars, root_node);
            }
        }

        // 求解最终主问题, 提取完整解
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
// 功能: 基于初始列构建并求解主问题, 获取初始对偶价格
// 主问题结构:
//   min sum(Y_k)                              (目标: 最小化母板数)
//   s.t. sum(C_jk*Y_k) - sum(A_jp*X_p) >= 0   (条带平衡: 产出>=消耗)
//        sum(B_ip*X_p) >= d_i                 (需求满足: 切出>=需求)
// 对偶价格:
//   - duals_[0..num_strip_types-1]: 条带平衡约束对偶 v_j
//   - duals_[num_strip_types..]: 需求约束对偶 pi_i
// 返回值: true=可行, false=不可行
bool SolveRootInitMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& root_node) {

    int num_y_cols = static_cast<int>(root_node.y_columns_.size());
    int num_x_cols = static_cast<int>(root_node.x_columns_.size());
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_rows = num_strip_types + num_item_types;  // 约束总数

    LOG_FMT("[MP-0] 构建初始主问题 (Y=%d, X=%d)\n", num_y_cols, num_x_cols);

    // 构建约束的上下界
    IloNumArray con_min(env);  // 约束下界
    IloNumArray con_max(env);  // 约束上界

    // 条带平衡约束: sum(C_jk*Y_k) - sum(A_jp*X_p) >= 0
    // 下界=0, 上界=无穷 (即 >= 0 的形式)
    for (int j = 0; j < num_strip_types; j++) {
        con_min.add(0);
        con_max.add(IloInfinity);
    }

    // 需求约束: sum(B_ip*X_p) >= d_i
    // 下界=d_i (需求量), 上界=无穷
    for (int i = 0; i < num_item_types; i++) {
        con_min.add(data.item_types_[i].demand_);
        con_max.add(IloInfinity);
    }

    // 创建约束数组并添加到模型
    cons = IloRangeArray(env, con_min, con_max);
    model.add(cons);
    con_min.end();
    con_max.end();

    // 添加Y变量 (母板模式)
    // 目标系数 = 1 (每使用一块母板, 目标值+1)
    for (int col = 0; col < num_y_cols; col++) {
        IloNumColumn cplex_col = obj(1.0);  // 目标函数系数

        // 条带产出系数: C_jk = pattern[j]
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j](root_node.y_columns_[col].pattern_[j]);
        }
        // 需求约束系数: 0 (Y列不直接满足需求)
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](0);
        }

        string var_name = "Y_" + to_string(col + 1);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();
    }

    // 添加X变量 (条带模式)
    // 目标系数 = 0 (条带模式不计入目标)
    for (int col = 0; col < num_x_cols; col++) {
        IloNumColumn cplex_col = obj(0.0);  // 目标函数系数
        int strip_type = root_node.x_columns_[col].strip_type_id_;

        // 条带消耗系数: -1在对应条带类型位置
        // 这表示X列消耗一个对应类型的条带
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j]((j == strip_type) ? -1 : 0);
        }
        // 需求约束系数: B_ip = pattern[i]
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](root_node.x_columns_[col].pattern_[i]);
        }

        string var_name = "X_" + to_string(col + 1);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();
    }

    // 求解初始主问题
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());  // 关闭CPLEX输出

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

    // 提取对偶价格, 用于子问题求解
    // 对偶价格反映约束"放松一单位"对目标的改善
    root_node.duals_.clear();
    for (int row = 0; row < num_rows; row++) {
        double dual = cplex.getDual(cons[row]);
        if (dual == -0.0) dual = 0.0;  // 处理负零
        root_node.duals_.push_back(dual);
    }

    cplex.end();
    return true;
}

// 更新主问题: 添加由子问题生成的新列
// 功能: 将SP1/SP2生成的新列添加到主问题, 并重新求解获取新对偶价格
// 新Y列: 来自SP1, 表示新的母板切割模式
// 新X列: 来自SP2, 表示新的条带切割模式
// 返回值: true=更新后可行, false=不可行
bool SolveRootUpdateMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode& node) {

    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;

    // 添加新Y列 (如果SP1找到改进列)
    if (!node.new_y_col_.pattern_.empty()) {
        IloNumColumn cplex_col = obj(1.0);  // 目标系数=1

        // 添加条带产出系数
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j](node.new_y_col_.pattern_[j]);
        }
        // 需求约束系数为0
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](0);
        }

        int col_id = static_cast<int>(node.y_columns_.size()) + 1;
        string var_name = "Y_" + to_string(col_id);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();

        // 保存新列到列池
        YColumn y_col;
        y_col.pattern_ = node.new_y_col_.pattern_;
        y_col.arc_set_ = node.new_y_col_.arc_set_;  // Arc集合 (用于Arc分支)
        node.y_columns_.push_back(y_col);

        // 清空临时存储
        node.new_y_col_.pattern_.clear();
        node.new_y_col_.arc_set_.clear();
    }

    // 添加新X列 (如果SP2找到改进列)
    if (!node.new_x_col_.pattern_.empty()) {
        int strip_type = node.new_strip_type_;  // 该X列属于哪种条带类型
        IloNumColumn cplex_col = obj(0.0);  // 目标系数=0

        // 添加条带消耗系数: 在对应条带类型位置为-1
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j]((j == strip_type) ? -1 : 0);
        }
        // 添加需求满足系数
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](node.new_x_col_.pattern_[i]);
        }

        int col_id = static_cast<int>(node.x_columns_.size()) + 1;
        string var_name = "X_" + to_string(col_id);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();

        // 保存新列到列池
        XColumn x_col;
        x_col.strip_type_id_ = strip_type;
        x_col.pattern_ = node.new_x_col_.pattern_;
        x_col.arc_set_ = node.new_x_col_.arc_set_;  // Arc集合 (用于Arc分支)
        node.x_columns_.push_back(x_col);

        // 清空临时存储
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

    // 提取新的对偶价格, 用于下一轮子问题求解
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

// 求解最终主问题并提取完整解
// 功能: 列生成收敛后, 求解最终LP并提取所有列的解值
// 输出:
//   - node.lower_bound_: LP最优目标值 (作为该节点的下界)
//   - node.solution_: 完整的LP解 (包括每列的取值)
// 返回值: true=可行, false=不可行
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

    // 提取目标值作为该节点的下界
    double obj_val = cplex.getValue(obj);
    node.lower_bound_ = obj_val;
    node.solution_.obj_val_ = obj_val;

    LOG_FMT("[MP] 最终目标值: %.4f\n", obj_val);

    // 提取Y列的解值
    // Y列对应母板切割模式, 其值表示该模式的使用次数
    node.solution_.y_columns_.clear();
    for (int col = 0; col < (int)node.y_columns_.size(); col++) {
        double val = cplex.getValue(vars[col]);
        if (fabs(val) < kZeroTolerance) val = 0;  // 处理数值误差

        YColumn y_col = node.y_columns_[col];
        y_col.value_ = val;  // 记录解值
        node.solution_.y_columns_.push_back(y_col);

        // 输出非零解
        if (val > kZeroTolerance) {
            LOG_FMT("  Y_%d = %.4f\n", col + 1, val);
        }
    }

    // 提取X列的解值
    // X列对应条带切割模式, 其值表示该模式的使用次数
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
