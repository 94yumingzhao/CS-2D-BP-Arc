// new_node.cpp - 非根节点(分支节点)列生成和主问题求解
//
// 本文件实现Branch and Price树中非根节点的列生成过程
// 与根节点的主要区别:
// 1. 使用指针BPNode*而非引用, 便于树结构管理
// 2. 继承父节点的列池和分支约束
// 3. 支持变量分支约束 (branched_var_ids_, branched_bounds_)
// 4. 支持Arc分支约束 (sp1_*_arcs_, sp2_*_arcs_)
//
// 分支节点的处理流程:
// 1. 从父节点继承列池 (y_columns_, x_columns_)
// 2. 应用分支约束到变量上界
// 3. 执行列生成直至收敛
// 4. 检查是否需要剪枝或进一步分支

#include "2DBP.h"

using namespace std;

// 非根节点列生成主循环
// 功能: 对分支节点执行列生成, 求解其LP松弛问题
// 与根节点的区别:
//   - 继承父节点的列池和对偶价格
//   - 变量上界受分支约束限制
//   - 子问题中应用Arc分支约束 (在SolveNodeSP1ArcFlow等中处理)
// 返回值: 0=正常完成, -1=节点不可行(被剪枝)
int SolveNodeCG(ProblemParams& params, ProblemData& data, BPNode* node) {
    LOG_FMT("[CG] 节点%d 列生成开始\n", node->id_);

    // 继承全局SP方法设置
    // 注意: 非根节点通常使用Arc Flow以支持Arc分支约束
    node->sp1_method_ = params.sp1_method_;
    node->sp2_method_ = params.sp2_method_;

    // 初始化CPLEX环境
    IloEnv env;
    IloModel model(env);
    IloObjective obj = IloAdd(model, IloMinimize(env));
    IloNumVarArray vars(env);
    IloRangeArray cons(env);

    node->iter_ = 0;  // 迭代计数器

    // 构建并求解初始主问题
    // 初始列继承自父节点, 变量上界受分支约束限制
    bool feasible = SolveNodeInitMP(params, data, env, model, obj, cons, vars, node);

    if (!feasible) {
        // 节点不可行 (分支约束导致无法满足需求)
        // 标记剪枝并返回
        node->prune_flag_ = 1;
        env.end();
        LOG_FMT("[CG] 节点%d 不可行, 剪枝\n", node->id_);
        return -1;
    }

    // 列生成主循环
    while (true) {
        node->iter_++;

        // 检查最大迭代次数限制
        if (node->iter_ >= kMaxCgIter) {
            LOG_FMT("[CG] 达到最大迭代次数 %d, 终止\n", kMaxCgIter);
            break;
        }

        // 步骤1: 求解SP1子问题 (宽度方向背包)
        // Arc分支约束在子问题求解函数中应用
        bool sp1_converged = SolveNodeSP1(params, data, node);

        if (sp1_converged) {
            // SP1收敛, 检查所有SP2子问题
            bool all_sp2_converged = true;

            for (int j = 0; j < params.num_strip_types_; j++) {
                bool sp2_converged = SolveNodeSP2(params, data, node, j);

                if (!sp2_converged) {
                    all_sp2_converged = false;
                    // 添加新X列到主问题
                    SolveNodeUpdateMP(params, data, env, model, obj, cons, vars, node);
                }
            }

            // 检查是否完全收敛
            if (all_sp2_converged) {
                LOG_FMT("[CG] 列生成收敛, 迭代%d次\n", node->iter_);
                break;
            }
        } else {
            // SP1找到改进列, 添加新Y列
            SolveNodeUpdateMP(params, data, env, model, obj, cons, vars, node);
        }
    }

    // 求解最终主问题, 提取完整解
    SolveNodeFinalMP(params, data, env, model, obj, cons, vars, node);

    // 释放CPLEX资源
    obj.end();
    vars.end();
    cons.end();
    model.end();
    env.end();

    LOG_FMT("[CG] 节点%d 列生成结束, 下界=%.4f\n", node->id_, node->lower_bound_);
    return 0;
}

// 构建并求解非根节点的初始主问题
// 功能: 基于继承的列池构建主问题, 应用变量分支约束
// 变量分支约束:
//   - branched_var_ids_: 受分支约束影响的变量索引
//   - branched_bounds_: 对应变量的上界 (来自左分支的floor(v))
// 注意: 变量索引 = [0, num_y_cols) 为Y变量, [num_y_cols, ...) 为X变量
// 返回值: true=可行, false=不可行
bool SolveNodeInitMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode* node) {

    int num_y_cols = static_cast<int>(node->y_columns_.size());
    int num_x_cols = static_cast<int>(node->x_columns_.size());
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;
    int num_rows = num_strip_types + num_item_types;

    LOG_FMT("[MP-0] 节点%d 构建初始主问题 (Y=%d, X=%d)\n",
        node->id_, num_y_cols, num_x_cols);

    // 构建约束的上下界 (与根节点相同)
    IloNumArray con_min(env);
    IloNumArray con_max(env);

    // 条带平衡约束: >= 0
    for (int j = 0; j < num_strip_types; j++) {
        con_min.add(0);
        con_max.add(IloInfinity);
    }

    // 需求约束: >= d_i
    for (int i = 0; i < num_item_types; i++) {
        con_min.add(data.item_types_[i].demand_);
        con_max.add(IloInfinity);
    }

    cons = IloRangeArray(env, con_min, con_max);
    model.add(cons);
    con_min.end();
    con_max.end();

    // 添加Y变量 (母板模式)
    // 需要检查变量分支约束, 可能限制变量上界
    for (int col = 0; col < num_y_cols; col++) {
        IloNumColumn cplex_col = obj(1.0);  // 目标系数=1

        // 条带产出系数
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j](node->y_columns_[col].pattern_[j]);
        }
        // 需求约束系数 (Y列不直接满足需求)
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](0);
        }

        // 检查该变量是否受分支约束限制
        // 如果是左分支, 变量上界被设为floor(分数值)
        double var_ub = IloInfinity;  // 默认无上界
        for (int k = 0; k < (int)node->branched_var_ids_.size(); k++) {
            if (node->branched_var_ids_[k] == col) {
                var_ub = node->branched_bounds_[k];  // 应用分支约束
                break;
            }
        }

        string var_name = "Y_" + to_string(col + 1);
        IloNumVar var(cplex_col, 0, var_ub, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();
    }

    // 添加X变量 (条带模式)
    for (int col = 0; col < num_x_cols; col++) {
        IloNumColumn cplex_col = obj(0.0);  // 目标系数=0
        int strip_type = node->x_columns_[col].strip_type_id_;

        // 条带消耗系数
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j]((j == strip_type) ? -1 : 0);
        }
        // 需求满足系数
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](node->x_columns_[col].pattern_[i]);
        }

        // 检查变量分支约束
        // X变量的索引偏移量 = num_y_cols
        int var_idx = num_y_cols + col;
        double var_ub = IloInfinity;
        for (int k = 0; k < (int)node->branched_var_ids_.size(); k++) {
            if (node->branched_var_ids_[k] == var_idx) {
                var_ub = node->branched_bounds_[k];
                break;
            }
        }

        string var_name = "X_" + to_string(col + 1);
        IloNumVar var(cplex_col, 0, var_ub, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();
    }

    // 求解初始主问题
    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    if (!feasible) {
        // 不可行: 分支约束过紧, 无法满足需求
        LOG("[MP] 初始主问题不可行");
        cplex.end();
        return false;
    }

    double obj_val = cplex.getValue(obj);
    LOG_FMT("[MP] 目标值: %.4f\n", obj_val);

    // 提取对偶价格, 用于子问题求解
    node->duals_.clear();
    for (int row = 0; row < num_rows; row++) {
        double dual = cplex.getDual(cons[row]);
        if (dual == -0.0) dual = 0.0;
        node->duals_.push_back(dual);
    }

    cplex.end();
    return true;
}

// 更新非根节点主问题: 添加新生成的列
// 功能: 将子问题找到的改进列添加到主问题
// 注意: 新列不受变量分支约束限制 (只有继承的老列受限)
// 返回值: true=更新后可行, false=不可行
bool SolveNodeUpdateMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode* node) {

    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;

    // 添加新Y列 (如果SP1找到改进列)
    if (!node->new_y_col_.pattern_.empty()) {
        IloNumColumn cplex_col = obj(1.0);

        // 条带产出系数
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j](node->new_y_col_.pattern_[j]);
        }
        // 需求约束系数
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](0);
        }

        int col_id = static_cast<int>(node->y_columns_.size()) + 1;
        string var_name = "Y_" + to_string(col_id);
        // 新列上界为无穷 (不受已有分支约束限制)
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();

        // 保存新列到列池
        YColumn y_col;
        y_col.pattern_ = node->new_y_col_.pattern_;
        node->y_columns_.push_back(y_col);
        node->new_y_col_.pattern_.clear();
    }

    // 添加新X列 (如果SP2找到改进列)
    if (!node->new_x_col_.pattern_.empty()) {
        int strip_type = node->new_strip_type_;
        IloNumColumn cplex_col = obj(0.0);

        // 条带消耗系数
        for (int j = 0; j < num_strip_types; j++) {
            cplex_col += cons[j]((j == strip_type) ? -1 : 0);
        }
        // 需求满足系数
        for (int i = 0; i < num_item_types; i++) {
            cplex_col += cons[num_strip_types + i](node->new_x_col_.pattern_[i]);
        }

        int col_id = static_cast<int>(node->x_columns_.size()) + 1;
        string var_name = "X_" + to_string(col_id);
        IloNumVar var(cplex_col, 0, IloInfinity, ILOFLOAT, var_name.c_str());
        vars.add(var);
        cplex_col.end();

        // 保存新列到列池
        XColumn x_col;
        x_col.strip_type_id_ = strip_type;
        x_col.pattern_ = node->new_x_col_.pattern_;
        node->x_columns_.push_back(x_col);
        node->new_x_col_.pattern_.clear();
    }

    // 求解更新后的主问题
    LOG_FMT("[MP-%d] 更新并求解主问题\n", node->iter_);
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

    // 提取新的对偶价格
    node->duals_.clear();
    int num_rows = num_strip_types + num_item_types;
    for (int row = 0; row < num_rows; row++) {
        double dual = cplex.getDual(cons[row]);
        if (dual == -0.0) dual = 0.0;
        node->duals_.push_back(dual);
    }

    cplex.end();
    return true;
}

// 求解非根节点最终主问题并提取解
// 功能: 列生成收敛后, 提取完整LP解
// 输出:
//   - node->lower_bound_: LP最优目标值 (该节点的下界)
//   - node->solution_: 完整解 (用于整数性检查和分支)
//   - node->prune_flag_: 若不可行则设为1
// 返回值: true=可行, false=不可行
bool SolveNodeFinalMP(ProblemParams& params, ProblemData& data,
    IloEnv& env, IloModel& model, IloObjective& obj,
    IloRangeArray& cons, IloNumVarArray& vars, BPNode* node) {

    LOG_FMT("[MP-Final] 节点%d 求解最终主问题\n", node->id_);

    IloCplex cplex(env);
    cplex.extract(model);
    cplex.setOut(env.getNullStream());
    bool feasible = cplex.solve();

    if (!feasible) {
        LOG("[MP] 最终主问题不可行");
        node->prune_flag_ = 1;  // 标记节点被剪枝
        cplex.end();
        return false;
    }

    // 提取目标值作为该节点的下界
    double obj_val = cplex.getValue(obj);
    node->lower_bound_ = obj_val;
    node->solution_.obj_val_ = obj_val;

    LOG_FMT("[MP] 最终目标值: %.4f\n", obj_val);

    // 提取Y列的解值
    node->solution_.y_columns_.clear();
    for (int col = 0; col < (int)node->y_columns_.size(); col++) {
        double val = cplex.getValue(vars[col]);
        if (fabs(val) < kZeroTolerance) val = 0;  // 处理数值误差

        YColumn y_col = node->y_columns_[col];
        y_col.value_ = val;
        node->solution_.y_columns_.push_back(y_col);
    }

    // 提取X列的解值
    node->solution_.x_columns_.clear();
    int y_count = static_cast<int>(node->y_columns_.size());
    for (int col = 0; col < static_cast<int>(node->x_columns_.size()); col++) {
        double val = cplex.getValue(vars[y_count + col]);
        if (fabs(val) < kZeroTolerance) val = 0;

        XColumn x_col = node->x_columns_[col];
        x_col.value_ = val;
        node->solution_.x_columns_.push_back(x_col);
    }

    cplex.end();
    return true;
}
