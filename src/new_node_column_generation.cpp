// =============================================================================
// new_node_column_generation.cpp - 新节点列生成
// =============================================================================
// 功能: 对分支定界树中新生成的节点执行列生成求解
//
// 与根节点列生成的区别:
//   1. 继承父节点的列集合 (不从头开始)
//   2. 分支变量被固定为整数值 (floor 或 ceil)
//   3. 之前已分支的变量保持其固定值
// =============================================================================

#include "2DBP.h"

using namespace std;


// 新节点列生成主函数
void SolveNodeCG(
    ProblemParams& params,
    ProblemData& data,
    BPNode& cur_node,
    BPNode& parent_node) {
    // 第一步: 初始化 CPLEX 环境
    IloEnv mp_env;
    IloModel mp_model(mp_env);
    IloObjective mp_obj = IloAdd(mp_model, IloMinimize(mp_env));
    IloRangeArray mp_cons(mp_env);
    IloNumVarArray mp_vars(mp_env);

    cur_node.iter_ = 0;

    // 第二步: 求解初始主问题
    // 与根节点不同, 这里需要考虑当前分支变量的固定值和之前已分支变量的固定值
    bool mp_feasible = SolveNodeInitMP(
        params,
        data,
        mp_env,
        mp_model,
        mp_obj,
        mp_cons,
        mp_vars,
        cur_node,
        parent_node);

    // 第三步: 列生成主循环
    if (mp_feasible == 1) {
        // 初始主问题可行, 进入列生成循环
        while (1) {
            cur_node.iter_++;

            // 求解子问题
            int sp_flag = SolveSP1(params, data, cur_node);

            if (sp_flag == 0) {
                // 无改进列, 列生成收敛
                break;
            }
            if (sp_flag == 1) {
                // 找到改进列, 更新主问题
                UpdateMP(
                    params,
                    data,
                    mp_env,
                    mp_model,
                    mp_obj,
                    mp_cons,
                    mp_vars,
                    cur_node);
            }
        }

        // 求解最终主问题, 获取节点下界
        SolveFinalMP(
            params,
            data,
            mp_env,
            mp_model,
            mp_obj,
            mp_cons,
            mp_vars,
            cur_node);
    }
    // 如果 mp_feasible = 0, 初始主问题不可行, 节点已在函数内标记为剪枝

    // 第四步: 释放 CPLEX 资源
    mp_vars.clear();
    mp_vars.end();
    mp_cons.clear();
    mp_cons.end();
    mp_obj.removeAllProperties();
    mp_obj.end();
    mp_model.removeAllProperties();
    mp_model.end();
    mp_env.removeAllProperties();
    mp_env.end();
}
