// =============================================================================
// root_node_column_generation.cpp - 根节点列生成
// =============================================================================
//
// 功能: 在根节点执行完整的列生成求解过程
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// SolveRootCG - 根节点列生成主循环
// =============================================================================
void SolveRootCG(ProblemParams& params, ProblemData& data, BPNode& root_node) {

    cout << "[列生成] 根节点列生成开始\n";

    // =========================================================================
    // 初始化 CPLEX 环境和模型对象
    // =========================================================================
    IloEnv mp_env;
    IloModel mp_model(mp_env);
    IloObjective mp_obj = IloAdd(mp_model, IloMinimize(mp_env));
    IloNumVarArray mp_vars(mp_env);
    IloRangeArray mp_cons(mp_env);

    // 初始化迭代计数
    root_node.iter_ = 0;

    // =========================================================================
    // 求解初始主问题
    // =========================================================================
    bool mp_feasible = SolveRootInitMP(
        params, data, mp_env, mp_model, mp_obj, mp_cons, mp_vars, root_node);

    if (mp_feasible == 1) {
        // ---------------------------------------------------------------------
        // 初始主问题可行, 进入列生成循环
        // ---------------------------------------------------------------------
        while (1) {
            root_node.iter_++;

            // -----------------------------------------------------------------
            // 检查最大迭代次数
            // -----------------------------------------------------------------
            if (root_node.iter_ == kMaxCgIter) {
                cout << "[警告] 达到最大迭代次数 " << kMaxCgIter << ", 强制终止列生成\n";
                break;
            }

            // -----------------------------------------------------------------
            // 求解子问题
            // -----------------------------------------------------------------
            int has_improvement = SolveSP1(params, data, root_node);

            if (has_improvement == 0) {
                cout << "[列生成] 收敛 (共迭代 " << root_node.iter_ << " 次)\n";
                break;
            }

            if (has_improvement == 1) {
                UpdateMP(params, data, mp_env, mp_model, mp_obj, mp_cons, mp_vars, root_node);
            }
        }

        // ---------------------------------------------------------------------
        // 求解最终主问题
        // ---------------------------------------------------------------------
        SolveFinalMP(params, data, mp_env, mp_model, mp_obj, mp_cons, mp_vars, root_node);
    }

    // =========================================================================
    // 释放 CPLEX 资源
    // =========================================================================
    mp_obj.removeAllProperties();
    mp_obj.end();
    mp_vars.clear();
    mp_vars.end();
    mp_cons.clear();
    mp_cons.end();
    mp_model.removeAllProperties();
    mp_model.end();
    mp_env.removeAllProperties();
    mp_env.end();

    cout << "[列生成] 根节点列生成结束\n";
}
