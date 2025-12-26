// =============================================================================
// branching.cpp - 分支决策模块
// =============================================================================
//
// 功能: 实现节点处理和分支变量选择
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// ProcessNode - 完成节点处理
// =============================================================================
int ProcessNode(ProblemParams& params, ProblemData& data, BPNode& node) {

    // -------------------------------------------------------------------------
    // 状态变量
    // -------------------------------------------------------------------------
    int is_integer = -1;
    int need_search = -1;

    // =========================================================================
    // 检查整数性并选择分支变量
    // =========================================================================
    is_integer = SelectBranchVar(params, data, node);

    // =========================================================================
    // 根据整数性决定下一步操作
    // =========================================================================

    if (node.prune_flag_ == 1) {
        // ---------------------------------------------------------------------
        // 节点已被剪枝
        // ---------------------------------------------------------------------
        need_search = 1;
    }
    else {
        if (is_integer == 0) {
            // -----------------------------------------------------------------
            // 存在非整数解, 记录分支变量信息
            // -----------------------------------------------------------------
            int var_id = node.branch_var_id_;
            node.branched_var_ids_.push_back(var_id);

            double var_val = node.branch_var_val_;
            node.branched_vals_.push_back(var_val);

            need_search = 0;
        }

        if (is_integer == 1) {
            // -----------------------------------------------------------------
            // 所有非零解均为整数
            // -----------------------------------------------------------------

            if (node.id_ == 1) {
                // --------- 根节点即为整数解 ---------
                params.best_obj_ = node.lower_bound_;
                cout << "[分支] 根节点解全为整数, 最优下界 = "
                     << fixed << setprecision(4) << params.best_obj_ << "\n";
                cout.unsetf(ios::fixed);
            }

            if (node.id_ > 1) {
                // --------- 非根节点 ---------
                if (params.best_obj_ == -1) {
                    params.best_obj_ = node.lower_bound_;
                    cout << "[分支] 找到首个整数解, 最优下界 = "
                         << fixed << setprecision(4) << params.best_obj_ << "\n";
                    cout.unsetf(ios::fixed);
                }
                else {
                    if (node.lower_bound_ < params.best_obj_) {
                        params.best_obj_ = node.lower_bound_;
                        cout << "[分支] 找到更优整数解, 最优下界 = "
                             << fixed << setprecision(4) << params.best_obj_ << "\n";
                        cout.unsetf(ios::fixed);
                    }
                    if (node.lower_bound_ >= params.best_obj_) {
                        node.prune_flag_ = 1;
                        cout << "[分支] 节点_" << node.id_
                             << " 下界不优 (LB=" << fixed << setprecision(4) << node.lower_bound_
                             << " >= " << params.best_obj_ << "), 需剪枝\n";
                        cout.unsetf(ios::fixed);
                    }
                }
            }

            need_search = 1;
        }
    }

    // =========================================================================
    // 清理临时列表
    // =========================================================================
    data.used_stocks_.clear();
    data.assigned_items_.clear();
    data.strips_.clear();

    return need_search;
}


// =============================================================================
// SelectBranchVar - 选择分支变量
// =============================================================================
int SelectBranchVar(ProblemParams& params, ProblemData& data, BPNode& node) {

    int is_integer = 1;
    double sol_val;

    // =========================================================================
    // 遍历所有变量寻找非整数解
    // =========================================================================
    int num_sols = node.solution_.size();

    for (int col = 0; col < num_sols; col++) {
        sol_val = node.solution_[col];

        if (sol_val > 0) {
            int int_val = int(sol_val);

            if (int_val != sol_val) {
                // ---------------------------------------------------------
                // 找到非整数变量
                // ---------------------------------------------------------
                cout << "[分支] 节点_" << node.id_
                     << " 变量_" << col + 1
                     << " = " << fixed << setprecision(4) << sol_val
                     << " 非整数\n";
                cout.unsetf(ios::fixed);

                node.branch_var_id_ = col;
                node.branch_var_val_ = sol_val;
                node.branch_floor_ = floor(sol_val);
                node.branch_ceil_ = ceil(sol_val);

                is_integer = 0;
                break;
            }
        }
    }

    return is_integer;
}
