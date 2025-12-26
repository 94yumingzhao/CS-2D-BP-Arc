// =============================================================================
// branch_and_price.cpp - 分支定价主循环
// =============================================================================
//
// 功能: 实现分支定价算法的主控制循环
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// RunBranchAndPrice - 分支定价搜索树主循环
// =============================================================================
int RunBranchAndPrice(ProblemParams& params, ProblemData& data) {

    params.num_nodes_ = 1;

    // =========================================================================
    // 分支定价主循环
    // =========================================================================
    while (1) {

        // ---------------------------------------------------------------------
        // 状态 1: 继续分支 (need_search_ = 0)
        // ---------------------------------------------------------------------
        if (params.need_search_ == 0) {

            // -----------------------------------------------------------------
            // 选择待分支节点
            // -----------------------------------------------------------------
            BPNode parent_node;
            int has_parent = SelectBranchNode(params, data, parent_node);

            if (has_parent == 0) {
                cout << "[分支定价] 求解完成, 无可分支节点\n";
                cout << "[分支定价] 最优目标值 = " << fixed << setprecision(4)
                     << params.best_obj_ << "\n";
                cout.unsetf(ios::fixed);
                break;
            }

            if (has_parent == 1) {
                BPNode left_node;
                BPNode right_node;

                // =============================================================
                // 步骤1: 处理左子节点 (向下取整分支)
                // =============================================================
                params.branch_state_ = 1;
                params.num_nodes_++;

                CreateChildNode(params, data, left_node, parent_node);

                SolveNodeCG(params, data, left_node, parent_node);

                int left_need_search = ProcessNode(params, data, left_node);

                data.nodes_.push_back(left_node);

                // =============================================================
                // 步骤2: 处理右子节点 (向上取整分支)
                // =============================================================
                params.branch_state_ = 2;
                params.num_nodes_++;

                CreateChildNode(params, data, right_node, parent_node);

                SolveNodeCG(params, data, right_node, parent_node);

                int right_need_search = ProcessNode(params, data, right_node);

                data.nodes_.push_back(right_node);

                params.is_at_root_ = 0;

                // =============================================================
                // 步骤3: 选择下一个探索方向
                // =============================================================
                double parent_val = parent_node.branch_var_val_;

                if (parent_val <= 1) {
                    // ---------------------------------------------------------
                    // 分支变量值 <= 1: 优先探索右子节点
                    // ---------------------------------------------------------
                    params.need_search_ = right_need_search;

                    if (params.need_search_ != 1) {
                        params.fathom_dir_ = 2;
                        cout << "[分支定价] 分支变量值 = " << fixed << setprecision(4)
                             << parent_val << " <= 1, 深入右子节点_"
                             << right_node.id_ << "\n";
                        cout.unsetf(ios::fixed);
                    }
                }

                if (parent_val > 1) {
                    // ---------------------------------------------------------
                    // 分支变量值 > 1: 选择下界更优的子节点
                    // ---------------------------------------------------------
                    if (left_node.lower_bound_ < right_node.lower_bound_) {
                        params.need_search_ = left_need_search;

                        if (params.need_search_ != 1) {
                            params.fathom_dir_ = 1;
                            cout << "[分支定价] 左子节点_" << left_node.id_
                                 << " (LB=" << fixed << setprecision(4) << left_node.lower_bound_
                                 << ") < 右子节点_" << right_node.id_
                                 << " (LB=" << right_node.lower_bound_ << ")\n";
                            cout << "[分支定价] 深入左子节点\n";
                            cout.unsetf(ios::fixed);
                        }
                    }

                    if (left_node.lower_bound_ >= right_node.lower_bound_) {
                        params.need_search_ = right_need_search;

                        if (params.need_search_ != 1) {
                            params.fathom_dir_ = 2;
                            cout << "[分支定价] 左子节点_" << left_node.id_
                                 << " (LB=" << fixed << setprecision(4) << left_node.lower_bound_
                                 << ") >= 右子节点_" << right_node.id_
                                 << " (LB=" << right_node.lower_bound_ << ")\n";
                            cout << "[分支定价] 深入右子节点\n";
                            cout.unsetf(ios::fixed);
                        }
                    }
                }

                params.branch_state_ = 1;
            }
        }

        // ---------------------------------------------------------------------
        // 状态 2: 搜索其他节点 (need_search_ = 1)
        // ---------------------------------------------------------------------
        if (params.need_search_ == 1) {
            params.branch_state_ = 3;
            params.fathom_dir_ = -1;
            params.need_search_ = 0;

            cout << "[分支定价] 当前节点解均为整数\n";
            cout << "[分支定价] 当前最优下界 = " << fixed << setprecision(4)
                 << params.best_obj_ << "\n";
            cout.unsetf(ios::fixed);
        }

        // ---------------------------------------------------------------------
        // 安全检查: 防止无限循环
        // ---------------------------------------------------------------------
        if (params.num_nodes_ > kMaxBpNodes) {
            cout << "[警告] 达到最大节点数 " << kMaxBpNodes << ", 强制终止\n";
            break;
        }
    }

    return 0;
}
