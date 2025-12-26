// =============================================================================
// branch_and_price.cpp - 分支定价主循环
// =============================================================================
//
// 功能: 实现分支定价算法的主控制循环
//
// -----------------------------------------------------------------------------
// 分支定价算法概述
// -----------------------------------------------------------------------------
//
// 分支定价 (Branch and Price) = 分支定界 + 列生成
//   - 在每个节点使用列生成求解 LP 松弛
//   - 若 LP 解非整数, 则进行分支
//   - 构建搜索树, 通过剪枝加速求解
//
// 算法流程:
//
//   +------------------------+
//   | 选择待分支节点         |
//   | ChooseNodeToBranch     |
//   +-----------+------------+
//               |
//         +-----+-----+
//         |           |
//      有节点       无节点
//         |           |
//         v           v
//   +-----+-----+   +---+
//   |生成子节点 |   |终止|
//   +-----------+   +---+
//         |
//   +-----v-----+
//   |列生成求解 |
//   +-----------+
//         |
//   +-----v-----+
//   |检查整数性 |
//   +-----------+
//         |
//   +-----v-----+
//   |更新最优解 |
//   |剪枝/继续  |
//   +-----------+
//         |
//         +-----> 返回选择节点
//
// 搜索策略:
//   - 优先搜索下界较小的节点 (最优优先)
//   - 当分支变量值 <= 1 时, 优先探索右子节点
//   - 当分支变量值 > 1 时, 选择下界更优的子节点
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// BranchAndPriceTree - 分支定价搜索树主循环
// =============================================================================
//
// 功能: 管理分支定界树的搜索过程, 直至找到最优整数解或穷尽搜索
//
// 主循环逻辑:
//   1. search_flag = 0: 继续分支当前父节点
//      - 选择待分支节点
//      - 生成左子节点 (向下取整)
//      - 生成右子节点 (向上取整)
//      - 对子节点执行列生成
//      - 选择下一个探索方向
//
//   2. search_flag = 1: 当前节点为整数解
//      - 更新最优解
//      - 搜索其他未探索节点
//
// 参数:
//   Values - 全局参数 (更新 optimal_LB, node_num 等)
//   Lists  - 全局列表 (更新 all_nodes_list)
//
// 返回值:
//   0 = 正常结束
//
// =============================================================================
int BranchAndPriceTree(All_Values& Values, All_Lists& Lists) {

    Values.node_num = 1;  // 根节点已生成

    // =========================================================================
    // 分支定价主循环
    // =========================================================================
    while (1) {

        // ---------------------------------------------------------------------
        // 状态 1: 继续分支 (search_flag = 0)
        // ---------------------------------------------------------------------
        if (Values.search_flag == 0) {

            // -----------------------------------------------------------------
            // 选择待分支节点
            // -----------------------------------------------------------------
            Node parent_node;
            int parent_branch_flag = ChooseNodeToBranch(Values, Lists, parent_node);

            if (parent_branch_flag == 0) {
                // 无可分支节点, 算法终止
                cout << "[分支定价] 求解完成, 无可分支节点\n";
                cout << "[分支定价] 最优目标值 = " << fixed << setprecision(4)
                     << Values.optimal_LB << "\n";
                cout.unsetf(ios::fixed);
                break;
            }

            if (parent_branch_flag == 1) {
                // 找到待分支节点, 生成左右子节点

                Node new_left_node;
                Node new_right_node;

                // =============================================================
                // 步骤1: 处理左子节点 (向下取整分支)
                // =============================================================
                // 左子节点: 分支变量 <= floor(原值)
                Values.branch_status = 1;  // 标记为左分支
                Values.node_num++;

                // 生成左子节点
                GenerateNewNode(Values, Lists, new_left_node, parent_node);

                // 对左子节点执行列生成
                NewNodeColumnGeneration(Values, Lists, new_left_node, parent_node);

                // 完成节点处理, 检查整数性
                int left_search_flag = FinishNode(Values, Lists, new_left_node);

                // 保存左子节点到列表
                Lists.all_nodes_list.push_back(new_left_node);

                // =============================================================
                // 步骤2: 处理右子节点 (向上取整分支)
                // =============================================================
                // 右子节点: 分支变量 >= ceil(原值)
                Values.branch_status = 2;  // 标记为右分支
                Values.node_num++;

                // 生成右子节点
                GenerateNewNode(Values, Lists, new_right_node, parent_node);

                // 对右子节点执行列生成
                NewNodeColumnGeneration(Values, Lists, new_right_node, parent_node);

                // 完成节点处理, 检查整数性
                int right_search_flag = FinishNode(Values, Lists, new_right_node);

                // 保存右子节点到列表
                Lists.all_nodes_list.push_back(new_right_node);

                // 标记已离开根节点
                Values.root_flag = 0;

                // =============================================================
                // 步骤3: 选择下一个探索方向
                // =============================================================
                double parent_branch_val = parent_node.var_to_branch_soln;

                if (parent_branch_val <= 1) {
                    // ---------------------------------------------------------
                    // 分支变量值 <= 1: 优先探索右子节点
                    // ---------------------------------------------------------
                    // 当变量值很小时, floor(val) = 0, 右分支更有可能保持可行
                    Values.search_flag = right_search_flag;

                    if (Values.search_flag != 1) {
                        Values.fathom_flag = 2;  // 深入右子节点
                        cout << "[分支定价] 分支变量值 = " << fixed << setprecision(4)
                             << parent_branch_val << " <= 1, 深入右子节点_"
                             << new_right_node.index << "\n";
                        cout.unsetf(ios::fixed);
                    }
                }

                if (parent_branch_val > 1) {
                    // ---------------------------------------------------------
                    // 分支变量值 > 1: 选择下界更优的子节点
                    // ---------------------------------------------------------
                    if (new_left_node.LB < new_right_node.LB) {
                        // 左子节点下界更优
                        Values.search_flag = left_search_flag;

                        if (Values.search_flag != 1) {
                            Values.fathom_flag = 1;  // 深入左子节点
                            cout << "[分支定价] 左子节点_" << new_left_node.index
                                 << " (LB=" << fixed << setprecision(4) << new_left_node.LB
                                 << ") < 右子节点_" << new_right_node.index
                                 << " (LB=" << new_right_node.LB << ")\n";
                            cout << "[分支定价] 深入左子节点\n";
                            cout.unsetf(ios::fixed);
                        }
                    }

                    if (new_left_node.LB >= new_right_node.LB) {
                        // 右子节点下界更优或相等
                        Values.search_flag = right_search_flag;

                        if (Values.search_flag != 1) {
                            Values.fathom_flag = 2;  // 深入右子节点
                            cout << "[分支定价] 左子节点_" << new_left_node.index
                                 << " (LB=" << fixed << setprecision(4) << new_left_node.LB
                                 << ") >= 右子节点_" << new_right_node.index
                                 << " (LB=" << new_right_node.LB << ")\n";
                            cout << "[分支定价] 深入右子节点\n";
                            cout.unsetf(ios::fixed);
                        }
                    }
                }

                // 下一轮从左子节点开始
                Values.branch_status = 1;
            }
        }

        // ---------------------------------------------------------------------
        // 状态 2: 搜索其他节点 (search_flag = 1)
        // ---------------------------------------------------------------------
        if (Values.search_flag == 1) {
            // 当前节点解均为整数, 搜索其他未探索节点
            Values.branch_status = 3;   // 搜索模式
            Values.fathom_flag = -1;    // 重置深入标志
            Values.search_flag = 0;     // 继续搜索

            cout << "[分支定价] 当前节点解均为整数\n";
            cout << "[分支定价] 当前最优下界 = " << fixed << setprecision(4)
                 << Values.optimal_LB << "\n";
            cout.unsetf(ios::fixed);
        }

        // ---------------------------------------------------------------------
        // 安全检查: 防止无限循环
        // ---------------------------------------------------------------------
        // TODO: 建议将 30 定义为常量 MAX_NODES
        if (Values.node_num > 30) {
            cout << "[警告] 达到最大节点数 30, 强制终止\n";
            break;
        }
    }

    return 0;
}
