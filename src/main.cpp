// =============================================================================
// main.cpp - 程序入口
// =============================================================================
//
// 功能: 二维下料问题分支定价求解器的主函数
//
// 算法流程:
//   1. 读取数据
//   2. 原始启发式生成初始解
//   3. 根节点列生成求解
//   4. 检查整数性, 若需要则进入分支定界
//   5. 输出结果
//
// =============================================================================

#include "2DBP.h"

using namespace std;

int main() {
	// 记录程序开始时间
	clock_t start, finish;
	start = clock();

	// 初始化全局数据容器
	All_Lists Lists;
	All_Values Values;

	// 初始化根节点
	Node root_node;
	root_node.index = 1;
	Values.branch_status = 0;

	// ---------------------------
	// 阶段1: 数据读取与预处理
	// ---------------------------
	ReadData(Values, Lists);

	// ---------------------------
	// 阶段2: 原始启发式生成初始解
	// ---------------------------
	PrimalHeuristic(Values, Lists, root_node);

	// ---------------------------
	// 阶段3: 根节点列生成求解
	// ---------------------------
	RootNodeColumnGeneration(Values, Lists, root_node);

	// ---------------------------
	// 阶段4: 检查整数性并决定是否分支
	// ---------------------------
	Values.search_flag = FinishNode(Values, Lists, root_node);

	// 将根节点加入节点列表
	Lists.all_nodes_list.push_back(root_node);
	Values.root_flag = 1;

	// ---------------------------
	// 阶段5: 分支定界 (若需要)
	// ---------------------------
	if (Values.search_flag == 0) {
		// 存在分数解, 进入分支定界
		Values.branch_status = 1;
		BranchAndPriceTree(Values, Lists);
	}

	// 记录并输出运行时间
	finish = clock();
	double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("\n\t Process Time = %f seconds\n", duration);

	return 0;
}
