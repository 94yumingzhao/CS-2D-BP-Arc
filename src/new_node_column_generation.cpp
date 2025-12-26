// =============================================================================
// new_node_column_generation.cpp - 新节点列生成
// =============================================================================
//
// 功能: 对分支产生的新节点执行列生成求解
//
// 与根节点列生成的区别:
//   - 新节点继承父节点的列集合
//   - 分支变量被固定为整数值
//   - 之前分支的变量保持固定
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// NewNodeColumnGeneration - 新节点列生成主循环
// -----------------------------------------------------------------------------
// 功能: 对新生成的子节点执行完整的列生成过程
// 参数:
//   Values      - 全局参数
//   Lists       - 全局列表
//   this_node   - 当前新节点
//   parent_node - 父节点
// -----------------------------------------------------------------------------
void NewNodeColumnGeneration(
	All_Values& Values,
	All_Lists& Lists,
	Node& this_node,
	Node& parent_node) {

	// ----- 初始化 CPLEX 环境 -----
	IloEnv Env_MP;
	IloModel Model_MP(Env_MP);
	IloObjective Obj_MP = IloAdd(Model_MP, IloMinimize(Env_MP));
	IloRangeArray Cons_MP(Env_MP);
	IloNumVarArray Vars_MP(Env_MP);

	this_node.iter = 0;

	// ----- 求解初始主问题 -----
	bool MP_flag = SolveNewNodeFirstMasterProblem(
		Values,
		Lists,
		Env_MP,
		Model_MP,
		Obj_MP,
		Cons_MP,
		Vars_MP,
		this_node,
		parent_node);

	cout << endl;

	if (MP_flag == 1) {
		// 初始主问题可行, 进入列生成循环
		while (1) {
			this_node.iter++;

			// 求解子问题
			int SP_flag = SolveStageOneSubProblem(Values, Lists, this_node);

			if (SP_flag == 0) {
				// 无改进列, 列生成收敛
				break;
			}
			if (SP_flag == 1) {
				// 找到改进列, 更新主问题
				SolveUpdateMasterProblem(
					Values,
					Lists,
					Env_MP,
					Model_MP,
					Obj_MP,
					Cons_MP,
					Vars_MP,
					this_node);
			}
		}

		// 求解最终主问题
		SolveFinalMasterProblem(
			Values,
			Lists,
			Env_MP,
			Model_MP,
			Obj_MP,
			Cons_MP,
			Vars_MP,
			this_node);
	}

	// ----- 释放 CPLEX 资源 -----
	Vars_MP.clear();
	Vars_MP.end();
	Cons_MP.clear();
	Cons_MP.end();
	Obj_MP.removeAllProperties();
	Obj_MP.end();
	Model_MP.removeAllProperties();
	Model_MP.end();
	Env_MP.removeAllProperties();
	Env_MP.end();

	cout << endl;
}
