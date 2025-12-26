# CS-2D-BP 技术文档: 二维下料问题分支定价求解器

## 1. 项目概述

**项目名称:** CS-2D-BP (2D Cutting Stock Problem - Branch and Price)

**问题领域:** 二维下料问题 (2D Cutting Stock Problem)
- 给定固定尺寸 (长度 L x 宽度 W) 的母板
- 多种类型的子件，具有不同长度、宽度和需求量
- 采用两阶段切割方式 (Two-Stage Cutting)
- 目标: 最小化使用的母板数量

**求解方法:** 分支定价算法 (Branch and Price)
- 列生成 (Column Generation) 求解LP松弛
- 分支定界 (Branch and Bound) 处理整数约束

**技术栈:**
- 语言: C++17
- 编译器: MSVC (Visual Studio 2022 Community)
- 构建系统: CMake 3.24+
- 优化求解器: IBM CPLEX 22.1.0
- 平台: Windows x64

**代码规模:** 约 2,700 行 C++ 代码

---

## 2. 目录结构

```
CS-2D-BP/
├── CMakeLists.txt                      # CMake构建配置
├── CMakePresets.json                   # 构建预设
└── src/
    ├── 2DBP.h                          # 主头文件 (数据结构和函数声明)
    ├── main.cpp                        # 程序入口
    ├── input.cpp                       # 数据读取
    ├── primal_heuristic.cpp            # 原始启发式 (初始解生成)
    ├── root_node_column_generation.cpp # 根节点列生成主循环
    ├── root_node_first_master_problem.cpp # 根节点初始主问题
    ├── sub_problem.cpp                 # 子问题求解 (SP1 + SP2)
    ├── update_master_problem.cpp       # 更新主问题和最终求解
    ├── branch_and_price.cpp            # 分支定价树主循环
    ├── branching.cpp                   # 分支变量选择
    ├── new_node_column_generation.cpp  # 非根节点列生成
    ├── new_node_first_master_problem.cpp # 非根节点初始主问题
    ├── new_node_generating.cpp         # 新节点生成
    ├── output_models.cpp               # 输出模型信息
    └── output_results.cpp              # 输出切割结果
```

---

## 3. 两阶段切割模型

### 3.1 切割层次

```
母板 (Stock)          条带 (Strip)           子件 (Item)
┌──────────────┐     ┌──────────────┐      ┌────┐ ┌────┐
│              │     │   Strip 1    │  --> │ I1 │ │ I2 │ ...
│              │     ├──────────────┤      └────┘ └────┘
│    Stock     │ --> │   Strip 2    │  --> ┌────┐ ┌────┐
│              │     ├──────────────┤      │ I3 │ │ I4 │ ...
│              │     │   Strip 3    │      └────┘ └────┘
└──────────────┘     └──────────────┘

    第一阶段切割              第二阶段切割
   (沿宽度方向)              (沿长度方向)
```

### 3.2 切割规则

**第一阶段 (Stock -> Strip):**
- 沿宽度方向将母板切割成若干条带
- 条带宽度由放入的第一个子件宽度决定
- 条带长度 = 母板长度

**第二阶段 (Strip -> Item):**
- 沿长度方向将条带切割成子件
- 子件宽度必须 <= 条带宽度
- 子件长度之和 <= 条带长度 (= 母板长度)

---

## 4. 数学模型

### 4.1 符号定义

| 符号 | 含义 |
|------|------|
| K | 第一阶段切割模式数量 (母板切割模式) |
| P | 第二阶段切割模式数量 (条带切割模式) |
| J | 条带类型数量 |
| N | 子件类型数量 |
| L | 母板长度 |
| W | 母板宽度 |
| l_i | 子件类型 i 的长度 |
| w_i | 子件类型 i 的宽度 |
| d_i | 子件类型 i 的需求量 |

### 4.2 主问题 (Master Problem)

**矩阵结构:**

```
              K_num 列           P_num 列
           (母板切割模式 Y)    (条带切割模式 X)
         ┌─────────────────┬─────────────────┐
  J_num  │        C        │        D        │  条带平衡约束 >= 0
   行    │   c_jk >= 0     │   d_jp = -1/0   │
         ├─────────────────┼─────────────────┤
  N_num  │        0        │        B        │  子件需求约束 >= demand_i
   行    │   全为 0        │   b_ip >= 0     │
         └─────────────────┴─────────────────┘
```

**决策变量:**
- Y_k: 第一阶段切割模式 k 的使用次数 (目标系数 = 1)
- X_p: 第二阶段切割模式 p 的使用次数 (目标系数 = 0)

**目标函数:**
```
Min sum(Y_k)  -- 最小化母板使用数量
```

**约束条件:**

1. **条带平衡约束** (J_num 个):
```
sum_k(c_jk * Y_k) + sum_p(d_jp * X_p) >= 0   对所有条带类型 j

其中:
  c_jk = 模式 k 中条带类型 j 的产出数量
  d_jp = -1 (若模式 p 属于条带类型 j) 或 0 (否则)
```

2. **子件需求约束** (N_num 个):
```
sum_p(b_ip * X_p) >= d_i   对所有子件类型 i

其中:
  b_ip = 模式 p 中子件类型 i 的切割数量
  d_i  = 子件类型 i 的需求量
```

### 4.3 第一阶段子问题 (SP1)

**问题描述:** 宽度方向的背包问题，寻找新的母板切割模式

**数学公式:**
```
Max sum_j(pi_j * G_j) - 1

s.t. sum_j(w_j * G_j) <= W
     G_j >= 0 且为整数

其中:
  pi_j = 条带类型约束 j 的对偶价格
  w_j  = 条带类型 j 的宽度
  W    = 母板宽度
  G_j  = 条带类型 j 的切割数量
```

**判定准则:**
- 若目标值 > 1 + RC_EPS: 找到改进列，加入主问题
- 否则: 继续求解 SP2

### 4.4 第二阶段子问题 (SP2)

**问题描述:** 长度方向的背包问题，寻找新的条带切割模式

**数学公式:**
```
Max sum_i(mu_i * D_i)

s.t. sum_i(l_i * D_i) <= L
     w_i <= w_j  (可行性约束: 子件宽度 <= 条带宽度)
     D_i >= 0 且为整数

其中:
  mu_i = 子件类型约束 i 的对偶价格
  l_i  = 子件类型 i 的长度
  L    = 母板长度 (= 条带长度)
  D_i  = 子件类型 i 的切割数量
```

**判定准则:**
- 对每种条带类型 j 求解 SP2
- 若目标值 > pi_j + RC_EPS: 找到改进列，加入主问题

---

## 5. 求解算法

### 5.1 算法总体流程

```
┌─────────────────────────────────────────────────────────┐
│                      程序开始                            │
└─────────────────────────────────────────────────────────┘
                           │
                           v
┌─────────────────────────────────────────────────────────┐
│  阶段1: 初始化                                          │
│  - ReadData(): 读取母板和子件数据                        │
│  - PrimalHeuristic(): 原始启发式生成初始可行解           │
└─────────────────────────────────────────────────────────┘
                           │
                           v
┌─────────────────────────────────────────────────────────┐
│  阶段2: 根节点列生成                                     │
│  - RootNodeColumnGeneration()                           │
│    ┌─────────────────────────────────────────────────┐  │
│    │  循环:                                          │  │
│    │  1. 求解主问题 MP -> 获取对偶价格               │  │
│    │  2. 求解子问题 SP1                              │  │
│    │     - 若 SP1 找到改进列 -> 加入 MP              │  │
│    │     - 否则求解 SP2 (对每种条带类型)              │  │
│    │  3. 若无改进列 -> 跳出循环                      │  │
│    └─────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
                           │
                           v
┌─────────────────────────────────────────────────────────┐
│  阶段3: 检查整数性                                       │
│  - FinishNode(): 检查解是否全为整数                      │
└─────────────────────────────────────────────────────────┘
                           │
              ┌────────────┴────────────┐
              v                         v
     ┌────────────────┐        ┌────────────────┐
     │  全为整数解    │        │  存在分数解    │
     │  输出最优解    │        │  进入分支定界  │
     └────────────────┘        └────────────────┘
                                        │
                                        v
┌─────────────────────────────────────────────────────────┐
│  阶段4: 分支定价树 (BranchAndPriceTree)                  │
│  ┌─────────────────────────────────────────────────────┐│
│  │  循环:                                              ││
│  │  1. 选择待分支节点 (ChooseNodeToBranch)             ││
│  │  2. 选择分支变量 (第一个分数解)                     ││
│  │  3. 创建左分支: var <= floor(val)                   ││
│  │  4. 创建右分支: var >= ceil(val)                    ││
│  │  5. 对每个子节点执行列生成                          ││
│  │  6. 剪枝/更新最优解                                 ││
│  │  7. 继续直到无待分支节点                            ││
│  └─────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────┘
                           │
                           v
┌─────────────────────────────────────────────────────────┐
│                      程序结束                            │
└─────────────────────────────────────────────────────────┘
```

### 5.2 列生成循环详解

```
┌─────────────────────────────────────────┐
│         求解主问题 (MP)                  │
│         获取 LP 最优解和对偶价格         │
└─────────────────────────────────────────┘
                    │
                    v
┌─────────────────────────────────────────┐
│         求解第一阶段子问题 (SP1)         │
│         Max sum(pi_j * G_j)             │
│         s.t. sum(w_j * G_j) <= W        │
└─────────────────────────────────────────┘
                    │
        ┌───────────┴───────────┐
        v                       v
   SP1_obj > 1?            SP1_obj <= 1
        │                       │
        v                       v
┌───────────────┐     ┌─────────────────────────┐
│ 添加新 Y 列    │     │ 对每种条带类型 j:       │
│ (母板切割模式) │     │   求解 SP2_j            │
│ 继续循环       │     │   Max sum(mu_i * D_i)   │
└───────────────┘     │   s.t. sum(l_i*D_i)<=L  │
                      │   若 SP2_obj > pi_j:    │
                      │     添加新 X 列          │
                      └─────────────────────────┘
                                  │
                     ┌────────────┴────────────┐
                     v                         v
              找到改进列                  无改进列
                     │                         │
                     v                         v
              继续循环                   列生成收敛
                                        求解最终 MP
```

### 5.3 分支策略

**分支变量选择:**
- 遍历所有非零解
- 选择第一个分数值作为分支变量

**分支规则:**
- 左分支: 变量值 <= floor(原值)
- 右分支: 变量值 >= ceil(原值)

**节点选择策略:**
- 优先深度优先搜索
- 选择下界较小的节点优先探索

**剪枝条件:**
- 节点不可行
- 节点下界 >= 当前最优整数解

---

## 6. 核心数据结构

### 6.1 子件相关

```cpp
// 子件类型
struct One_Item_Type {
    int item_type_idx;      // 子件类型索引
    double demand;          // 需求量
    int length;             // 长度
    int width;              // 宽度
    int this_item_type_num; // 当前模式中的数量
};

// 单个子件实例
struct One_Item {
    int item_idx;           // 子件索引
    int item_type_idx;      // 所属类型
    int demand;             // 需求量
    int length, width;      // 尺寸
    int area;               // 面积
    int pos_x, pos_y;       // 位置坐标
    int strip_idx;          // 所属条带
    int stock_idx;          // 所属母板
    int occupied_flag;      // 是否已分配
};
```

### 6.2 条带相关

```cpp
// 条带类型
struct One_Strip_Type {
    int strip_type_idx;      // 条带类型索引
    int width;               // 宽度
    int length;              // 长度
    int this_strip_type_num; // 当前模式中的数量
};

// 单个条带
struct One_Strip {
    int strip_idx;           // 条带索引
    int strip_type_idx;      // 所属类型
    int pattern;             // 切割模式编号
    vector<One_Item> items_list;           // 包含的子件
    vector<One_Item_Type> item_types_list; // 子件类型统计
    int length, width;       // 尺寸
    int area;                // 面积
    int pos_x, pos_y;        // 位置坐标
    int stock_idx;           // 所属母板
    int wasted_area;         // 废料面积
};
```

### 6.3 母板相关

```cpp
// 单个母板
struct One_Stock {
    int stock_idx;           // 母板索引
    int stock_type_idx;      // 母板类型
    int pattern;             // 切割模式编号
    vector<One_Strip> strips_list;           // 包含的条带
    vector<One_Strip_Type> strip_types_list; // 条带类型统计
    int length, width;       // 尺寸
    int area;                // 面积
    int pos_x, pos_y;        // 位置坐标
    int wasted_area;         // 废料面积
};
```

### 6.4 分支定界节点

```cpp
struct Node {
    int index;               // 节点索引
    double LB;               // 下界值

    // 父节点信息
    int parent_index;
    int parent_branching_flag;   // 1=左分支, 2=右分支
    double parent_var_to_branch_val;

    // 分支变量信息
    int var_to_branch_idx;       // 分支变量列索引
    double var_to_branch_soln;   // 分支变量解值
    double var_to_branch_floor;  // 向下取整
    double var_to_branch_ceil;   // 向上取整
    double var_to_branch_final;  // 最终整数值

    // 分支历史
    vector<int> branched_idx_list;    // 已分支变量索引列表
    vector<double> branched_int_list; // 已分支变量整数值列表

    // 列生成数据
    int iter;                         // 迭代次数
    vector<vector<double>> model_matrix;  // 系数矩阵
    vector<double> dual_prices_list;      // 对偶价格

    // 切割模式
    vector<One_Stock> Y_patterns_list;    // 第一阶段模式 (母板)
    vector<One_Strip> X_patterns_list;    // 第二阶段模式 (条带)
    vector<vector<double>> Y_cols_list;   // Y 变量列
    vector<vector<double>> X_cols_list;   // X 变量列

    // 新列
    vector<double> new_Y_col;
    vector<vector<double>> new_X_cols_list;

    // 解信息
    vector<double> all_solns_val_list;    // 所有变量的解值
    int node_pruned_flag;                 // 是否被剪枝
    int node_branched_flag;               // 是否已分支
};
```

### 6.5 全局数据容器

```cpp
struct All_Values {
    bool Finish;              // 启发式完成标志

    int item_types_num;       // 子件类型数
    int strip_types_num;      // 条带类型数
    int stocks_num;           // 母板数量
    int stock_length;         // 母板长度
    int stock_width;          // 母板宽度

    int node_num;             // 节点计数
    double optimal_LB;        // 当前最优下界

    int branch_status;        // 分支状态: 1=左, 2=右, 3=搜索
    int search_flag;          // 搜索标志: 0=继续分支, 1=搜索新节点
    int fathom_flag;          // 深入标志: 1=左节点, 2=右节点
    int root_flag;            // 根节点标志
};

struct All_Lists {
    vector<Node> all_nodes_list;          // 所有节点
    vector<One_Stock> all_stocks_list;    // 所有母板
    vector<One_Strip> all_strips_list;    // 所有条带
    vector<One_Item> all_items_list;      // 所有子件
    vector<One_Strip_Type> all_strip_types_list;
    vector<One_Item_Type> all_item_types_list;
    vector<One_Stock> occupied_stocks_list;   // 已使用母板
    vector<One_Item> occupied_items_list;     // 已分配子件
};
```

---

## 7. 关键函数说明

| 函数 | 文件 | 功能 |
|------|------|------|
| main | main.cpp | 程序入口，调度各阶段 |
| ReadData | input.cpp | 读取输入数据文件 |
| SplitString | input.cpp | 字符串分割工具 |
| PrimalHeuristic | primal_heuristic.cpp | 原始启发式生成初始解和矩阵 |
| RootNodeColumnGeneration | root_node_column_generation.cpp | 根节点列生成主循环 |
| SolveRootNodeFirstMasterProblem | root_node_first_master_problem.cpp | 求解根节点初始主问题 |
| SolveStageOneSubProblem | sub_problem.cpp | 求解第一阶段子问题 SP1 |
| SolveStageTwoSubProblem | sub_problem.cpp | 求解第二阶段子问题 SP2 |
| SolveUpdateMasterProblem | update_master_problem.cpp | 添加新列并更新主问题 |
| SolveFinalMasterProblem | update_master_problem.cpp | 求解最终主问题 |
| BranchAndPriceTree | branch_and_price.cpp | 分支定价树主循环 |
| FinishNode | branching.cpp | 完成节点处理，检查整数性 |
| ChooseVarToBranch | branching.cpp | 选择分支变量 |
| ChooseNodeToBranch | new_node_generating.cpp | 选择待分支节点 |
| GenerateNewNode | new_node_generating.cpp | 生成新的分支节点 |
| NewNodeColumnGeneration | new_node_column_generation.cpp | 非根节点列生成 |
| SolveNewNodeFirstMasterProblem | new_node_first_master_problem.cpp | 非根节点初始主问题 |
| OutputMasterProblem | output_models.cpp | 输出主问题模型 |
| OutputHeuristicResults | output_results.cpp | 输出启发式结果 |

---

## 8. 输入输出格式

### 8.1 输入文件格式

```
第1行: 母板数量
第2行: 子件类型数量
第3行: 母板长度 <TAB> 母板宽度
第4行起: 子件长度 <TAB> 子件宽度 <TAB> 需求量 <TAB> 类型索引
```

**示例:**
```
10
3
1000    500
200     100     5       1
150     80      3       2
300     120     4       3
```

### 8.2 输出内容

- **控制台输出:**
  - 列生成迭代信息
  - 主问题解值和对偶价格
  - 分支定界进度
  - 最终最优解

- **文件输出:**
  - `The First Master Problem.lp`: 初始主问题模型
  - `Final Master Problem.lp`: 最终主问题模型
  - `Master Problem.txt`: 主问题矩阵
  - `Stock_*.txt`: 切割方案坐标 (用于绘图)

---

## 9. 构建与运行

### 9.1 构建配置

```
编译器: MSVC (Visual Studio 2022 Community)
C++标准: C++17
平台: Windows x64
CPLEX路径: D:/CPLEX
```

### 9.2 构建命令

```bash
cmake --preset vs2022-release
cmake --build --preset build-release
```

### 9.3 运行

```bash
./build/bin/Release/2DBP.exe
```

---

## 10. 算法复杂度

### 10.1 列生成

- **主问题:** LP求解，多项式时间
- **SP1:** 一维背包问题，O(J * W)
- **SP2:** 一维背包问题，O(N * L)
- **迭代次数:** 依赖实例，通常几十到几百次

### 10.2 分支定界

- **节点数:** 最坏指数级，实际取决于问题结构
- **每节点:** 完整列生成过程
- **剪枝效果:** 决定实际复杂度

---

## 11. 与一维问题的区别

| 方面 | 1D-CSP (CS-1D) | 2D-CSP (CS-2D-BP) |
|------|----------------|-------------------|
| 切割维度 | 一维 (长度) | 二维 (长度 x 宽度) |
| 切割阶段 | 单阶段 | 两阶段 |
| 决策变量 | 一类 (模式使用次数) | 两类 (Y: 母板模式, X: 条带模式) |
| 子问题 | 一个背包问题 | 两个背包问题 (SP1 + SP2) |
| 约束类型 | 需求约束 | 需求约束 + 条带平衡约束 |
| 模型复杂度 | 较低 | 较高 |

---

## 12. 技术亮点

1. **两阶段分解:** 将复杂的二维问题分解为两个一维背包子问题

2. **列生成效率:** 动态生成切割模式，避免枚举所有可能模式

3. **分支定界集成:** 在LP松弛基础上保证整数最优解

4. **启发式初始化:** 快速生成初始可行解，加速收敛

5. **CPLEX集成:** 利用商业求解器处理LP和MIP子问题

---

## 13. 总结

CS-2D-BP 实现了用于二维下料问题的**分支定价算法**，采用**两阶段切割**策略。核心特点:

- **两阶段建模:** 母板->条带->子件的分层切割
- **列生成:** SP1 (宽度背包) + SP2 (长度背包)
- **分支定界:** 基于变量值的分支策略
- **CPLEX求解:** 高效处理LP和IP子问题

该方法适用于需要两阶段切割的工业下料场景，能够在合理时间内找到高质量的整数最优解或近似最优解。
