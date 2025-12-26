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
    ├── logger.h                        # 日志工具头文件
    ├── logger.cpp                      # 日志工具实现
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

## 3. 命名规范

本项目采用 Google C++ Style Guide 命名规范:

| 类别 | 规范 | 示例 |
|------|------|------|
| 结构体/类 | PascalCase | `ItemType`, `BPNode`, `ProblemParams` |
| 函数 | PascalCase | `LoadInput`, `SolveSP1`, `RunBranchAndPrice` |
| 成员变量 | snake_case_ | `type_id_`, `lower_bound_`, `branch_var_val_` |
| 局部变量 | snake_case | `num_items`, `mp_env`, `cur_node` |
| 常量 | kPascalCase | `kRcTolerance`, `kMaxCgIter`, `kMaxBpNodes` |

---

## 4. 两阶段切割模型

### 4.1 切割层次

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

### 4.2 切割规则

**第一阶段 (Stock -> Strip):**
- 沿宽度方向将母板切割成若干条带
- 条带宽度由放入的第一个子件宽度决定
- 条带长度 = 母板长度

**第二阶段 (Strip -> Item):**
- 沿长度方向将条带切割成子件
- 子件宽度必须 <= 条带宽度
- 子件长度之和 <= 条带长度 (= 母板长度)

---

## 5. 数学模型

### 5.1 符号定义

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

### 5.2 主问题 (Master Problem)

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

### 5.3 第一阶段子问题 (SP1)

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
- 若目标值 > 1 + kRcTolerance: 找到改进列，加入主问题
- 否则: 继续求解 SP2

### 5.4 第二阶段子问题 (SP2)

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
- 若目标值 > pi_j + kRcTolerance: 找到改进列，加入主问题

---

## 6. 求解算法

### 6.1 算法总体流程

```
┌─────────────────────────────────────────────────────────┐
│                      程序开始                            │
└─────────────────────────────────────────────────────────┘
                           │
                           v
┌─────────────────────────────────────────────────────────┐
│  阶段1: 初始化                                          │
│  - LoadInput(): 读取母板和子件数据                       │
│  - RunHeuristic(): 原始启发式生成初始可行解              │
└─────────────────────────────────────────────────────────┘
                           │
                           v
┌─────────────────────────────────────────────────────────┐
│  阶段2: 根节点列生成                                     │
│  - SolveRootCG()                                        │
│    ┌─────────────────────────────────────────────────┐  │
│    │  循环:                                          │  │
│    │  1. 求解主问题 MP -> 获取对偶价格               │  │
│    │  2. 求解子问题 SolveSP1()                       │  │
│    │     - 若 SP1 找到改进列 -> 加入 MP              │  │
│    │     - 否则求解 SolveSP2() (对每种条带类型)       │  │
│    │  3. 若无改进列 -> 跳出循环                      │  │
│    └─────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
                           │
                           v
┌─────────────────────────────────────────────────────────┐
│  阶段3: 检查整数性                                       │
│  - ProcessNode(): 检查解是否全为整数                     │
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
│  阶段4: 分支定价树 (RunBranchAndPrice)                   │
│  ┌─────────────────────────────────────────────────────┐│
│  │  循环:                                              ││
│  │  1. 选择待分支节点 (SelectBranchNode)               ││
│  │  2. 选择分支变量 (SelectBranchVar)                  ││
│  │  3. 创建左分支: var <= floor(val)                   ││
│  │  4. 创建右分支: var >= ceil(val)                    ││
│  │  5. 对每个子节点执行列生成 (SolveNodeCG)             ││
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

### 6.2 列生成循环详解

```
┌─────────────────────────────────────────┐
│         求解主问题 (MP)                  │
│         获取 LP 最优解和对偶价格         │
└─────────────────────────────────────────┘
                    │
                    v
┌─────────────────────────────────────────┐
│         求解第一阶段子问题 (SolveSP1)    │
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
│ (母板切割模式) │     │   求解 SolveSP2()       │
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

### 6.3 分支策略

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

## 7. 核心数据结构

### 7.1 子件相关

```cpp
// 子件类型
struct ItemType {
    int type_id_ = -1;        // 子件类型索引 (从 1 开始编号)
    int length_ = -1;         // 子件长度 (沿 X 轴方向)
    int width_ = -1;          // 子件宽度 (沿 Y 轴方向)
    double demand_ = -1;      // 需求量
    int count_ = 0;           // 当前模式中该类型子件的数量
};

// 单个子件实例
struct Item {
    int id_ = -1;             // 子件实例索引 (全局唯一)
    int type_id_ = -1;        // 所属子件类型索引
    int length_ = -1;         // 子件长度
    int width_ = -1;          // 子件宽度
    int area_ = -1;           // 子件面积
    int demand_ = -1;         // 该类型的需求量
    int x_ = -1;              // 左上角 X 坐标
    int y_ = -1;              // 左上角 Y 坐标
    int strip_id_ = -1;       // 所属条带索引
    int stock_id_ = -1;       // 所属母板索引
    int assign_flag_ = 0;     // 分配标志: 0=未分配, 1=已分配
};
```

### 7.2 条带相关

```cpp
// 条带类型
struct StripType {
    int type_id_ = -1;        // 条带类型索引 (从 1 开始编号)
    int width_ = -1;          // 条带宽度 (沿 Y 轴方向)
    int length_ = -1;         // 条带长度 (沿 X 轴方向)
    int count_ = 0;           // 当前模式中该类型条带的数量
};

// 单个条带
struct Strip {
    int id_ = -1;             // 条带实例索引 (全局唯一)
    int type_id_ = -1;        // 条带类型索引
    int pattern_ = -1;        // 切割模式编号
    vector<Item> items_;              // 该条带内的所有子件列表
    vector<ItemType> item_types_;     // 各子件类型的统计信息
    int length_ = -1;         // 条带长度 (= 母板长度)
    int width_ = -1;          // 条带宽度 (= 首个子件宽度)
    int area_ = -1;           // 条带面积
    int x_ = -1;              // 左上角 X 坐标
    int y_ = -1;              // 左上角 Y 坐标
    int stock_id_ = -1;       // 所属母板索引
    int waste_area_ = -1;     // 废料面积
};
```

### 7.3 母板相关

```cpp
// 单个母板
struct Stock {
    int id_ = -1;             // 母板实例索引 (全局唯一)
    int type_id_ = 0;         // 母板类型索引 (当前版本固定为 0)
    int pattern_ = -1;        // 切割模式编号
    vector<Strip> strips_;            // 该母板内的所有条带列表
    vector<StripType> strip_types_;   // 各条带类型的统计信息
    int length_ = -1;         // 母板长度 (沿 X 轴)
    int width_ = -1;          // 母板宽度 (沿 Y 轴)
    int area_ = -1;           // 母板面积
    int x_ = -1;              // 左上角 X 坐标
    int y_ = -1;              // 左上角 Y 坐标
    int waste_area_ = -1;     // 总废料面积
};
```

### 7.4 分支定界节点

```cpp
struct BPNode {
    // 节点标识
    int id_ = -1;                     // 节点索引 (根节点为 1)

    // 父节点信息
    int parent_id_ = -1;              // 父节点索引 (-1 表示根节点)
    int parent_branch_dir_ = -1;      // 父节点的分支方向: 1=左, 2=右
    double parent_branch_val_ = -1;   // 父节点分支变量的原始值

    // 节点状态
    double lower_bound_ = -1;         // 该节点的下界值 (LP 松弛最优值)
    int prune_flag_ = -1;             // 剪枝标志: 0=未剪枝, 1=已剪枝
    int branched_flag_ = -1;          // 分支完成标志: 0=未分支, 1=已分支

    // 分支变量信息
    int branch_var_id_ = -1;          // 待分支变量的列索引 (0-based)
    double branch_var_val_ = -1;      // 待分支变量的解值 (分数值)
    double branch_floor_ = -1;        // 向下取整值
    double branch_ceil_ = -1;         // 向上取整值
    double branch_bound_ = -1;        // 最终确定的整数值

    // 分支历史
    vector<int> branched_var_ids_;    // 所有已分支变量的列索引
    vector<double> branched_bounds_;  // 所有已分支变量的整数值
    vector<double> branched_vals_;    // 所有已分支变量的原始解值

    // 解信息
    vector<double> solution_;         // 所有变量的最终解值

    // 列生成迭代数据
    int iter_ = -1;                   // 当前迭代次数
    vector<vector<double>> matrix_;   // 当前主问题的系数矩阵
    vector<double> duals_;            // 主问题约束的对偶价格

    // 切割模式存储
    vector<Stock> y_patterns_;        // 第一阶段模式详细信息
    vector<Strip> x_patterns_;        // 第二阶段模式详细信息
    vector<vector<double>> y_cols_;   // 第一阶段模式的系数列
    vector<vector<double>> x_cols_;   // 第二阶段模式的系数列

    // 新生成的列
    vector<double> new_y_col_;        // 新的第一阶段模式列
    vector<vector<double>> new_x_cols_;  // 新的第二阶段模式列集合

    // 子问题信息
    double sp2_obj_ = -1;             // 第二阶段子问题的最优目标值
    vector<double> sp2_solution_;     // 第二阶段子问题的最优解
    int col_type_flag_ = -1;          // 新列类型: 1=Y列, 0=X列
};
```

### 7.5 全局数据容器

```cpp
struct ProblemParams {
    // 算法控制标志
    bool is_finished_ = false;        // 启发式完成标志

    // 问题规模参数
    int num_item_types_ = -1;         // 子件类型数量 (N)
    int num_strip_types_ = -1;        // 条带类型数量 (J)
    int num_stocks_ = -1;             // 可用母板数量
    int stock_length_ = -1;           // 母板长度 (L)
    int stock_width_ = -1;            // 母板宽度 (W)

    // 分支定界树参数
    int num_nodes_ = -1;              // 已生成的节点总数
    double best_obj_ = -1;            // 当前最优下界

    // 分支状态标志
    int branch_state_ = -1;           // 1=生成左子节点, 2=生成右子节点, 3=搜索其他节点
    int need_search_ = -1;            // 0=继续分支, 1=搜索其他节点
    int fathom_dir_ = -1;             // 深入方向: 1=左, 2=右
    int is_at_root_ = -1;             // 1=当前在根节点, 0=不在
};

struct ProblemData {
    // 分支定界树节点
    vector<BPNode> nodes_;            // 所有分支节点的列表

    // 切割对象列表
    vector<Stock> stocks_;            // 所有可用母板的列表
    vector<Strip> strips_;            // 所有已生成条带的列表
    vector<Item> items_;              // 所有子件的列表

    // 类型列表
    vector<StripType> strip_types_;   // 所有条带类型的列表
    vector<ItemType> item_types_;     // 所有子件类型的列表

    // 已使用/已分配列表
    vector<Stock> used_stocks_;       // 已使用的母板列表
    vector<Item> assigned_items_;     // 已分配的子件列表
};
```

---

## 8. 关键函数说明

| 函数 | 文件 | 功能 |
|------|------|------|
| main | main.cpp | 程序入口，调度各阶段 |
| LoadInput | input.cpp | 读取输入数据文件 |
| SplitString | input.cpp | 字符串分割工具 |
| RunHeuristic | primal_heuristic.cpp | 原始启发式生成初始解和矩阵 |
| SolveRootCG | root_node_column_generation.cpp | 根节点列生成主循环 |
| SolveRootInitMP | root_node_first_master_problem.cpp | 求解根节点初始主问题 |
| SolveSP1 | sub_problem.cpp | 求解第一阶段子问题 |
| SolveSP2 | sub_problem.cpp | 求解第二阶段子问题 |
| UpdateMP | update_master_problem.cpp | 添加新列并更新主问题 |
| SolveFinalMP | update_master_problem.cpp | 求解最终主问题 |
| RunBranchAndPrice | branch_and_price.cpp | 分支定价树主循环 |
| ProcessNode | branching.cpp | 完成节点处理，检查整数性 |
| SelectBranchVar | branching.cpp | 选择分支变量 |
| SelectBranchNode | new_node_generating.cpp | 选择待分支节点 |
| CreateChildNode | new_node_generating.cpp | 生成新的分支节点 |
| SolveNodeCG | new_node_column_generation.cpp | 非根节点列生成 |
| SolveNodeInitMP | new_node_first_master_problem.cpp | 非根节点初始主问题 |
| ExportMP | output_models.cpp | 输出主问题模型 |
| ExportDualMP | output_models.cpp | 输出对偶主问题模型 |
| ExportResults | output_results.cpp | 输出切割结果 |

---

## 9. 全局常量

```cpp
// 检验数容差 (Reduced Cost Tolerance)
constexpr double kRcTolerance = 1.0e-6;

// 最大迭代次数
constexpr int kMaxCgIter = 100;

// 最大节点数
constexpr int kMaxBpNodes = 30;
```

---

## 10. 输入输出格式

### 10.1 输入文件格式

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

### 10.2 输出内容

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

## 11. 构建与运行

### 11.1 构建配置

```
编译器: MSVC (Visual Studio 2022 Community)
C++标准: C++17
平台: Windows x64
CPLEX路径: D:/CPLEX
```

### 11.2 构建命令

```bash
cmake --preset vs2022-release
cmake --build --preset release
```

### 11.3 运行

```bash
./build/release/bin/Release/2DBP.exe
```

---

## 12. 算法复杂度

### 12.1 列生成

- **主问题:** LP求解，多项式时间
- **SP1:** 一维背包问题，O(J * W)
- **SP2:** 一维背包问题，O(N * L)
- **迭代次数:** 依赖实例，通常几十到几百次

### 12.2 分支定界

- **节点数:** 最坏指数级，实际取决于问题结构
- **每节点:** 完整列生成过程
- **剪枝效果:** 决定实际复杂度

---

## 13. 与一维问题的区别

| 方面 | 1D-CSP (CS-1D) | 2D-CSP (CS-2D-BP) |
|------|----------------|-------------------|
| 切割维度 | 一维 (长度) | 二维 (长度 x 宽度) |
| 切割阶段 | 单阶段 | 两阶段 |
| 决策变量 | 一类 (模式使用次数) | 两类 (Y: 母板模式, X: 条带模式) |
| 子问题 | 一个背包问题 | 两个背包问题 (SP1 + SP2) |
| 约束类型 | 需求约束 | 需求约束 + 条带平衡约束 |
| 模型复杂度 | 较低 | 较高 |

---

## 14. 技术亮点

1. **两阶段分解:** 将复杂的二维问题分解为两个一维背包子问题

2. **列生成效率:** 动态生成切割模式，避免枚举所有可能模式

3. **分支定界集成:** 在LP松弛基础上保证整数最优解

4. **启发式初始化:** 快速生成初始可行解，加速收敛

5. **CPLEX集成:** 利用商业求解器处理LP和MIP子问题

6. **规范化命名:** 采用Google C++ Style Guide，代码可读性强

---

## 15. 总结

CS-2D-BP 实现了用于二维下料问题的**分支定价算法**，采用**两阶段切割**策略。核心特点:

- **两阶段建模:** 母板->条带->子件的分层切割
- **列生成:** SolveSP1 (宽度背包) + SolveSP2 (长度背包)
- **分支定界:** 基于变量值的分支策略
- **CPLEX求解:** 高效处理LP和IP子问题

该方法适用于需要两阶段切割的工业下料场景，能够在合理时间内找到高质量的整数最优解或近似最优解。
