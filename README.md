# CS-2D-BP: 二维下料问题分支定价求解器

## 1. 项目概述

**CS-2D-BP** (2D Cutting Stock Problem - Branch and Price) 是一个用于求解二维下料问题的分支定价算法实现。

### 问题描述

- **输入**: 固定尺寸的母板 ($L \times W$) 和多种子件类型 (各有长度、宽度、需求量)
- **切割方式**: 两阶段切割 (Two-Stage Cutting)
- **目标**: 最小化使用的母板数量

### 技术栈

| 项目 | 说明 |
|------|------|
| 语言 | C++17 |
| 编译器 | MSVC (Visual Studio 2022) |
| 构建系统 | CMake 3.24+ |
| 优化求解器 | IBM CPLEX 22.1.0 |
| 平台 | Windows x64 |

---

## 2. 两阶段切割模型

### 切割层次

两阶段切割将二维切割问题分解为两个一维问题:

1. **第一阶段**: 沿宽度方向将母板切割成若干条带
2. **第二阶段**: 沿长度方向将条带切割成子件

### 切割规则

- 条带宽度由放入的第一个子件宽度决定
- 条带长度等于母板长度
- 子件宽度必须小于等于所属条带宽度

---

## 3. 数学模型

### 3.1 符号定义

| 符号 | 含义 |
|------|------|
| $K$ | 第一阶段切割模式数量 |
| $P$ | 第二阶段切割模式数量 |
| $J$ | 条带类型数量 |
| $N$ | 子件类型数量 |
| $L, W$ | 母板长度、宽度 |
| $l_i, w_i$ | 子件类型 $i$ 的长度、宽度 |
| $d_i$ | 子件类型 $i$ 的需求量 |

### 3.2 主问题 (Master Problem)

**决策变量**:
- $y_k$: 第一阶段模式 $k$ 的使用次数
- $x_p$: 第二阶段模式 $p$ 的使用次数

**目标函数**:

$$\min \sum_{k=1}^{K} y_k$$

**约束条件**:

条带平衡约束 ($j = 1, \ldots, J$):

$$\sum_{k=1}^{K} c_{jk} y_k - \sum_{p \in P_j} x_p \geq 0$$

其中 $c_{jk}$ 为模式 $k$ 中条带类型 $j$ 的产出数量，$P_j$ 为属于条带类型 $j$ 的模式集合。

子件需求约束 ($i = 1, \ldots, N$):

$$\sum_{p=1}^{P} b_{ip} x_p \geq d_i$$

其中 $b_{ip}$ 为模式 $p$ 中子件类型 $i$ 的切割数量。

### 3.3 第一阶段子问题 (SP1)

宽度方向背包问题，寻找新的母板切割模式:

$$\max \sum_{j=1}^{J} \pi_j G_j$$

$$\text{s.t.} \quad \sum_{j=1}^{J} w_j G_j \leq W$$

$$G_j \in \mathbb{Z}^+$$

其中 $\pi_j$ 为条带平衡约束的对偶价格，$w_j$ 为条带类型 $j$ 的宽度。

**定价判断**: 若目标值 $> 1$，则找到改进列 (reduced cost $> 0$)。

### 3.4 第二阶段子问题 (SP2)

长度方向背包问题，寻找新的条带切割模式:

$$\max \sum_{i=1}^{N} \mu_i D_i$$

$$\text{s.t.} \quad \sum_{i=1}^{N} l_i D_i \leq L$$

$$w_i \leq w_j \quad \text{(可行性约束)}$$

$$D_i \in \mathbb{Z}^+$$

其中 $\mu_i$ 为子件需求约束的对偶价格，$l_i$ 为子件类型 $i$ 的长度。

**定价判断**: 对条带类型 $j$，若目标值 $> \pi_j$，则找到改进列。

---

## 4. 求解算法

### 4.1 算法流程

1. **初始化**: 读取数据，启发式生成初始可行解
2. **根节点列生成**: 迭代求解主问题和子问题，直到收敛
3. **整数性检查**: 若解全为整数则输出；否则进入分支定界
4. **分支定价**: 对分数变量分支，递归求解子节点

### 4.2 列生成循环

每次迭代:
1. 求解主问题 LP，获取对偶价格 $(\pi, \mu)$
2. 求解 SP1，若 reduced cost $> 0$ 则添加新 $y$ 列
3. 若 SP1 无改进，对每种条带类型求解 SP2
4. 若 SP2 找到改进列则添加新 $x$ 列
5. 若无改进列则列生成收敛

### 4.3 分支策略

- **分支变量选择**: 选择第一个分数值变量
- **分支规则**: 左分支 $\leq \lfloor v \rfloor$，右分支 $\geq \lceil v \rceil$
- **节点选择**: 深度优先，优先选择下界较小的节点
- **剪枝条件**: 节点不可行，或下界 $\geq$ 当前最优整数解

---

## 5. 程序结构

### 5.1 目录结构

```
CS-2D-BP/
├── CMakeLists.txt          # 构建配置
├── CMakePresets.json       # 构建预设
├── README.md               # 项目说明
└── src/
    ├── 2DBP.h              # 主头文件
    ├── logger.h/cpp        # 日志系统
    ├── main.cpp            # 程序入口
    ├── input.cpp           # 数据读取
    ├── primal_heuristic.cpp           # 启发式初始解
    ├── root_node_column_generation.cpp # 根节点列生成
    ├── root_node_first_master_problem.cpp
    ├── sub_problem.cpp     # 子问题 (SP1 + SP2)
    ├── update_master_problem.cpp
    ├── branch_and_price.cpp           # 分支定价主循环
    ├── branching.cpp       # 分支变量选择
    ├── new_node_column_generation.cpp
    ├── new_node_first_master_problem.cpp
    ├── new_node_generating.cpp
    ├── output_models.cpp   # 模型输出
    └── output_results.cpp  # 结果输出
```

### 5.2 核心模块

| 模块 | 主要函数 | 功能 |
|------|----------|------|
| 数据读取 | `LoadInput` | 读取母板和子件数据 |
| 启发式 | `RunHeuristic` | 贪心生成初始可行解 |
| 列生成 | `SolveRootCG`, `SolveNodeCG` | 列生成主循环 |
| 主问题 | `SolveRootInitMP`, `UpdateMP`, `SolveFinalMP` | 构建和求解主问题 |
| 子问题 | `SolveSP1`, `SolveSP2` | 定价子问题求解 |
| 分支定界 | `RunBranchAndPrice`, `ProcessNode` | 分支定价树搜索 |
| 节点管理 | `SelectBranchNode`, `CreateChildNode` | 节点选择和生成 |

### 5.3 全局常量

| 常量 | 值 | 含义 |
|------|-----|------|
| `kRcTolerance` | $10^{-6}$ | reduced cost 容差 |
| `kZeroTolerance` | $10^{-10}$ | 浮点零值容差 |
| `kMaxCgIter` | 100 | 最大列生成迭代次数 |
| `kMaxBpNodes` | 30 | 最大分支节点数 |

---

## 6. 输入输出

### 6.1 输入格式

```
第1行: 母板数量
第2行: 子件类型数量
第3行: 母板长度 <TAB> 母板宽度
第4行起: 子件长度 <TAB> 子件宽度 <TAB> 需求量 <TAB> 类型索引
```

### 6.2 输出

- **控制台**: 迭代信息、目标值、分支进度
- **日志文件**: 带时间戳的完整运行日志
- **模型文件**: `.lp` 格式的 CPLEX 模型文件

---

## 7. 构建与运行

### 构建

```bash
cmake --preset vs2022-release
cmake --build --preset release
```

### 运行

```bash
./build/release/bin/Release/2DBP.exe
```

---

## 8. 代码规范

本项目遵循 Google C++ Style Guide:

| 类别 | 规范 | 示例 |
|------|------|------|
| 类/结构体 | PascalCase | `BPNode`, `ItemType` |
| 函数 | PascalCase | `SolveSP1`, `LoadInput` |
| 成员变量 | snake_case_ | `lower_bound_`, `type_id_` |
| 局部变量 | snake_case | `num_items`, `dual_val` |
| 常量 | kPascalCase | `kMaxCgIter` |

---

## 9. 算法特点

1. **两阶段分解**: 将二维问题转化为两个一维背包子问题
2. **动态列生成**: 按需生成切割模式，避免枚举爆炸
3. **分支定价集成**: 在 LP 松弛基础上保证整数最优解
4. **启发式加速**: 快速生成初始解，加速列生成收敛
5. **CPLEX 求解**: 利用商业求解器高效处理 LP/MIP

---

## 10. 参考文献

- Gilmore, P.C., Gomory, R.E. (1965). Multistage cutting stock problems of two and more dimensions. *Operations Research*.
- Barnhart, C., et al. (1998). Branch-and-price: Column generation for solving huge integer programs. *Operations Research*.
