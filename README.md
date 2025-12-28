# CS-2D-BP-Arc: 二维下料问题分支定价求解器 (Arc Flow)

## 1. 项目概述

**CS-2D-BP-Arc** 是一个用于求解二维下料问题的分支定价算法实现，采用两阶段切割模型，子问题支持 CPLEX IP、Arc Flow、DP 三种求解方法，分支策略采用 Arc 流量分支。

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

```
母板 (L x W)
    |
    | 第一阶段: 沿宽度方向切割 (SP1)
    v
条带 (L x w_j)
    |
    | 第二阶段: 沿长度方向切割 (SP2)
    v
子件 (l_i x w_i)
```

### 切割规则

- 条带宽度 $w_j$ 对应某种子件宽度
- 条带长度等于母板长度 $L$
- 放入条带的子件宽度必须等于该条带宽度

---

## 3. 数学模型

### 3.1 符号定义

| 符号 | 含义 |
|------|------|
| $L, W$ | 母板长度、宽度 |
| $N$ | 子件类型数量 |
| $J$ | 条带类型数量 |
| $l_i, w_i, d_i$ | 子件类型 $i$ 的长度、宽度、需求量 |
| $w_j$ | 条带类型 $j$ 的宽度 |

### 3.2 主问题 (Master Problem)

**决策变量**:
- $y_k$: Y列 (母板切割模式) $k$ 的使用次数
- $x_p$: X列 (条带切割模式) $p$ 的使用次数

**目标函数**:

$$\min \sum_{k} y_k$$

**约束条件**:

条带平衡约束 (条带产出 >= 条带消耗):

$$\sum_{k} c_{jk} y_k - \sum_{p \in P_j} x_p \geq 0, \quad \forall j$$

子件需求约束:

$$\sum_{p} b_{ip} x_p \geq d_i, \quad \forall i$$

### 3.3 SP1: 宽度方向背包

选择条带放置在母板上:

$$\max \sum_{j=1}^{J} \pi_j G_j$$

$$\text{s.t.} \quad \sum_{j=1}^{J} w_j G_j \leq W, \quad G_j \in \mathbb{Z}^+$$

其中 $\pi_j$ 为条带平衡约束的对偶价格。

### 3.4 SP2: 长度方向背包

选择子件放置在条带上:

$$\max \sum_{i: w_i = w_j} \mu_i D_i$$

$$\text{s.t.} \quad \sum_{i: w_i = w_j} l_i D_i \leq L, \quad D_i \in \mathbb{Z}^+$$

其中 $\mu_i$ 为子件需求约束的对偶价格。

---

## 4. Arc Flow 模型

### 4.1 网络结构

将背包问题建模为网络流问题:

**SP1 网络 (宽度方向)**:
- 节点: $0, 1, 2, \ldots, W$ 表示已使用的母板宽度
- Arc $(i, i+w_j)$: 在位置 $i$ 放置宽度为 $w_j$ 的条带
- 目标: 从节点 0 到节点 W 的最大价值流

**SP2 网络 (长度方向)**:
- 节点: $0, 1, 2, \ldots, L$ 表示已使用的条带长度
- Arc $(i, i+l_k)$: 在位置 $i$ 放置长度为 $l_k$ 的子件
- 每种条带类型有独立的 SP2 网络

### 4.2 Arc 数据结构

```cpp
struct SP1ArcFlowData {
    vector<int> begin_nodes_;           // 起点节点 (位置0)
    vector<int> end_nodes_;             // 终点节点 (位置W)
    vector<int> mid_nodes_;             // 中间节点
    vector<array<int, 2>> arc_list_;    // Arc列表 [起点, 终点]
    map<array<int, 2>, int> arc_to_index_;
};
```

---

## 5. 子问题求解方法

| 方法 | SP1 (宽度) | SP2 (长度) | 复杂度 |
|------|------------|------------|--------|
| CPLEX IP | `SolveRootSP1Knapsack` | `SolveRootSP2Knapsack` | 依赖求解器 |
| Arc Flow | `SolveRootSP1ArcFlow` | `SolveRootSP2ArcFlow` | 依赖求解器 |
| DP | `SolveRootSP1DP` | `SolveRootSP2DP` | $O(J \cdot W)$ / $O(N \cdot L)$ |

通过 `SPMethod` 枚举选择方法:

```cpp
enum SPMethod {
    kCplexIP = 0,   // CPLEX整数规划
    kArcFlow = 1,   // Arc Flow模型
    kDP = 2         // 动态规划
};
```

---

## 6. Arc 流量分支策略

### 6.1 分支类型

```cpp
enum BranchType {
    kBranchNone = 0,    // 无需分支 (整数解)
    kBranchSP1Arc = 1,  // SP1 Arc 分支 (宽度方向)
    kBranchSP2Arc = 2   // SP2 Arc 分支 (长度方向)
};
```

### 6.2 分支规则

若 Arc $(i, j)$ 的流量 $f$ 为分数值:
- **左分支**: $f_{ij} \leq \lfloor f \rfloor$ (若 $\lfloor f \rfloor = 0$ 则禁用该 Arc)
- **右分支**: $f_{ij} \geq \lceil f \rceil$

### 6.3 分支约束继承

子节点继承父节点的所有 Arc 约束:

```cpp
struct BPNode {
    // SP1 Arc 约束
    set<array<int, 2>> sp1_zero_arcs_;          // Arc = 0 (禁用)
    vector<array<int, 2>> sp1_lower_arcs_;      // Arc <= N
    vector<array<int, 2>> sp1_greater_arcs_;    // Arc >= N

    // SP2 Arc 约束 (按条带类型)
    map<int, set<array<int, 2>>> sp2_zero_arcs_;
    map<int, vector<array<int, 2>>> sp2_lower_arcs_;
    map<int, vector<array<int, 2>>> sp2_greater_arcs_;
};
```

---

## 7. 程序结构

### 7.1 目录结构

```
CS-2D-BP-Arc/
├── CMakeLists.txt              # 构建配置
├── CMakePresets.json           # 构建预设
├── data/                       # 测试数据
├── logs/                       # 运行日志
├── lp/                         # LP模型文件
└── src/
    ├── 2DBP.h                  # 主头文件 (数据结构 + 函数声明)
    ├── logger.h/cpp            # 日志系统
    ├── main.cpp                # 程序入口
    ├── input.cpp               # 数据读取 + 打印函数
    ├── arc_flow.cpp            # Arc Flow 网络构建 + 解转换
    ├── heuristic.cpp           # 启发式初始解
    ├── column_generation.cpp   # 列生成调度
    ├── root_node.cpp           # 根节点主问题
    ├── root_node_sub.cpp       # 根节点子问题 (SP1 + SP2)
    ├── new_node.cpp            # 非根节点主问题
    ├── new_node_sub.cpp        # 非根节点子问题
    └── branch_and_price.cpp    # 分支定价主循环 + Arc分支
```

### 7.2 核心模块

| 文件 | 主要函数 | 功能 |
|------|----------|------|
| `arc_flow.cpp` | `GenerateSP1Arcs`, `GenerateSP2Arcs` | 构建 Arc Flow 网络 |
| `arc_flow.cpp` | `ConvertYColsToSP1ArcFlow` | Y列解转Arc流量 |
| `arc_flow.cpp` | `FindBranchArcSP1`, `FindBranchArcSP2` | 寻找分数流量Arc |
| `root_node_sub.cpp` | `SolveRootSP1ArcFlow`, `SolveRootSP2ArcFlow` | Arc Flow子问题求解 |
| `branch_and_price.cpp` | `SelectBranchArc` | 选择分支Arc |
| `branch_and_price.cpp` | `CreateLeftChild`, `CreateRightChild` | 创建子节点 |

### 7.3 全局常量

| 常量 | 值 | 含义 |
|------|-----|------|
| `kRcTolerance` | $10^{-6}$ | reduced cost 容差 |
| `kZeroTolerance` | $10^{-10}$ | 浮点零值容差 |
| `kMaxCgIter` | 100 | 最大列生成迭代次数 |
| `kExportLp` | false | 是否导出LP文件 |

---

## 8. 构建与运行

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

## 9. 输入格式

```
第1行: 母板数量
第2行: 子件类型数量
第3行: 母板长度 <TAB> 母板宽度
第4行起: 子件长度 <TAB> 子件宽度 <TAB> 需求量 <TAB> 类型索引
```

---

## 10. 与其他项目的关系

| 项目 | 维度 | 子问题方法 | 分支策略 |
|------|------|------------|----------|
| CS-1D-BP | 1D | CPLEX IP | 变量分支 |
| CS-1D-BP-Arc | 1D | Arc Flow | Arc流量分支 |
| CS-1D-BP-Arc-DP | 1D | Arc Flow + DP | Arc流量分支 |
| CS-2D-BP | 2D | CPLEX IP | 变量分支 |
| **CS-2D-BP-Arc** | 2D | CPLEX IP + Arc Flow + DP | Arc流量分支 |

---

## 11. 参考文献

- Gilmore, P.C., Gomory, R.E. (1965). Multistage cutting stock problems of two and more dimensions. *Operations Research*.
- Valerio de Carvalho, J.M. (1999). Exact solution of bin-packing problems using column generation and branch-and-bound. *Annals of Operations Research*.
- Barnhart, C., et al. (1998). Branch-and-price: Column generation for solving huge integer programs. *Operations Research*.
