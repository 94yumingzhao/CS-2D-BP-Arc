# =============================================================================
# 2DBP: 二维下料问题求解器 (分支定价算法)
# 2DBP: Two-Dimensional Bin Packing Problem Solver (Branch and Price)
# =============================================================================

cmake_minimum_required(VERSION 3.24)
project(2DBP VERSION 1.0.0 LANGUAGES CXX)

# =============================================================================
# C++ 标准设置
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# CPLEX 路径配置
# =============================================================================
set(CPLEX_ROOT "D:/CPLEX")
set(CPLEX_INCLUDE_DIR "${CPLEX_ROOT}/cplex/include")
set(CONCERT_INCLUDE_DIR "${CPLEX_ROOT}/concert/include")
set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x64_windows_msvc14/stat_mda")
set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x64_windows_msvc14/stat_mda")

# =============================================================================
# 源文件
# =============================================================================
set(SOURCES
    src/main.cpp
    src/input.cpp
    src/primal_heuristic.cpp
    src/root_node_column_generation.cpp
    src/root_node_first_master_problem.cpp
    src/update_master_problem.cpp
    src/sub_problem.cpp
    src/branch_and_price.cpp
    src/branching.cpp
    src/new_node_column_generation.cpp
    src/new_node_first_master_problem.cpp
    src/new_node_generating.cpp
    src/output_models.cpp
    src/output_results.cpp
)

# =============================================================================
# 可执行文件
# =============================================================================
add_executable(${PROJECT_NAME} ${SOURCES})

# =============================================================================
# 头文件目录
# =============================================================================
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CPLEX_INCLUDE_DIR}
    ${CONCERT_INCLUDE_DIR}
)

# =============================================================================
# 预处理器定义
# =============================================================================
target_compile_definitions(${PROJECT_NAME} PRIVATE
    IL_STD
    _CRT_SECURE_NO_WARNINGS
)

# =============================================================================
# 链接库
# =============================================================================
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${CPLEX_LIB_DIR}/cplex2210.lib
    ${CPLEX_LIB_DIR}/ilocplex.lib
    ${CONCERT_LIB_DIR}/concert.lib
)

# =============================================================================
# 编译选项
# =============================================================================
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W3
        /EHsc
        /bigobj
    )
endif()

# =============================================================================
# 输出目录
# =============================================================================
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
