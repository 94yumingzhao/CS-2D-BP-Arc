# =============================================================================
# CS-2D-BP-Arc - 二维下料问题求解器 (Arc Flow分支定价)
# CMake配置文件
# =============================================================================

cmake_minimum_required(VERSION 3.24)

project(CS-2D-BP-Arc
    VERSION 1.0.0
    DESCRIPTION "Two-Dimensional Cutting Stock Problem Solver using Arc Flow Branch and Price"
    LANGUAGES CXX
)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 多配置生成器输出目录
foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${CONFIG} CONFIG_UPPER)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${CMAKE_BINARY_DIR}/bin/${CONFIG})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib/${CONFIG})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib/${CONFIG})
endforeach()

# CPLEX配置
set(CPLEX_DIR "D:/CPLEX" CACHE PATH "CPLEX installation directory")
if(NOT EXISTS ${CPLEX_DIR})
    message(WARNING "CPLEX directory not found at ${CPLEX_DIR}. Please set CPLEX_DIR correctly.")
endif()

# 源文件目录
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# 包含目录
include_directories(
    ${SRC_DIR}
    "${CPLEX_DIR}/cplex/include"
    "${CPLEX_DIR}/concert/include"
)

# 定义宏
add_definitions(-DIL_STD)

# MSVC运行时库
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
endif()

# 库目录
link_directories(
    "${CPLEX_DIR}/cplex/lib/x64_windows_msvc14/stat_mda"
    "${CPLEX_DIR}/concert/lib/x64_windows_msvc14/stat_mda"
)

# 源文件
set(SOURCES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/logger.cpp
    ${SRC_DIR}/input.cpp
    ${SRC_DIR}/output.cpp
    ${SRC_DIR}/heuristic.cpp
    ${SRC_DIR}/arc_flow.cpp
    ${SRC_DIR}/root_node.cpp
    ${SRC_DIR}/root_node_sub.cpp
    ${SRC_DIR}/column_generation.cpp
    ${SRC_DIR}/new_node.cpp
    ${SRC_DIR}/new_node_sub.cpp
    ${SRC_DIR}/branch_and_price.cpp
)

# 头文件
set(HEADERS
    ${SRC_DIR}/2DBP.h
    ${SRC_DIR}/logger.h
)

# IDE中平级显示文件
source_group("Source Files" FILES ${SOURCES})
source_group("Header Files" FILES ${HEADERS})

# 创建可执行文件
add_executable(CS-2D-BP-Arc ${SOURCES} ${HEADERS})

# 目标属性
set_target_properties(CS-2D-BP-Arc PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
)

# 链接CPLEX库
target_link_libraries(CS-2D-BP-Arc PRIVATE
    cplex2210
    ilocplex
    concert
)

# 编译选项
if(MSVC)
    target_compile_options(CS-2D-BP-Arc PRIVATE
        /W3
        /EHsc
        /bigobj
    )
endif()

# 运行目标
add_custom_target(run
    COMMAND CS-2D-BP-Arc
    DEPENDS CS-2D-BP-Arc
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running CS-2D-BP-Arc..."
)

# 构建信息
message(STATUS "")
message(STATUS "=== CS-2D-BP-Arc Build Configuration ===")
message(STATUS "CMake Version: ${CMAKE_VERSION}")
message(STATUS "Project Version: ${PROJECT_VERSION}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CPLEX Directory: ${CPLEX_DIR}")
message(STATUS "Source Directory: ${SRC_DIR}")
message(STATUS "Output Directory: ${CMAKE_BINARY_DIR}")
message(STATUS "=========================================")
message(STATUS "")
