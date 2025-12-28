// output.cpp - 切割方案导出
//
// 将求解结果导出为 CS-2D-Fig 可视化工具可读取的格式
//
// 输出格式 (每个矩形 4 行):
//   X1\tY1\tID   (左下角)
//   X2\tY2\tID   (左上角)
//   X3\tY3\tID   (右上角)
//   X4\tY4\tID   (右下角)

#include "2DBP.h"

using namespace std;

// 导出单个矩形的 4 个顶点
// 参数: x, y - 左下角坐标; w, h - 宽度和高度; id - 标识; fout - 输出流
void ExportRectangle(int x, int y, int w, int h, int id, ofstream& fout) {
    // 逆时针顺序: 左下 -> 左上 -> 右上 -> 右下
    fout << x << "\t" << y << "\t" << id << "\n";           // 左下
    fout << x << "\t" << (y + h) << "\t" << id << "\n";     // 左上
    fout << (x + w) << "\t" << (y + h) << "\t" << id << "\n"; // 右上
    fout << (x + w) << "\t" << y << "\t" << id << "\n";     // 右下
}

// 导出切割方案到文件
// 生成每块母板的切割坐标, 供 CS-2D-Fig 可视化
void ExportSolution(ProblemParams& params, ProblemData& data) {
    // 创建输出目录
    filesystem::create_directories("results");

    // 输出文件命名: results/solution_YYYYMMDD_HHMMSS.txt
    string output_file = "results/solution_" + GetTimestampString() + ".txt";
    ofstream fout(output_file);

    if (!fout) {
        LOG_FMT("[错误] 无法创建输出文件: %s\n", output_file.c_str());
        return;
    }

    LOG_FMT("[导出] 输出文件: %s\n", output_file.c_str());

    int stock_length = params.stock_length_;
    int stock_width = params.stock_width_;
    int num_strip_types = params.num_strip_types_;
    int num_item_types = params.num_item_types_;

    // 统计每种条带类型的总需求 (从所有 Y 列累加)
    vector<int> strip_demand(num_strip_types, 0);
    for (const auto& y_col : params.global_best_y_cols_) {
        if (y_col.value_ > kZeroTolerance) {
            int y_count = static_cast<int>(round(y_col.value_));
            for (int j = 0; j < num_strip_types; j++) {
                strip_demand[j] += y_count * y_col.pattern_[j];
            }
        }
    }

    // 为每种条带类型收集对应的 X 列
    vector<vector<const XColumn*>> strip_x_cols(num_strip_types);
    for (const auto& x_col : params.global_best_x_cols_) {
        if (x_col.value_ > kZeroTolerance) {
            strip_x_cols[x_col.strip_type_id_].push_back(&x_col);
        }
    }

    int stock_id = 0;  // 母板编号

    // 遍历每个使用的 Y 列 (母板切割方案)
    for (const auto& y_col : params.global_best_y_cols_) {
        if (y_col.value_ < kZeroTolerance) continue;

        int y_count = static_cast<int>(round(y_col.value_));

        // 对该 Y 列方案的每块母板
        for (int k = 0; k < y_count; k++) {
            stock_id++;

            // 先输出母板边框 (ID = 0)
            ExportRectangle(0, 0, stock_length, stock_width, 0, fout);

            int strip_y = 0;  // 当前条带的 Y 坐标 (从下往上)

            // 遍历该 Y 列中的每种条带类型
            for (int j = 0; j < num_strip_types; j++) {
                int strip_count = y_col.pattern_[j];
                if (strip_count <= 0) continue;

                int strip_width = data.strip_types_[j].width_;

                // 对该类型的每个条带
                for (int s = 0; s < strip_count; s++) {
                    // 找到一个 X 列来切割该条带
                    const XColumn* x_col = nullptr;
                    for (auto& col : strip_x_cols[j]) {
                        if (col->value_ >= 1.0 - kZeroTolerance) {
                            x_col = col;
                            // 减少该 X 列的可用次数
                            const_cast<XColumn*>(col)->value_ -= 1.0;
                            break;
                        }
                    }

                    int item_x = 0;  // 当前子板的 X 坐标 (从左往右)

                    if (x_col != nullptr) {
                        // 按 X 列的 pattern 放置子板
                        for (int i = 0; i < num_item_types; i++) {
                            int item_count = x_col->pattern_[i];
                            if (item_count <= 0) continue;

                            // 只放置宽度匹配的子板
                            if (data.item_types_[i].width_ != strip_width) continue;

                            int item_length = data.item_types_[i].length_;
                            int item_width = data.item_types_[i].width_;

                            for (int c = 0; c < item_count; c++) {
                                // 输出子板矩形 (ID = 子板类型编号 + 1)
                                ExportRectangle(item_x, strip_y,
                                    item_length, item_width,
                                    i + 1, fout);
                                item_x += item_length;
                            }
                        }
                    }

                    strip_y += strip_width;
                }
            }

            // 母板之间空一行
            fout << "\n";
        }
    }

    fout.close();
    LOG_FMT("[导出] 导出完成, 共 %d 块母板\n", stock_id);
}

// 导出结果汇总 (可选)
void ExportResults(ProblemParams& params, ProblemData& data) {
    ExportSolution(params, data);
}
