// output.cpp - 切割方案导出（JSON格式）
//
// 将求解结果导出为JSON格式，方便机器解析和可视化工具读取
//
// JSON格式:
// {
//   "metadata": {
//     "instance_file": "...",
//     "timestamp": "...",
//     "solver": "CS-2D-BP-Arc"
//   },
//   "summary": {
//     "num_plates": 2,
//     "objective_value": 2.0,
//     "root_lb": 0.2675,
//     "gap": 0.866,
//     "total_utilization": 0.875
//   },
//   "stock": {
//     "width": 200,
//     "length": 400
//   },
//   "plates": [
//     {
//       "plate_id": 1,
//       "utilization": 0.9,
//       "num_items": 5,
//       "items": [
//         {
//           "item_type": 1,
//           "x": 0,
//           "y": 0,
//           "width": 40,
//           "length": 80
//         },
//         ...
//       ]
//     },
//     ...
//   ]
// }

#include "2DBP.h"
#include <sstream>
#include <iomanip>

using namespace std;

// JSON辅助函数：转义字符串
string JsonEscape(const string& str) {
    stringstream ss;
    for (char c : str) {
        switch (c) {
            case '"':  ss << "\\\""; break;
            case '\\': ss << "\\\\"; break;
            case '\b': ss << "\\b";  break;
            case '\f': ss << "\\f";  break;
            case '\n': ss << "\\n";  break;
            case '\r': ss << "\\r";  break;
            case '\t': ss << "\\t";  break;
            default:   ss << c;      break;
        }
    }
    return ss.str();
}

// JSON辅助函数：格式化浮点数
string JsonDouble(double value, int precision = 4) {
    stringstream ss;
    ss << fixed << setprecision(precision) << value;
    return ss.str();
}

// 计算单个母板的利用率
double CalculatePlateUtilization(const vector<pair<int, int>>& items,
                                   const ProblemData& data,
                                   int stock_width, int stock_length) {
    double total_area = 0;
    for (const auto& [item_type, count] : items) {
        int item_width = data.item_types_[item_type].width_;
        int item_length = data.item_types_[item_type].length_;
        total_area += item_width * item_length * count;
    }
    double stock_area = stock_width * stock_length;
    return (stock_area > 0) ? (total_area / stock_area) : 0.0;
}

// 导出切割方案为JSON格式
void ExportSolution(ProblemParams& params, ProblemData& data) {
    // 创建输出目录
    filesystem::create_directories("results");

    // 输出文件命名: results/solution_YYYYMMDD_HHMMSS.json
    string timestamp = GetTimestampString();
    string output_file = "results/solution_" + timestamp + ".json";
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

    // 开始构建JSON
    fout << "{\n";

    // =========================================================================
    // Metadata部分
    // =========================================================================
    fout << "  \"metadata\": {\n";
    fout << "    \"instance_file\": \"" << JsonEscape(params.instance_file_) << "\",\n";
    fout << "    \"timestamp\": \"" << timestamp << "\",\n";
    fout << "    \"solver\": \"CS-2D-BP-Arc\"\n";
    fout << "  },\n";

    // =========================================================================
    // Summary部分
    // =========================================================================

    // 统计母板数量
    int num_plates = 0;
    for (const auto& y_col : params.global_best_y_cols_) {
        if (y_col.value_ > kZeroTolerance) {
            num_plates += static_cast<int>(round(y_col.value_));
        }
    }

    // 计算总利用率
    double total_item_area = 0;
    for (int i = 0; i < num_item_types; i++) {
        int width = data.item_types_[i].width_;
        int length = data.item_types_[i].length_;
        int demand = data.item_types_[i].demand_;
        total_item_area += width * length * demand;
    }
    double total_stock_area = num_plates * stock_width * stock_length;
    double total_utilization = (total_stock_area > 0) ? (total_item_area / total_stock_area) : 0.0;

    fout << "  \"summary\": {\n";
    fout << "    \"num_plates\": " << num_plates << ",\n";
    fout << "    \"objective_value\": " << JsonDouble(params.global_best_int_) << ",\n";
    fout << "    \"root_lb\": " << JsonDouble(params.root_lb_) << ",\n";

    double gap = 0.0;
    if (params.global_best_int_ > kZeroTolerance) {
        gap = (params.global_best_int_ - params.root_lb_) / params.global_best_int_;
    }
    fout << "    \"gap\": " << JsonDouble(gap) << ",\n";
    fout << "    \"total_utilization\": " << JsonDouble(total_utilization) << "\n";
    fout << "  },\n";

    // =========================================================================
    // Stock信息
    // =========================================================================
    fout << "  \"stock\": {\n";
    fout << "    \"width\": " << stock_width << ",\n";
    fout << "    \"length\": " << stock_length << "\n";
    fout << "  },\n";

    // =========================================================================
    // Plates部分
    // =========================================================================
    fout << "  \"plates\": [\n";

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

    int global_plate_id = 0;  // 全局母板编号
    bool first_plate = true;

    // 遍历每个使用的 Y 列 (母板切割方案)
    for (const auto& y_col : params.global_best_y_cols_) {
        if (y_col.value_ < kZeroTolerance) continue;

        int y_count = static_cast<int>(round(y_col.value_));

        // 对该 Y 列方案的每块母板
        for (int k = 0; k < y_count; k++) {
            global_plate_id++;

            if (!first_plate) {
                fout << ",\n";
            }
            first_plate = false;

            // 开始这块母板的JSON对象
            fout << "    {\n";
            fout << "      \"plate_id\": " << global_plate_id << ",\n";

            // 收集该母板上的所有子板信息
            vector<map<string, string>> items_json;
            int item_x = 0;  // X坐标（沿长度方向）
            int strip_y = 0;  // Y坐标（沿宽度方向）

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

                    item_x = 0;  // 重置X坐标

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
                                // 记录子板信息
                                map<string, string> item;
                                item["item_type"] = to_string(i);
                                item["x"] = to_string(item_x);
                                item["y"] = to_string(strip_y);
                                item["width"] = to_string(item_width);
                                item["length"] = to_string(item_length);
                                items_json.push_back(item);

                                item_x += item_length;
                            }
                        }
                    }

                    strip_y += strip_width;
                }
            }

            // 计算该母板的利用率
            vector<pair<int, int>> plate_items;
            for (const auto& item : items_json) {
                int item_type = stoi(item.at("item_type"));
                plate_items.push_back({item_type, 1});
            }
            double utilization = CalculatePlateUtilization(plate_items, data, stock_width, stock_length);

            fout << "      \"utilization\": " << JsonDouble(utilization) << ",\n";
            fout << "      \"num_items\": " << items_json.size() << ",\n";
            fout << "      \"items\": [\n";

            // 输出子板列表
            for (size_t idx = 0; idx < items_json.size(); idx++) {
                const auto& item = items_json[idx];
                fout << "        {\n";
                fout << "          \"item_type\": " << item.at("item_type") << ",\n";
                fout << "          \"x\": " << item.at("x") << ",\n";
                fout << "          \"y\": " << item.at("y") << ",\n";
                fout << "          \"width\": " << item.at("width") << ",\n";
                fout << "          \"length\": " << item.at("length") << "\n";
                fout << "        }";
                if (idx < items_json.size() - 1) {
                    fout << ",";
                }
                fout << "\n";
            }

            fout << "      ]\n";
            fout << "    }";
        }
    }

    fout << "\n  ]\n";
    fout << "}\n";

    fout.close();
    LOG_FMT("[导出] 导出完成, 共 %d 块母板\n", global_plate_id);
}

// 导出结果汇总
void ExportResults(ProblemParams& params, ProblemData& data) {
    ExportSolution(params, data);
}
