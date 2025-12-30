// output.cpp - 切割方案导出 (JSON格式)
//
// JSON文件包含完整的算例信息，可视化工具无需单独读取算例文件
// item_type 和 id 从1开始编号

#include "2DBP.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <map>

using namespace std;

// JSON辅助函数
static string JsonEscape(const string& str) {
    stringstream ss;
    for (char c : str) {
        switch (c) {
            case '"':  ss << "\\\""; break;
            case '\\': ss << "\\\\"; break;
            case '\n': ss << "\\n";  break;
            case '\r': ss << "\\r";  break;
            case '\t': ss << "\\t";  break;
            default:   ss << c;      break;
        }
    }
    return ss.str();
}

static string JsonDouble(double value, int precision = 4) {
    stringstream ss;
    ss << fixed << setprecision(precision) << value;
    return ss.str();
}

// 导出切割方案为JSON格式
void ExportSolution(ProblemParams& params, ProblemData& data) {
    filesystem::create_directories("results");

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

    fout << "{\n";

    // Metadata
    fout << "  \"metadata\": {\n";
    fout << "    \"instance_file\": \"" << JsonEscape(params.instance_file_) << "\",\n";
    fout << "    \"timestamp\": \"" << timestamp << "\",\n";
    fout << "    \"solver\": \"CS-2D-BP-Arc\"\n";
    fout << "  },\n";

    // Summary
    int num_plates = 0;
    for (const auto& y_col : params.global_best_y_cols_) {
        if (y_col.value_ > kZeroTolerance) {
            num_plates += static_cast<int>(round(y_col.value_));
        }
    }

    double total_item_area = 0;
    for (int i = 0; i < num_item_types; i++) {
        total_item_area += data.item_types_[i].width_ *
                          data.item_types_[i].length_ *
                          data.item_types_[i].demand_;
    }
    double total_stock_area = num_plates * stock_width * stock_length;
    double total_utilization = (total_stock_area > 0) ? (total_item_area / total_stock_area) : 0.0;

    double gap = 0.0;
    if (params.global_best_int_ > kZeroTolerance) {
        gap = (params.global_best_int_ - params.root_lb_) / params.global_best_int_;
    }

    fout << "  \"summary\": {\n";
    fout << "    \"num_plates\": " << num_plates << ",\n";
    fout << "    \"objective_value\": " << JsonDouble(params.global_best_int_) << ",\n";
    fout << "    \"root_lb\": " << JsonDouble(params.root_lb_) << ",\n";
    fout << "    \"gap\": " << JsonDouble(gap) << ",\n";
    fout << "    \"total_utilization\": " << JsonDouble(total_utilization) << "\n";
    fout << "  },\n";

    // Stock
    fout << "  \"stock\": {\n";
    fout << "    \"width\": " << stock_width << ",\n";
    fout << "    \"length\": " << stock_length << "\n";
    fout << "  },\n";

    // Item Types (从1开始编号)
    fout << "  \"item_types\": [\n";
    for (int i = 0; i < num_item_types; i++) {
        const auto& item = data.item_types_[i];
        fout << "    {\"id\": " << (i + 1)
             << ", \"width\": " << item.width_
             << ", \"length\": " << item.length_
             << ", \"demand\": " << item.demand_ << "}";
        if (i < num_item_types - 1) fout << ",";
        fout << "\n";
    }
    fout << "  ],\n";

    // Plates
    vector<vector<const XColumn*>> strip_x_cols(num_strip_types);
    for (const auto& x_col : params.global_best_x_cols_) {
        if (x_col.value_ > kZeroTolerance) {
            strip_x_cols[x_col.strip_type_id_].push_back(&x_col);
        }
    }

    struct PlateData {
        int plate_id;
        double utilization;
        vector<map<string, int>> items;
    };
    vector<PlateData> all_plates;

    int global_plate_id = 0;

    for (const auto& y_col : params.global_best_y_cols_) {
        if (y_col.value_ < kZeroTolerance) continue;

        int y_count = static_cast<int>(round(y_col.value_));

        for (int k = 0; k < y_count; k++) {
            global_plate_id++;

            PlateData plate;
            plate.plate_id = global_plate_id;

            vector<map<string, int>> items_json;
            int strip_y = 0;

            for (int j = 0; j < num_strip_types; j++) {
                int strip_count = y_col.pattern_[j];
                if (strip_count <= 0) continue;

                int strip_width = data.strip_types_[j].width_;

                for (int s = 0; s < strip_count; s++) {
                    const XColumn* x_col = nullptr;
                    for (auto& col : strip_x_cols[j]) {
                        if (col->value_ >= 1.0 - kZeroTolerance) {
                            x_col = col;
                            const_cast<XColumn*>(col)->value_ -= 1.0;
                            break;
                        }
                    }

                    int item_x = 0;

                    if (x_col != nullptr) {
                        for (int i = 0; i < num_item_types; i++) {
                            int item_count = x_col->pattern_[i];
                            if (item_count <= 0) continue;
                            if (data.item_types_[i].width_ != strip_width) continue;

                            int item_length = data.item_types_[i].length_;
                            int item_width = data.item_types_[i].width_;

                            for (int c = 0; c < item_count; c++) {
                                map<string, int> item;
                                item["item_type"] = i + 1;  // 从1开始
                                item["x"] = item_x;
                                item["y"] = strip_y;
                                item["width"] = item_width;
                                item["length"] = item_length;
                                items_json.push_back(item);
                                item_x += item_length;
                            }
                        }
                    }

                    strip_y += strip_width;
                }
            }

            // 按条带分组并按长度降序排序
            map<int, vector<map<string, int>>> strips_map;
            for (const auto& item : items_json) {
                strips_map[item.at("y")].push_back(item);
            }

            items_json.clear();
            for (auto& [y, strip_items] : strips_map) {
                sort(strip_items.begin(), strip_items.end(),
                     [](const map<string, int>& a, const map<string, int>& b) {
                         if (a.at("length") != b.at("length"))
                             return a.at("length") > b.at("length");
                         return a.at("width") > b.at("width");
                     });

                int x_pos = 0;
                for (auto& item : strip_items) {
                    item["x"] = x_pos;
                    x_pos += item.at("length");
                    items_json.push_back(item);
                }
            }

            // 计算利用率
            double plate_area = 0;
            for (const auto& item : items_json) {
                plate_area += item.at("width") * item.at("length");
            }
            plate.utilization = plate_area / (stock_width * stock_length);
            plate.items = items_json;
            all_plates.push_back(plate);
        }
    }

    // 按利用率降序排序
    sort(all_plates.begin(), all_plates.end(),
         [](const PlateData& a, const PlateData& b) {
             return a.utilization > b.utilization;
         });

    // 输出plates
    fout << "  \"plates\": [\n";
    for (size_t plate_idx = 0; plate_idx < all_plates.size(); plate_idx++) {
        const auto& plate = all_plates[plate_idx];

        if (plate_idx > 0) fout << ",\n";

        fout << "    {\n";
        fout << "      \"plate_id\": " << plate.plate_id << ",\n";
        fout << "      \"utilization\": " << JsonDouble(plate.utilization) << ",\n";
        fout << "      \"num_items\": " << plate.items.size() << ",\n";
        fout << "      \"items\": [\n";

        for (size_t idx = 0; idx < plate.items.size(); idx++) {
            const auto& item = plate.items[idx];
            fout << "        {\"item_type\": " << item.at("item_type")
                 << ", \"x\": " << item.at("x")
                 << ", \"y\": " << item.at("y")
                 << ", \"width\": " << item.at("width")
                 << ", \"length\": " << item.at("length") << "}";
            if (idx < plate.items.size() - 1) fout << ",";
            fout << "\n";
        }

        fout << "      ]\n";
        fout << "    }";
    }

    fout << "\n  ]\n";
    fout << "}\n";

    fout.close();
    LOG_FMT("[导出] 导出完成, 共 %d 块母板\n", global_plate_id);
}

void ExportResults(ProblemParams& params, ProblemData& data) {
    ExportSolution(params, data);
}
