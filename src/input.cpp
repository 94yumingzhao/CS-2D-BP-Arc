// =============================================================================
// input.cpp - 数据读取模块
// =============================================================================
//
// 功能: 从文件读取二维下料问题的输入数据
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// SplitString - 字符串分割函数
// =============================================================================
void SplitString(const string& str, vector<string>& result, const string& delimiter) {

    string::size_type start_pos, delim_pos;

    delim_pos = str.find(delimiter);
    start_pos = 0;

    result.clear();

    while (string::npos != delim_pos) {
        result.push_back(str.substr(start_pos, delim_pos - start_pos));
        start_pos = delim_pos + delimiter.size();
        delim_pos = str.find(delimiter, start_pos);
    }

    if (start_pos != str.length()) {
        result.push_back(str.substr(start_pos));
    }
}


// =============================================================================
// LoadInput - 读取问题数据
// =============================================================================
void LoadInput(ProblemParams& params, ProblemData& data) {

    ostringstream path_stream;
    string file_path;
    string line;
    vector<string> tokens;

    // =========================================================================
    // 设置输入文件路径
    // =========================================================================
    path_stream.str("");
    path_stream << "C:/Users/zym60/OneDrive/2DCSP/2DCG/cutdata1207.txt";
    file_path = path_stream.str();

    cout << "[数据] 读取文件: " << file_path << "\n";

    // =========================================================================
    // 打开并读取文件
    // =========================================================================
    ifstream fin(file_path.c_str());

    if (fin) {

        // ---------------------------------------------------------------------
        // 读取第 1 行: 母板数量
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, tokens, "\t");
        params.num_stocks_ = atoi(tokens[0].c_str());

        // ---------------------------------------------------------------------
        // 读取第 2 行: 子件类型数量
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, tokens, "\t");
        params.num_item_types_ = atoi(tokens[0].c_str());

        // 条带类型数 = 子件类型数
        params.num_strip_types_ = params.num_item_types_;

        // ---------------------------------------------------------------------
        // 读取第 3 行: 母板尺寸
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, tokens, "\t");
        params.stock_length_ = atoi(tokens[0].c_str());
        params.stock_width_ = atoi(tokens[1].c_str());

        cout << "[数据] 母板尺寸: " << params.stock_length_ << " x " << params.stock_width_ << "\n";
        cout << "[数据] 母板数量: " << params.num_stocks_ << "\n";
        cout << "[数据] 子件类型数: " << params.num_item_types_ << "\n";

        // ---------------------------------------------------------------------
        // 初始化母板列表
        // ---------------------------------------------------------------------
        for (int i = 0; i < params.num_stocks_; i++) {
            Stock stock;
            stock.length_ = params.stock_length_;
            stock.width_ = params.stock_width_;
            stock.area_ = params.stock_length_ * params.stock_width_;
            stock.x_ = 0;
            stock.y_ = 0;

            data.stocks_.insert(data.stocks_.begin(), stock);
        }

        // ---------------------------------------------------------------------
        // 读取子件数据 (第 4 行起)
        // ---------------------------------------------------------------------
        int item_id = 1;
        int num_item_types = params.num_item_types_;
        int total_demand = 0;

        for (int i = 0; i < num_item_types; i++) {
            getline(fin, line);
            SplitString(line, tokens, "\t");

            int type_demand = atoi(tokens[2].c_str());
            total_demand += type_demand;

            // -----------------------------------------------------------------
            // 按需求量展开为独立子件实例
            // -----------------------------------------------------------------
            for (int k = 0; k < type_demand; k++) {
                Item item;

                item.type_id_ = atoi(tokens[3].c_str());
                item.id_ = item_id;
                item.demand_ = atoi(tokens[2].c_str());
                item.length_ = atoi(tokens[0].c_str());
                item.width_ = atoi(tokens[1].c_str());
                item.area_ = item.length_ * item.width_;

                item.x_ = -1;
                item.y_ = -1;
                item.stock_id_ = -1;
                item.assign_flag_ = 0;

                data.items_.push_back(item);

                item_id++;
                params.num_items_++;
            }

            // -----------------------------------------------------------------
            // 创建子件类型记录
            // -----------------------------------------------------------------
            ItemType item_type;
            item_type.type_id_ = atoi(tokens[3].c_str());
            item_type.demand_ = atoi(tokens[2].c_str());
            item_type.width_ = atoi(tokens[1].c_str());
            item_type.length_ = atoi(tokens[0].c_str());

            data.item_types_.push_back(item_type);
        }

        cout << "[数据] 子件总需求: " << total_demand << "\n";

        fin.close();

    } else {
        cout << "[错误] 无法打开文件: " << file_path << "\n";
    }

    // =========================================================================
    // 按宽度降序排序所有子件
    // =========================================================================
    Item temp_item;
    int num_items = data.items_.size();

    for (int i = 0; i < num_items - 1; i++) {
        for (int j = i + 1; j < num_items; j++) {
            if (data.items_[i].width_ < data.items_[j].width_) {
                temp_item = data.items_[i];
                data.items_[i] = data.items_[j];
                data.items_[j] = temp_item;
            }
        }
    }

    cout << "[数据] 数据读取完成, 子件已按宽度降序排序\n";
}
