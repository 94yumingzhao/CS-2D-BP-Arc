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
        params.num_stocks_ = std::stoi(tokens[0]);

        // ---------------------------------------------------------------------
        // 读取第 2 行: 子件类型数量
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, tokens, "\t");
        params.num_item_types_ = std::stoi(tokens[0]);

        // 条带类型数 = 子件类型数
        params.num_strip_types_ = params.num_item_types_;

        // ---------------------------------------------------------------------
        // 读取第 3 行: 母板尺寸
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, tokens, "\t");
        params.stock_length_ = std::stoi(tokens[0]);
        params.stock_width_ = std::stoi(tokens[1]);

        cout << "[数据] 母板尺寸: " << params.stock_length_ << " x " << params.stock_width_ << "\n";
        cout << "[数据] 母板数量: " << params.num_stocks_ << "\n";
        cout << "[数据] 子件类型数: " << params.num_item_types_ << "\n";

        // ---------------------------------------------------------------------
        // 初始化母板列表
        // ---------------------------------------------------------------------
        data.stocks_.reserve(params.num_stocks_);
        for (int i = 0; i < params.num_stocks_; i++) {
            Stock stock;
            stock.length_ = params.stock_length_;
            stock.width_ = params.stock_width_;
            stock.area_ = params.stock_length_ * params.stock_width_;
            stock.x_ = 0;
            stock.y_ = 0;

            data.stocks_.push_back(stock);
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

            int type_demand = std::stoi(tokens[2]);
            total_demand += type_demand;

            // 预解析常用值避免重复转换
            int parsed_type_id = std::stoi(tokens[3]);
            int parsed_length = std::stoi(tokens[0]);
            int parsed_width = std::stoi(tokens[1]);

            // -----------------------------------------------------------------
            // 按需求量展开为独立子件实例
            // -----------------------------------------------------------------
            for (int k = 0; k < type_demand; k++) {
                Item item;

                item.type_id_ = parsed_type_id;
                item.id_ = item_id;
                item.demand_ = type_demand;
                item.length_ = parsed_length;
                item.width_ = parsed_width;
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
            item_type.type_id_ = parsed_type_id;
            item_type.demand_ = type_demand;
            item_type.width_ = parsed_width;
            item_type.length_ = parsed_length;

            data.item_types_.push_back(item_type);
        }

        cout << "[数据] 子件总需求: " << total_demand << "\n";

        fin.close();

    } else {
        cerr << "[错误] 无法打开文件: " << file_path << "\n";
        cerr << "[错误] 程序终止\n";
        exit(1);
    }

    // =========================================================================
    // 按宽度降序排序所有子件
    // =========================================================================
    std::sort(data.items_.begin(), data.items_.end(),
        [](const Item& a, const Item& b) {
            return a.width_ > b.width_;
        });

    cout << "[数据] 数据读取完成, 子件已按宽度降序排序\n";
}
