// input.cpp - 数据读取与辅助函数
//
// 本文件实现数据读取和各类辅助函数:
// - LoadInput: 从文件读取问题实例 (OR标准CSV格式)
// - Build*Index: 构建索引映射表
// - Print*: 调试输出函数
//
// 数据文件格式 (OR标准):
//   # 开头的行为注释
//   stock_width,stock_length  <- 表头 (W=宽度, L=长度)
//   200,400                   <- 母板尺寸
//   id,width,length,demand    <- 表头 (w_i, l_i, d_i)
//   0,50,80,3                 <- 子板数据
//   ...

#include "2DBP.h"

using namespace std;

// 字符串分割函数
// 功能: 按分隔符将字符串拆分为子串数组
// 参数: str-输入字符串, delimiter-分隔符
// 输出: result-分割后的子串数组
void SplitString(const string& str, vector<string>& result, const string& delimiter) {
    string::size_type start_pos = 0;
    string::size_type delim_pos = str.find(delimiter);
    result.clear();

    // 循环查找分隔符并提取子串
    while (string::npos != delim_pos) {
        result.push_back(str.substr(start_pos, delim_pos - start_pos));
        start_pos = delim_pos + delimiter.size();
        delim_pos = str.find(delimiter, start_pos);
    }

    // 添加最后一个子串
    if (start_pos != str.length()) {
        result.push_back(str.substr(start_pos));
    }
}

// 判断是否为注释行或空行
bool IsCommentOrEmpty(const string& line) {
    if (line.empty()) return true;
    size_t pos = line.find_first_not_of(" \t\r\n");
    if (pos == string::npos) return true;
    return line[pos] == '#';
}

// 判断是否为表头行 (包含字母)
bool IsHeaderLine(const string& line) {
    for (char c : line) {
        if (isalpha(c) && c != ',') return true;
    }
    return false;
}

// 获取目录中最新的算例文件
// 按文件名排序选择最后一个 (时间戳在前的命名方式)
string GetLatestInstanceFile(const string& data_dir) {
    namespace fs = filesystem;

    if (!fs::exists(data_dir) || !fs::is_directory(data_dir)) {
        LOG_FMT("[错误] 目录不存在: %s\n", data_dir.c_str());
        return "";
    }

    vector<string> csv_files;
    for (const auto& entry : fs::directory_iterator(data_dir)) {
        if (entry.is_regular_file()) {
            string filename = entry.path().filename().string();
            // 匹配以 kFilePattern 开头的 csv 文件
            if (filename.find(kFilePattern) == 0 &&
                filename.size() > 4 &&
                filename.substr(filename.size() - 4) == ".csv") {
                csv_files.push_back(entry.path().string());
            }
        }
    }

    if (csv_files.empty()) {
        LOG_FMT("[错误] 目录中无算例文件: %s\n", data_dir.c_str());
        return "";
    }

    // 按文件名排序, 时间戳在前所以最后一个是最新的
    sort(csv_files.begin(), csv_files.end());
    return csv_files.back();
}

// 读取问题数据 (2DPackLib CSV 格式)
// 功能: 从 CSV 文件读取二维下料问题实例
// 输入:
//   - specified_file: 指定算例文件路径 (空字符串则自动选择最新文件)
// 输出:
//   - params: 问题参数 (尺寸, 数量等)
//   - data: 问题数据 (子板类型, 条带类型等)
// 返回值: (状态码, 子板类型数, 条带类型数)
//   状态码: 0=成功, -1=失败
tuple<int, int, int> LoadInput(ProblemParams& params, ProblemData& data,
                                const string& specified_file) {
    string line;
    vector<string> tokens;

    // 确定要读取的文件路径
    string file_path;
    if (!specified_file.empty()) {
        // 使用指定的文件
        file_path = specified_file;
        LOG_FMT("[数据] 使用指定文件: %s\n", file_path.c_str());
    } else {
        // 自动选择最新的算例文件
        file_path = GetLatestInstanceFile(kDataDir);
        if (file_path.empty()) {
            LOG("[错误] 未找到算例文件");
            return make_tuple(-1, 0, 0);
        }
        LOG_FMT("[数据] 使用最新文件: %s\n", file_path.c_str());
    }

    // 保存算例文件名到params（用于输出JSON）
    params.instance_file_ = file_path;

    ifstream fin(file_path.c_str());
    if (!fin) {
        LOG_FMT("[错误] 无法打开文件: %s\n", file_path.c_str());
        return make_tuple(-1, 0, 0);
    }

    bool stock_read = false;
    int total_demand = 0;
    set<int> unique_widths;

    while (getline(fin, line)) {
        // 跳过注释行和空行
        if (IsCommentOrEmpty(line)) continue;
        // 跳过表头行
        if (IsHeaderLine(line)) continue;

        // 解析 CSV 数据
        SplitString(line, tokens, ",");
        if (tokens.empty()) continue;

        if (!stock_read) {
            // 第一个数据行: 母板尺寸 (width, length)
            // OR标准: width=W(切割方向), length=L(条带延伸方向)
            params.stock_width_ = stoi(tokens[0]);   // W: 宽度
            params.stock_length_ = stoi(tokens[1]);  // L: 长度
            stock_read = true;
            LOG_FMT("[数据] 母板尺寸: W=%d x L=%d\n",
                params.stock_width_, params.stock_length_);
        } else {
            // 子板数据行: id, width, length, demand
            // OR标准: w_i(宽度), l_i(长度), d_i(需求)
            ItemType item_type;
            item_type.type_id_ = stoi(tokens[0]);
            item_type.width_ = stoi(tokens[1]);    // w_i: 宽度
            item_type.length_ = stoi(tokens[2]);   // l_i: 长度
            item_type.demand_ = stoi(tokens[3]);

            data.item_types_.push_back(item_type);
            total_demand += item_type.demand_;
            unique_widths.insert(item_type.width_);
        }
    }
    fin.close();

    params.num_item_types_ = static_cast<int>(data.item_types_.size());
    LOG_FMT("[数据] 子板类型数: %d\n", params.num_item_types_);

    // 条带类型数量 = 不同宽度的数量
    // 因为同宽度的子板可以放在同一条带上切割
    params.num_strip_types_ = static_cast<int>(unique_widths.size());
    params.num_items_ = total_demand;

    LOG_FMT("[数据] 子板总需求: %d\n", total_demand);
    LOG_FMT("[数据] 条带类型数: %d\n", params.num_strip_types_);

    // 创建条带类型 (按宽度降序排列)
    // 宽度大的条带能容纳更多子板类型
    vector<int> widths(unique_widths.begin(), unique_widths.end());
    sort(widths.begin(), widths.end(), greater<int>());

    for (int i = 0; i < params.num_strip_types_; i++) {
        StripType strip_type;
        strip_type.type_id_ = i;
        strip_type.width_ = widths[i];
        strip_type.length_ = params.stock_length_;  // 条带长度 = 母板长度
        data.strip_types_.push_back(strip_type);
        data.strip_widths_.push_back(widths[i]);
    }

    // 构建索引映射 (用于快速查找)
    BuildLengthIndex(data);
    BuildWidthIndex(data);

    LOG("[数据] 数据读取完成");
    return make_tuple(0, params.num_item_types_, params.num_strip_types_);
}

// 构建长度到子板类型的索引
// 用于Arc Flow中根据Arc长度快速查找对应的子板类型
void BuildLengthIndex(ProblemData& data) {
    data.item_lengths_.clear();
    data.length_to_item_index_.clear();

    for (int i = 0; i < (int)data.item_types_.size(); i++) {
        int len = data.item_types_[i].length_;
        data.length_to_item_index_[len] = i;  // 长度 -> 子板索引
        data.item_lengths_.push_back(len);
    }

    // 按降序排序 (用于动态规划优化)
    sort(data.item_lengths_.begin(), data.item_lengths_.end(), greater<int>());
}

// 构建宽度到条带/子板类型的索引
// 用于Arc Flow中根据Arc宽度快速查找对应的条带类型
void BuildWidthIndex(ProblemData& data) {
    data.width_to_strip_index_.clear();
    data.width_to_item_indices_.clear();

    // 宽度 -> 条带类型索引
    for (int i = 0; i < (int)data.strip_types_.size(); i++) {
        int wid = data.strip_types_[i].width_;
        data.width_to_strip_index_[wid] = i;
    }

    // 宽度 -> 子板类型索引列表 (同宽度可能有多种子板)
    for (int i = 0; i < (int)data.item_types_.size(); i++) {
        int wid = data.item_types_[i].width_;
        data.width_to_item_indices_[wid].push_back(i);
    }
}

// 打印问题参数
void PrintParams(ProblemParams& params) {
    LOG("=== 问题参数 ===");
    LOG_FMT("  母板尺寸: %d x %d\n", params.stock_length_, params.stock_width_);
    LOG_FMT("  子板类型数: %d\n", params.num_item_types_);
    LOG_FMT("  条带类型数: %d\n", params.num_strip_types_);
    LOG_FMT("  SP1方法: %d\n", params.sp1_method_);
    LOG_FMT("  SP2方法: %d\n", params.sp2_method_);
}

// 打印需求信息
void PrintDemand(ProblemData& data) {
    LOG("=== 子板需求 ===");
    for (int i = 0; i < (int)data.item_types_.size(); i++) {
        LOG_FMT("  类型%d: %dx%d 需求=%d\n",
            i + 1,
            data.item_types_[i].length_,
            data.item_types_[i].width_,
            data.item_types_[i].demand_);
    }
}

// 打印初始矩阵 (启发式生成的初始列)
void PrintInitMatrix(ProblemParams& params) {
    LOG("=== 初始Y列矩阵 ===");
    for (int i = 0; i < (int)params.init_y_matrix_.size(); i++) {
        ostringstream oss;
        oss << "  Y" << (i + 1) << ": [";
        for (int j = 0; j < (int)params.init_y_matrix_[i].size(); j++) {
            if (j > 0) oss << ", ";
            oss << params.init_y_matrix_[i][j];
        }
        oss << "]";
        LOG(oss.str().c_str());
    }

    LOG("=== 初始X列矩阵 ===");
    for (int i = 0; i < (int)params.init_x_matrix_.size(); i++) {
        ostringstream oss;
        oss << "  X" << (i + 1) << ": [";
        for (int j = 0; j < (int)params.init_x_matrix_[i].size(); j++) {
            if (j > 0) oss << ", ";
            oss << params.init_x_matrix_[i][j];
        }
        oss << "]";
        LOG(oss.str().c_str());
    }
}

// 打印列生成解
void PrintCGSolution(BPNode* node, ProblemData& data) {
    LOG("=== 列生成解 ===");
    LOG_FMT("  目标值: %.4f\n", node->solution_.obj_val_);

    LOG("  Y列 (母板切割方案):");
    for (int i = 0; i < (int)node->y_columns_.size(); i++) {
        if (node->y_columns_[i].value_ > kZeroTolerance) {
            ostringstream oss;
            oss << "    Y" << (i + 1) << " = " << fixed << setprecision(4)
                << node->y_columns_[i].value_ << " [";
            for (int j = 0; j < (int)node->y_columns_[i].pattern_.size(); j++) {
                if (j > 0) oss << ", ";
                oss << node->y_columns_[i].pattern_[j];
            }
            oss << "]";
            LOG(oss.str().c_str());
        }
    }

    LOG("  X列 (条带切割方案):");
    for (int i = 0; i < (int)node->x_columns_.size(); i++) {
        if (node->x_columns_[i].value_ > kZeroTolerance) {
            ostringstream oss;
            oss << "    X" << (i + 1) << " (条带" << node->x_columns_[i].strip_type_id_ + 1
                << ") = " << fixed << setprecision(4) << node->x_columns_[i].value_ << " [";
            for (int j = 0; j < (int)node->x_columns_[i].pattern_.size(); j++) {
                if (j > 0) oss << ", ";
                oss << node->x_columns_[i].pattern_[j];
            }
            oss << "]";
            LOG(oss.str().c_str());
        }
    }
}

// 打印节点信息
void PrintNodeInfo(BPNode* node) {
    LOG_FMT("=== 节点 %d 信息 ===\n", node->id_);
    LOG_FMT("  父节点: %d\n", node->parent_id_);
    LOG_FMT("  下界: %.4f\n", node->lower_bound_);
    LOG_FMT("  剪枝: %d\n", node->prune_flag_);
    LOG_FMT("  分支完成: %d\n", node->branched_flag_);
    LOG_FMT("  Y列数: %d\n", (int)node->y_columns_.size());
    LOG_FMT("  X列数: %d\n", (int)node->x_columns_.size());
}
