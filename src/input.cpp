// =============================================================================
// input.cpp - 数据读取模块
// =============================================================================
//
// 功能: 从文件读取二维下料问题的输入数据
//
// -----------------------------------------------------------------------------
// 输入文件格式
// -----------------------------------------------------------------------------
//
// 文件采用制表符 (TAB) 分隔的纯文本格式:
//
//   第 1 行: <母板数量>
//   第 2 行: <子件类型数量>
//   第 3 行: <母板长度> TAB <母板宽度>
//   第 4 行起: <子件长度> TAB <子件宽度> TAB <需求量> TAB <类型索引>
//
// 示例文件:
//   100
//   3
//   1000    500
//   200     100     10      1
//   150     80      15      2
//   100     50      20      3
//
// 说明:
//   - 母板数量: 可用母板的上界 (实际使用数量由求解器决定)
//   - 子件类型数量: 不同规格子件的种类数
//   - 母板尺寸: 长度 (X方向) x 宽度 (Y方向)
//   - 子件信息: 每行一种类型, 包含尺寸、需求量和类型编号
//
// -----------------------------------------------------------------------------
// 数据预处理
// -----------------------------------------------------------------------------
//
// 读取数据后进行以下预处理:
//   1. 按需求量展开子件: 每个需求创建一个独立的子件实例
//   2. 按宽度降序排序: 便于启发式算法中的贪心放置
//   3. 初始化母板列表: 创建指定数量的空白母板
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// SplitString - 字符串分割函数
// =============================================================================
//
// 功能: 按指定分隔符将字符串分割成多个子字符串
//
// 算法:
//   1. 从位置 0 开始查找分隔符
//   2. 提取分隔符之前的子字符串
//   3. 移动到分隔符之后, 重复步骤 1-2
//   4. 处理最后一个子字符串 (分隔符之后的部分)
//
// 参数:
//   line_string - 待分割的原始字符串
//   string_list - 输出: 分割后的子字符串列表
//   data_string - 分隔符 (如 "\t" 表示制表符)
//
// 示例:
//   输入: "100\t50\t20", 分隔符="\t"
//   输出: ["100", "50", "20"]
//
// =============================================================================
void SplitString(const string& line_string, vector<string>& string_list, const string& data_string) {

    // 位置指针: pos1 = 当前子串起始位置, pos2 = 分隔符位置
    string::size_type pos1, pos2;

    // 查找第一个分隔符
    pos2 = line_string.find(data_string);
    pos1 = 0;

    // 清空输出列表
    string_list.clear();

    // 循环查找所有分隔符
    while (string::npos != pos2) {
        // 提取分隔符之前的子字符串
        string_list.push_back(line_string.substr(pos1, pos2 - pos1));

        // 移动到分隔符之后
        pos1 = pos2 + data_string.size();

        // 查找下一个分隔符
        pos2 = line_string.find(data_string, pos1);
    }

    // 处理最后一个子字符串 (最后一个分隔符之后的部分)
    if (pos1 != line_string.length()) {
        string_list.push_back(line_string.substr(pos1));
    }
}


// =============================================================================
// ReadData - 读取问题数据
// =============================================================================
//
// 功能: 从文件读取二维下料问题的完整输入数据
//
// 处理流程:
//   1. 打开输入文件
//   2. 读取母板数量和尺寸
//   3. 读取子件类型数量
//   4. 逐行读取每种子件的规格和需求量
//   5. 按需求量展开为独立子件实例
//   6. 按宽度降序排序所有子件
//   7. 初始化母板列表
//
// 参数:
//   Values - 输出: 全局参数结构体
//            填充: stocks_num, item_types_num, strip_types_num,
//                  stock_length, stock_width, items_num
//   Lists  - 输出: 全局列表结构体
//            填充: all_stocks_list, all_items_list, all_item_types_list
//
// 注意:
//   - 当前文件路径为硬编码, 实际使用时需要修改
//   - 建议改为命令行参数或配置文件
//
// =============================================================================
void ReadData(All_Values& Values, All_Lists& Lists) {

    // 文件路径和读取变量
    ostringstream s_in;
    string in_str;
    string line;
    vector<string> data_inline;

    // =========================================================================
    // 设置输入文件路径
    // =========================================================================
    // TODO: 建议改为命令行参数或配置文件
    s_in.str("");
    s_in << "C:/Users/zym60/OneDrive/2DCSP/2DCG/cutdata1207.txt";
    in_str = s_in.str();

    cout << "[数据] 读取文件: " << in_str << "\n";

    // =========================================================================
    // 打开并读取文件
    // =========================================================================
    ifstream fin(in_str.c_str());

    if (fin) {

        // ---------------------------------------------------------------------
        // 读取第 1 行: 母板数量
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, data_inline, "\t");
        Values.stocks_num = atoi(data_inline[0].c_str());

        // ---------------------------------------------------------------------
        // 读取第 2 行: 子件类型数量
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, data_inline, "\t");
        Values.item_types_num = atoi(data_inline[0].c_str());

        // 条带类型数 = 子件类型数 (每种子件宽度对应一种条带类型)
        Values.strip_types_num = Values.item_types_num;

        // ---------------------------------------------------------------------
        // 读取第 3 行: 母板尺寸
        // ---------------------------------------------------------------------
        getline(fin, line);
        SplitString(line, data_inline, "\t");
        Values.stock_length = atoi(data_inline[0].c_str());  // 长度 (X 方向)
        Values.stock_width = atoi(data_inline[1].c_str());   // 宽度 (Y 方向)

        // 输出读取的基本信息
        cout << "[数据] 母板尺寸: " << Values.stock_length << " x " << Values.stock_width << "\n";
        cout << "[数据] 母板数量: " << Values.stocks_num << "\n";
        cout << "[数据] 子件类型数: " << Values.item_types_num << "\n";

        // ---------------------------------------------------------------------
        // 初始化母板列表
        // ---------------------------------------------------------------------
        // 创建 stocks_num 个空白母板, 所有母板尺寸相同
        for (int i = 0; i < Values.stocks_num; i++) {
            One_Stock this_stock;
            this_stock.length = Values.stock_length;
            this_stock.width = Values.stock_width;
            this_stock.area = Values.stock_length * Values.stock_width;
            this_stock.pos_x = 0;
            this_stock.pos_y = 0;

            // 插入到列表头部 (后续会从头部取出使用)
            Lists.all_stocks_list.insert(Lists.all_stocks_list.begin(), this_stock);
        }

        // ---------------------------------------------------------------------
        // 读取子件数据 (第 4 行起)
        // ---------------------------------------------------------------------
        int item_index = 1;                    // 子件实例索引 (从 1 开始)
        int item_types_num = Values.item_types_num;
        int total_demand = 0;                  // 统计总需求量

        for (int i = 0; i < item_types_num; i++) {
            // 读取一行子件类型数据
            getline(fin, line);
            SplitString(line, data_inline, "\t");

            // 解析该类型的需求量
            int item_type_demand_num = atoi(data_inline[2].c_str());
            total_demand += item_type_demand_num;

            // -----------------------------------------------------------------
            // 按需求量展开: 每个需求创建一个独立的子件实例
            // -----------------------------------------------------------------
            // 这样做的目的:
            //   1. 便于在启发式中逐个分配子件
            //   2. 支持同类型子件放置在不同位置
            for (int k = 0; k < item_type_demand_num; k++) {
                One_Item this_item;

                // 设置基本属性
                this_item.item_type_idx = atoi(data_inline[3].c_str());  // 类型索引
                this_item.item_idx = item_index;                          // 实例索引
                this_item.demand = atoi(data_inline[2].c_str());          // 需求量 (冗余存储)
                this_item.length = atoi(data_inline[0].c_str());          // 长度
                this_item.width = atoi(data_inline[1].c_str());           // 宽度
                this_item.area = this_item.length * this_item.width;      // 面积

                // 初始化位置和状态
                this_item.pos_x = -1;          // 未放置
                this_item.pos_y = -1;          // 未放置
                this_item.stock_idx = -1;      // 未分配到母板
                this_item.occupied_flag = 0;   // 未分配

                // 添加到子件列表
                Lists.all_items_list.push_back(this_item);

                item_index++;
                Values.items_num++;
            }

            // -----------------------------------------------------------------
            // 创建子件类型记录
            // -----------------------------------------------------------------
            One_Item_Type this_item_type;
            this_item_type.item_type_idx = atoi(data_inline[3].c_str());
            this_item_type.demand = atoi(data_inline[2].c_str());
            this_item_type.width = atoi(data_inline[1].c_str());
            this_item_type.length = atoi(data_inline[0].c_str());

            Lists.all_item_types_list.push_back(this_item_type);
        }

        cout << "[数据] 子件总需求: " << total_demand << "\n";

        fin.close();

    } else {
        // 文件打开失败
        cout << "[错误] 无法打开文件: " << in_str << "\n";
    }

    // =========================================================================
    // 按宽度降序排序所有子件
    // =========================================================================
    // 排序目的:
    //   - 启发式算法优先处理较宽的子件
    //   - 较宽子件更难放置, 优先处理可提高装载率
    //
    // 排序算法: 简单冒泡排序 (数据量小, 效率足够)

    One_Item temp_item;
    int all_items_num = Lists.all_items_list.size();

    for (int i = 0; i < all_items_num - 1; i++) {
        for (int j = i + 1; j < all_items_num; j++) {
            // 如果后面的子件宽度更大, 则交换
            if (Lists.all_items_list[i].width < Lists.all_items_list[j].width) {
                temp_item = Lists.all_items_list[i];
                Lists.all_items_list[i] = Lists.all_items_list[j];
                Lists.all_items_list[j] = temp_item;
            }
        }
    }

    cout << "[数据] 数据读取完成, 子件已按宽度降序排序\n";
}
