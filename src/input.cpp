// =============================================================================
// input.cpp - 数据读取模块
// =============================================================================
//
// 功能: 从文件读取二维下料问题的输入数据
//
// 输入文件格式:
//   第1行: 母板数量
//   第2行: 子件类型数量
//   第3行: 母板长度 <TAB> 母板宽度
//   第4行起: 子件长度 <TAB> 子件宽度 <TAB> 需求量 <TAB> 类型索引
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// SplitString - 字符串分割函数
// -----------------------------------------------------------------------------
// 功能: 按指定分隔符将字符串分割成子串列表
// 参数:
//   line_string - 待分割的原始字符串
//   string_list - 输出的子串列表
//   data_string - 分隔符
// -----------------------------------------------------------------------------
void SplitString(const string& line_string, vector<string>& string_list, const string& data_string) {
	string::size_type pos1, pos2;
	pos2 = line_string.find(data_string);
	pos1 = 0;
	string_list.clear();

	// 循环查找分隔符并分割
	while (string::npos != pos2) {
		string_list.push_back(line_string.substr(pos1, pos2 - pos1));
		pos1 = pos2 + data_string.size();
		pos2 = line_string.find(data_string, pos1);
	}

	// 处理最后一个子串
	if (pos1 != line_string.length()) {
		string_list.push_back(line_string.substr(pos1));
	}
}

// -----------------------------------------------------------------------------
// ReadData - 读取问题数据
// -----------------------------------------------------------------------------
// 功能: 从文件读取母板和子件的完整信息
// 参数:
//   Values - 全局参数结构体
//   Lists  - 全局列表结构体
// -----------------------------------------------------------------------------
void ReadData(All_Values& Values, All_Lists& Lists) {
	ostringstream s_in;
	string in_str;
	string line;
	vector<string> data_inline;

	// 设置输入文件路径
	s_in.str("");
	s_in << "C:/Users/zym60/OneDrive/2DCSP/2DCG/cutdata1207.txt";
	in_str = s_in.str();

	ifstream fin(in_str.c_str());

	if (fin) {
		// ----- 读取第1行: 母板数量 -----
		getline(fin, line);
		SplitString(line, data_inline, "\t");
		Values.stocks_num = atoi(data_inline[0].c_str());

		// ----- 读取第2行: 子件类型数量 -----
		getline(fin, line);
		SplitString(line, data_inline, "\t");
		Values.item_types_num = atoi(data_inline[0].c_str());
		Values.strip_types_num = Values.item_types_num;

		// ----- 读取第3行: 母板尺寸 -----
		getline(fin, line);
		SplitString(line, data_inline, "\t");
		Values.stock_length = atoi(data_inline[0].c_str());
		Values.stock_width = atoi(data_inline[1].c_str());

		// 初始化母板列表
		for (int i = 0; i < Values.stocks_num; i++) {
			One_Stock this_stock;
			this_stock.length = Values.stock_length;
			this_stock.width = Values.stock_width;
			this_stock.area = Values.stock_length * Values.stock_width;
			this_stock.pos_x = 0;
			this_stock.pos_y = 0;
			Lists.all_stocks_list.insert(Lists.all_stocks_list.begin(), this_stock);
		}

		// ----- 读取子件数据 -----
		int item_index = 1;
		int item_types_num = Values.item_types_num;

		for (int i = 0; i < item_types_num; i++) {
			getline(fin, line);
			SplitString(line, data_inline, "\t");

			int item_type_demand_num = atoi(data_inline[2].c_str());

			// 按需求量生成单个子件实例
			for (int k = 0; k < item_type_demand_num; k++) {
				One_Item this_item;
				this_item.item_type_idx = atoi(data_inline[3].c_str());
				this_item.item_idx = item_index;
				this_item.demand = atoi(data_inline[2].c_str());
				this_item.length = atoi(data_inline[0].c_str());
				this_item.width = atoi(data_inline[1].c_str());
				this_item.area = this_item.length * this_item.width;
				this_item.pos_x = -1;
				this_item.pos_y = -1;
				this_item.stock_idx = -1;
				this_item.occupied_flag = 0;

				Lists.all_items_list.push_back(this_item);
				item_index++;
				Values.items_num++;
			}

			// 创建子件类型记录
			One_Item_Type this_item_type;
			this_item_type.item_type_idx = atoi(data_inline[3].c_str());
			this_item_type.demand = atoi(data_inline[2].c_str());
			this_item_type.width = atoi(data_inline[1].c_str());
			this_item_type.length = atoi(data_inline[0].c_str());
			Lists.all_item_types_list.push_back(this_item_type);
		}
	}

	// ----- 按宽度降序排序子件 -----
	// 目的: 启发式算法优先放置宽度较大的子件
	One_Item temp_item;
	int all_items_num = Lists.all_items_list.size();

	for (int i = 0; i < all_items_num - 1; i++) {
		for (int j = i + 1; j < all_items_num; j++) {
			if (Lists.all_items_list[i].width < Lists.all_items_list[j].width) {
				temp_item = Lists.all_items_list[i];
				Lists.all_items_list[i] = Lists.all_items_list[j];
				Lists.all_items_list[j] = temp_item;
			}
		}
	}
}
