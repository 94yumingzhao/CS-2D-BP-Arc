// =============================================================================
// primal_heuristic.cpp - 原始启发式算法
// =============================================================================
//
// 功能: 使用贪心启发式生成初始可行解和模型矩阵
//
// 算法思路:
//   1. 按宽度优先的两阶段切割策略
//   2. 逐个母板进行切割, 直到所有子件需求满足
//   3. 每个条带由其第一个子件的宽度决定类型
//   4. 在条带内贪心放置可行的子件
//
// 主问题矩阵结构:
//   ┌─────────────────┬─────────────────┐
//   │        C        │        D        │  条带约束 >= 0
//   ├─────────────────┼─────────────────┤
//   │        0        │        B        │  子件约束 >= demand
//   └─────────────────┴─────────────────┘
//      Y列(母板模式)      X列(条带模式)
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// PrimalHeuristic - 原始启发式生成初始解
// -----------------------------------------------------------------------------
// 功能: 使用两阶段切割启发式生成初始可行解, 构建模型矩阵
// 参数:
//   Values    - 全局参数
//   Lists     - 全局列表
//   root_node - 根节点, 用于存储生成的模式和矩阵
// -----------------------------------------------------------------------------
void PrimalHeuristic(All_Values& Values, All_Lists& Lists, Node& root_node) {
	int item_types_num = Values.item_types_num;
	int strip_types_num = Values.strip_types_num;

	Values.Finish = false;
	Lists.occupied_items_list.clear();

	int stock_index = 0;
	int stock_pattern = 0;
	int strip_index = 0;
	int strip_pattern = 0;

	// =========================================================================
	// 主循环: 逐个母板进行切割
	// =========================================================================
	while (Values.Finish == false) {

		// 从母板列表取出一块母板
		Lists.all_stocks_list.erase(Lists.all_stocks_list.begin());

		// ----- 初始化当前母板 -----
		One_Stock new_stock;
		new_stock.length = Lists.all_stocks_list[0].length;
		new_stock.width = Lists.all_stocks_list[0].width;
		new_stock.area = new_stock.length * new_stock.width;
		new_stock.stock_idx = stock_index;
		new_stock.pos_x = 0;
		new_stock.pos_y = 0;

		// 初始化母板内各条带类型计数
		for (int k = 0; k < strip_types_num; k++) {
			One_Strip_Type this_strip_type;
			this_strip_type.strip_type_idx = k + 1;
			new_stock.strip_types_list.push_back(this_strip_type);
		}

		// 记录母板剩余可用区域
		One_Item stock_remain;
		stock_remain.length = new_stock.length;
		stock_remain.width = new_stock.width;
		stock_remain.area = new_stock.area;
		stock_remain.pos_x = new_stock.pos_x;
		stock_remain.pos_y = new_stock.pos_y;
		stock_remain.stock_idx = new_stock.stock_idx;
		stock_remain.occupied_flag = 0;

		// =====================================================================
		// 内循环: 在当前母板上切割条带
		// =====================================================================
		int all_items_num = Lists.all_items_list.size();
		for (int j = 0; j < all_items_num; j++) {
			// 检查子件 j 是否可以放入剩余区域
			if (Lists.all_items_list[j].length <= stock_remain.length
				&& Lists.all_items_list[j].width <= stock_remain.width
				&& Lists.all_items_list[j].occupied_flag == 0) {

				// ----- 创建条带的第一个子件 -----
				One_Item first_item;
				Lists.all_items_list[j].occupied_flag = 1;

				first_item.item_type_idx = Lists.all_items_list[j].item_type_idx;
				first_item.item_idx = Lists.all_items_list[j].item_idx;
				first_item.demand = Lists.all_items_list[j].demand;
				first_item.length = Lists.all_items_list[j].length;
				first_item.width = Lists.all_items_list[j].width;
				first_item.area = first_item.length * first_item.width;
				first_item.pos_x = stock_remain.pos_x;
				first_item.pos_y = stock_remain.pos_y;
				first_item.stock_idx = stock_remain.stock_idx;
				first_item.strip_idx = strip_index;
				first_item.occupied_flag = 1;

				Lists.occupied_items_list.push_back(first_item);

				// ----- 初始化新条带 -----
				One_Strip new_strip;
				new_strip.strip_idx = strip_index;
				new_strip.strip_type_idx = first_item.item_type_idx;
				new_strip.length = stock_remain.length;  // 条带长度 = 母板长度
				new_strip.width = first_item.width;      // 条带宽度 = 首个子件宽度
				new_strip.area = new_strip.length * new_strip.width;
				new_strip.pos_x = first_item.pos_x;
				new_strip.pos_y = first_item.pos_y;
				new_strip.stock_idx = stock_remain.stock_idx;
				new_strip.items_list.push_back(first_item);

				// 初始化条带内各子件类型计数
				for (int k = 0; k < item_types_num; k++) {
					One_Item_Type this_item_type;
					this_item_type.item_type_idx = k + 1;
					new_strip.item_types_list.push_back(this_item_type);
				}

				int type_pos = first_item.item_type_idx - 1;
				new_strip.item_types_list[type_pos].this_item_type_num++;

				// ----- 计算第一个子件右侧剩余区域 -----
				One_Item first_item_right_side;
				first_item_right_side.length = stock_remain.length - first_item.length;
				first_item_right_side.width = first_item.width;
				first_item_right_side.area = first_item_right_side.length * first_item_right_side.width;
				first_item_right_side.pos_x = stock_remain.pos_x + first_item.length;
				first_item_right_side.pos_y = stock_remain.pos_y;
				first_item_right_side.stock_idx = stock_remain.stock_idx;
				first_item_right_side.item_type_idx = -1;
				first_item_right_side.occupied_flag = 0;

				// ----- 在条带内继续放置子件 -----
				for (int m = 0; m < all_items_num; m++) {
					if (Lists.all_items_list[m].length <= first_item_right_side.length
						&& Lists.all_items_list[m].width <= first_item_right_side.width
						&& Lists.all_items_list[m].occupied_flag == 0) {

						One_Item new_item;
						Lists.all_items_list[m].occupied_flag = 1;

						new_item.item_type_idx = Lists.all_items_list[m].item_type_idx;
						new_item.item_idx = Lists.all_items_list[m].item_idx;
						new_item.demand = Lists.all_items_list[m].demand;
						new_item.length = Lists.all_items_list[m].length;
						new_item.width = Lists.all_items_list[m].width;
						new_item.area = new_item.length * new_item.width;
						new_item.pos_x = first_item_right_side.pos_x;
						new_item.pos_y = first_item_right_side.pos_y;
						new_item.stock_idx = stock_remain.stock_idx;
						new_item.strip_idx = strip_index;
						new_item.occupied_flag = 1;

						Lists.occupied_items_list.push_back(new_item);
						new_strip.items_list.push_back(new_item);

						int itm_pos = new_item.item_type_idx - 1;
						new_strip.item_types_list[itm_pos].this_item_type_num++;

						// 更新剩余区域
						first_item_right_side.length = first_item_right_side.length - new_item.length;
						first_item_right_side.pos_x = first_item_right_side.pos_x + new_item.length;
					}
				}

				// ----- 判断条带模式是否为新模式 -----
				int Strip_Final_Cnt = 0;
				int all_strips_num = Lists.all_strips_list.size();

				if (all_strips_num == 0) {
					// 第一个条带, 必然是新模式
					new_strip.pattern = strip_pattern;
					strip_pattern++;
					root_node.X_patterns_list.push_back(new_strip);
				} else {
					// 与已有条带模式比较
					for (int s = 0; s < all_strips_num; s++) {
						int cnt01 = 0;
						for (int k = 0; k < item_types_num; k++) {
							int cnt1 = Lists.all_strips_list[s].item_types_list[k].this_item_type_num;
							int cnt2 = new_strip.item_types_list[k].this_item_type_num;
							if (cnt1 == cnt2) {
								cnt01++;
							}
						}
						Strip_Final_Cnt = cnt01;
						if (Strip_Final_Cnt == item_types_num) {
							// 找到相同模式
							new_strip.pattern = Lists.all_strips_list[s].pattern;
							break;
						}
					}

					if (Strip_Final_Cnt < item_types_num) {
						// 确认为新模式
						new_strip.pattern = strip_pattern;
						strip_pattern++;
						root_node.X_patterns_list.push_back(new_strip);
					}
				}

				strip_index++;
				Lists.all_strips_list.push_back(new_strip);
				new_stock.strips_list.push_back(new_strip);

				int stp_pos = new_strip.strip_type_idx - 1;
				new_stock.strip_types_list[stp_pos].this_strip_type_num++;

				// ----- 更新母板剩余区域 (向下移动) -----
				stock_remain.length = stock_remain.length;
				stock_remain.width = stock_remain.width - first_item.width;
				stock_remain.area = stock_remain.length * stock_remain.width;
				stock_remain.pos_x = stock_remain.pos_x;
				stock_remain.pos_y = stock_remain.pos_y + first_item.width;

				// ----- 检查是否所有子件已分配 -----
				int occupied_items_num = 0;
				int all_items_num = Lists.all_items_list.size();
				for (int k = 0; k < all_items_num; k++) {
					occupied_items_num += Lists.all_items_list[k].occupied_flag;
				}
				if (occupied_items_num == all_items_num) {
					Values.Finish = true;
				}
			}
		}

		// =====================================================================
		// 计算母板的切割成本
		// =====================================================================
		int strip_total_cut_distance = 0;
		int strips_num_in_stock = new_stock.strips_list.size();

		for (int j = 0; j < strips_num_in_stock; j++) {
			One_Strip this_strip = new_stock.strips_list[j];
			int item_total_cut_distance = 0;
			int items_num_in_strip = this_strip.items_list.size();

			for (int k = 0; k < items_num_in_strip; k++) {
				One_Item this_item = this_strip.items_list[k];

				if (this_item.width < this_strip.width) {
					this_item.cutting_distance = this_item.length + this_item.width;
				}
				if (this_item.width == this_strip.width) {
					this_item.cutting_distance = this_item.width;
				}

				item_total_cut_distance = item_total_cut_distance + this_item.cutting_distance;
			}

			if (this_strip.pos_x + this_strip.width < new_stock.pos_x + new_stock.width) {
				this_strip.cutting_distance = item_total_cut_distance + this_strip.length;
				this_strip.material_cutting_loss = this_strip.cutting_distance * Values.unit_cut_loss;
			}
			if (this_strip.pos_x + this_strip.width == new_stock.pos_x + new_stock.width) {
				this_strip.cutting_distance = item_total_cut_distance;
				this_strip.material_cutting_loss = this_strip.cutting_distance * Values.unit_cut_loss;
			}
			strip_total_cut_distance = strip_total_cut_distance + this_strip.cutting_distance;
		}

		new_stock.cutting_distance = strip_total_cut_distance;
		new_stock.material_cutting_loss = new_stock.cutting_distance * Values.unit_cut_loss;

		// =====================================================================
		// 计算母板的废料面积
		// =====================================================================
		int strip_total_waste_area = 0;
		for (int j = 0; j < strips_num_in_stock; j++) {
			One_Strip this_strip = new_stock.strips_list[j];
			int item_total_used_area = 0;
			int items_num_in_strip = this_strip.items_list.size();

			for (int k = 0; k < items_num_in_strip; k++) {
				item_total_used_area = item_total_used_area + this_strip.items_list[k].area;
			}

			this_strip.wasted_area = this_strip.area - item_total_used_area;
			this_strip.material_area_loss = this_strip.wasted_area * Values.unit_area_loss;
			strip_total_waste_area = strip_total_waste_area + this_strip.wasted_area;
		}

		new_stock.wasted_area = new_stock.area - strip_total_waste_area;
		new_stock.material_area_loss = new_stock.wasted_area * Values.unit_area_loss;

		// =====================================================================
		// 判断母板模式是否为新模式
		// =====================================================================
		int Stock_Final_Cnt = 0;
		int occupied_stocks_num = Lists.occupied_stocks_list.size();

		if (occupied_stocks_num == 0) {
			// 第一个母板, 必然是新模式
			new_stock.pattern = stock_pattern;
			stock_pattern++;
			root_node.Y_patterns_list.push_back(new_stock);
		}

		if (occupied_stocks_num != 0) {
			// 与已有母板模式比较
			for (int s = 0; s < occupied_stocks_num; s++) {
				int cnt01 = 0;
				for (int k = 0; k < strip_types_num; k++) {
					int cnt1 = Lists.occupied_stocks_list[s].strip_types_list[k].this_strip_type_num;
					int cnt2 = new_stock.strip_types_list[k].this_strip_type_num;

					if (cnt1 == cnt2) {
						cnt01++;
					}
				}

				Stock_Final_Cnt = cnt01;
				if (Stock_Final_Cnt == strip_types_num) {
					// 找到相同模式
					break;
				}
			}

			if (Stock_Final_Cnt < strip_types_num) {
				// 确认为新模式
				new_stock.pattern = stock_pattern;
				stock_pattern++;
				root_node.Y_patterns_list.push_back(new_stock);
			}
		}

		Lists.occupied_stocks_list.push_back(new_stock);
		stock_index = stock_index + 1;
	}

	// =========================================================================
	// 构建模型矩阵
	// =========================================================================
	//
	// 矩阵结构:
	//   ┌─────────────────┬─────────────────┐
	//   │     C (J x K)   │     D (J x P)   │  条带约束
	//   ├─────────────────┼─────────────────┤
	//   │     0 (N x K)   │     B (N x P)   │  子件约束
	//   └─────────────────┴─────────────────┘
	//
	// C: 母板模式中各条带类型的产出数量
	// D: 条带模式对应的条带类型 (-1 表示属于该类型)
	// 0: 零矩阵
	// B: 条带模式中各子件类型的产出数量

	int K_num = root_node.Y_patterns_list.size();  // 母板模式数
	int P_num = root_node.X_patterns_list.size();  // 条带模式数
	int J_num = strip_types_num;                   // 条带类型数
	int N_num = item_types_num;                    // 子件类型数

	// ----- 构建完整模型矩阵 -----
	for (int col = 0; col < K_num + P_num; col++) {
		vector<double> temp_col;

		for (int row = 0; row < J_num + N_num; row++) {
			if (col < K_num) {
				// 矩阵 C 和 矩阵 0
				if (row < J_num) {
					// 矩阵 C: 条带类型产出
					double val = root_node.Y_patterns_list[col].strip_types_list[row].this_strip_type_num;
					temp_col.push_back(val);
				}
				if (row >= J_num) {
					// 矩阵 0: 零值
					double val = 0;
					temp_col.push_back(val);
				}
			}

			if (col >= K_num) {
				// 矩阵 D 和 矩阵 B
				if (row < J_num) {
					// 矩阵 D: 条带类型对应关系
					int col_pos = col - K_num;
					int item_type_idx = row + 1;
					int strip_type_idx = root_node.X_patterns_list[col_pos].strip_type_idx;

					if (strip_type_idx == item_type_idx) {
						double val = -1;  // 该模式属于该条带类型
						temp_col.push_back(val);
					} else {
						double val = 0;   // 不属于
						temp_col.push_back(val);
					}
				}
				if (row >= J_num) {
					// 矩阵 B: 子件类型产出
					int col_pos = col - K_num;
					int row_pos = row - J_num;
					double val = root_node.X_patterns_list[col_pos].item_types_list[row_pos].this_item_type_num;
					temp_col.push_back(val);
				}
			}
		}

		root_node.model_matrix.push_back(temp_col);
	}

	cout << endl;

	// ----- 构建 Y 列列表 (第一阶段模式) -----
	for (int col = 0; col < K_num; col++) {
		vector<double> temp_col;

		for (int row = 0; row < J_num + N_num; row++) {
			if (row < J_num) {
				// 矩阵 C 部分
				double val = root_node.Y_patterns_list[col].strip_types_list[row].this_strip_type_num;
				temp_col.push_back(val);
			}
			if (row >= J_num) {
				// 矩阵 0 部分
				double val = 0;
				temp_col.push_back(val);
			}
		}

		root_node.Y_cols_list.push_back(temp_col);
	}

	cout << endl;

	// ----- 构建 X 列列表 (第二阶段模式) -----
	for (int col = K_num; col < K_num + P_num; col++) {
		vector<double> temp_col;

		for (int row = 0; row < J_num + N_num; row++) {
			if (row < J_num) {
				// 矩阵 D 部分
				int col_pos = col - K_num;
				int item_type_index = row + 1;
				int strip_type_index = root_node.X_patterns_list[col_pos].strip_type_idx;

				if (strip_type_index == item_type_index) {
					double val = -1;
					temp_col.push_back(val);
				} else {
					double val = 0;
					temp_col.push_back(val);
				}
			}
			if (row >= J_num) {
				// 矩阵 B 部分
				int col_pos = col - K_num;
				int row_pos = row - J_num;
				double val = root_node.X_patterns_list[col_pos].item_types_list[row_pos].this_item_type_num;
				temp_col.push_back(val);
			}
		}

		root_node.X_cols_list.push_back(temp_col);
	}

	// ----- 初始化条带类型列表 -----
	for (int k = 0; k < item_types_num; k++) {
		One_Strip_Type temp_stp;
		temp_stp.width = Lists.all_item_types_list[k].width;
		temp_stp.length = Values.stock_length;

		Lists.all_strip_types_list.push_back(temp_stp);
	}

	cout << endl;
}
