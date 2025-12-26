// =============================================================================
// primal_heuristic.cpp - 原始启发式算法
// =============================================================================
//
// 功能: 使用贪心启发式生成初始可行解和模型矩阵
//
// =============================================================================

#include "2DBP.h"

using namespace std;


// =============================================================================
// RunHeuristic - 原始启发式生成初始解
// =============================================================================
void RunHeuristic(ProblemParams& params, ProblemData& data, BPNode& root_node) {

    int num_item_types = params.num_item_types_;
    int num_strip_types = params.num_strip_types_;

    // =========================================================================
    // 初始化
    // =========================================================================
    params.is_finished_ = false;
    data.assigned_items_.clear();

    int stock_id = 0;
    int stock_pattern = 0;
    int strip_id = 0;
    int strip_pattern = 0;

    // =========================================================================
    // 主循环: 逐个母板进行切割
    // =========================================================================
    while (params.is_finished_ == false) {

        // ----- 从母板列表取出一块母板 -----
        data.stocks_.erase(data.stocks_.begin());

        // ----- 初始化当前母板 -----
        Stock new_stock;
        new_stock.length_ = data.stocks_[0].length_;
        new_stock.width_ = data.stocks_[0].width_;
        new_stock.area_ = new_stock.length_ * new_stock.width_;
        new_stock.id_ = stock_id;
        new_stock.x_ = 0;
        new_stock.y_ = 0;

        // 初始化母板内各条带类型计数
        for (int k = 0; k < num_strip_types; k++) {
            StripType strip_type;
            strip_type.type_id_ = k + 1;
            new_stock.strip_types_.push_back(strip_type);
        }

        // ----- 记录母板剩余可用区域 -----
        Item remain_area;
        remain_area.length_ = new_stock.length_;
        remain_area.width_ = new_stock.width_;
        remain_area.area_ = new_stock.area_;
        remain_area.x_ = new_stock.x_;
        remain_area.y_ = new_stock.y_;
        remain_area.stock_id_ = new_stock.id_;
        remain_area.assign_flag_ = 0;

        // =====================================================================
        // 内循环: 在当前母板上切割条带
        // =====================================================================
        int num_items = data.items_.size();

        for (int j = 0; j < num_items; j++) {
            // ----- 检查子件 j 是否可以放入剩余区域 -----
            if (data.items_[j].length_ <= remain_area.length_
                && data.items_[j].width_ <= remain_area.width_
                && data.items_[j].assign_flag_ == 0) {

                // =============================================================
                // 创建条带的第一个子件
                // =============================================================
                Item head_item;
                data.items_[j].assign_flag_ = 1;

                head_item.type_id_ = data.items_[j].type_id_;
                head_item.id_ = data.items_[j].id_;
                head_item.demand_ = data.items_[j].demand_;
                head_item.length_ = data.items_[j].length_;
                head_item.width_ = data.items_[j].width_;
                head_item.area_ = head_item.length_ * head_item.width_;
                head_item.x_ = remain_area.x_;
                head_item.y_ = remain_area.y_;
                head_item.stock_id_ = remain_area.stock_id_;
                head_item.strip_id_ = strip_id;
                head_item.assign_flag_ = 1;

                data.assigned_items_.push_back(head_item);

                // =============================================================
                // 初始化新条带
                // =============================================================
                Strip new_strip;
                new_strip.id_ = strip_id;
                new_strip.type_id_ = head_item.type_id_;
                new_strip.length_ = remain_area.length_;
                new_strip.width_ = head_item.width_;
                new_strip.area_ = new_strip.length_ * new_strip.width_;
                new_strip.x_ = head_item.x_;
                new_strip.y_ = head_item.y_;
                new_strip.stock_id_ = remain_area.stock_id_;
                new_strip.items_.push_back(head_item);

                // 初始化条带内各子件类型计数
                for (int k = 0; k < num_item_types; k++) {
                    ItemType item_type;
                    item_type.type_id_ = k + 1;
                    new_strip.item_types_.push_back(item_type);
                }

                int type_idx = head_item.type_id_ - 1;
                new_strip.item_types_[type_idx].count_++;

                // =============================================================
                // 计算第一个子件右侧的剩余区域
                // =============================================================
                Item strip_remain;
                strip_remain.length_ = remain_area.length_ - head_item.length_;
                strip_remain.width_ = head_item.width_;
                strip_remain.area_ = strip_remain.length_ * strip_remain.width_;
                strip_remain.x_ = remain_area.x_ + head_item.length_;
                strip_remain.y_ = remain_area.y_;
                strip_remain.stock_id_ = remain_area.stock_id_;
                strip_remain.type_id_ = -1;
                strip_remain.assign_flag_ = 0;

                // =============================================================
                // 在条带内继续放置子件 (贪心填充)
                // =============================================================
                for (int m = 0; m < num_items; m++) {
                    if (data.items_[m].length_ <= strip_remain.length_
                        && data.items_[m].width_ <= strip_remain.width_
                        && data.items_[m].assign_flag_ == 0) {

                        Item new_item;
                        data.items_[m].assign_flag_ = 1;

                        new_item.type_id_ = data.items_[m].type_id_;
                        new_item.id_ = data.items_[m].id_;
                        new_item.demand_ = data.items_[m].demand_;
                        new_item.length_ = data.items_[m].length_;
                        new_item.width_ = data.items_[m].width_;
                        new_item.area_ = new_item.length_ * new_item.width_;
                        new_item.x_ = strip_remain.x_;
                        new_item.y_ = strip_remain.y_;
                        new_item.stock_id_ = remain_area.stock_id_;
                        new_item.strip_id_ = strip_id;
                        new_item.assign_flag_ = 1;

                        data.assigned_items_.push_back(new_item);
                        new_strip.items_.push_back(new_item);

                        int item_idx = new_item.type_id_ - 1;
                        new_strip.item_types_[item_idx].count_++;

                        // 更新剩余区域 (向右移动)
                        strip_remain.length_ = strip_remain.length_ - new_item.length_;
                        strip_remain.x_ = strip_remain.x_ + new_item.length_;
                    }
                }

                // =============================================================
                // 判断条带模式是否为新模式
                // =============================================================
                int match_count = 0;
                int num_strips = data.strips_.size();

                if (num_strips == 0) {
                    // 第一个条带, 必然是新模式
                    new_strip.pattern_ = strip_pattern;
                    strip_pattern++;
                    root_node.x_patterns_.push_back(new_strip);
                } else {
                    // 与已有条带模式比较
                    for (int s = 0; s < num_strips; s++) {
                        int cnt = 0;
                        for (int k = 0; k < num_item_types; k++) {
                            int count1 = data.strips_[s].item_types_[k].count_;
                            int count2 = new_strip.item_types_[k].count_;
                            if (count1 == count2) {
                                cnt++;
                            }
                        }
                        match_count = cnt;
                        if (match_count == num_item_types) {
                            // 找到相同模式
                            new_strip.pattern_ = data.strips_[s].pattern_;
                            break;
                        }
                    }

                    if (match_count < num_item_types) {
                        // 确认为新模式
                        new_strip.pattern_ = strip_pattern;
                        strip_pattern++;
                        root_node.x_patterns_.push_back(new_strip);
                    }
                }

                strip_id++;
                data.strips_.push_back(new_strip);
                new_stock.strips_.push_back(new_strip);

                int strip_idx = new_strip.type_id_ - 1;
                new_stock.strip_types_[strip_idx].count_++;

                // ----- 更新母板剩余区域 (向下移动) -----
                remain_area.width_ = remain_area.width_ - head_item.width_;
                remain_area.y_ = remain_area.y_ + head_item.width_;

                // ----- 检查是否所有子件已分配 -----
                int num_assigned = 0;
                int total_items = data.items_.size();
                for (int k = 0; k < total_items; k++) {
                    num_assigned += data.items_[k].assign_flag_;
                }
                if (num_assigned == total_items) {
                    params.is_finished_ = true;
                }
            }
        }

        // =====================================================================
        // 计算母板的切割损耗
        // =====================================================================
        int total_cut_dist = 0;
        int num_strips_in_stock = new_stock.strips_.size();

        for (int j = 0; j < num_strips_in_stock; j++) {
            Strip cur_strip = new_stock.strips_[j];
            int strip_cut_dist = 0;
            int num_items_in_strip = cur_strip.items_.size();

            for (int k = 0; k < num_items_in_strip; k++) {
                Item cur_item = cur_strip.items_[k];

                if (cur_item.width_ < cur_strip.width_) {
                    cur_item.cut_dist_ = cur_item.length_ + cur_item.width_;
                }
                if (cur_item.width_ == cur_strip.width_) {
                    cur_item.cut_dist_ = cur_item.width_;
                }

                strip_cut_dist = strip_cut_dist + cur_item.cut_dist_;
            }

            if (cur_strip.x_ + cur_strip.width_ < new_stock.x_ + new_stock.width_) {
                cur_strip.cut_dist_ = strip_cut_dist + cur_strip.length_;
                cur_strip.cut_loss_ = cur_strip.cut_dist_ * params.unit_cut_cost_;
            }
            if (cur_strip.x_ + cur_strip.width_ == new_stock.x_ + new_stock.width_) {
                cur_strip.cut_dist_ = strip_cut_dist;
                cur_strip.cut_loss_ = cur_strip.cut_dist_ * params.unit_cut_cost_;
            }
            total_cut_dist = total_cut_dist + cur_strip.cut_dist_;
        }

        new_stock.cut_dist_ = total_cut_dist;
        new_stock.cut_loss_ = new_stock.cut_dist_ * params.unit_cut_cost_;

        // =====================================================================
        // 计算母板的废料面积
        // =====================================================================
        int total_waste = 0;

        for (int j = 0; j < num_strips_in_stock; j++) {
            Strip cur_strip = new_stock.strips_[j];
            int used_area = 0;
            int num_items_in_strip = cur_strip.items_.size();

            for (int k = 0; k < num_items_in_strip; k++) {
                used_area = used_area + cur_strip.items_[k].area_;
            }

            cur_strip.waste_area_ = cur_strip.area_ - used_area;
            cur_strip.area_loss_ = cur_strip.waste_area_ * params.unit_area_cost_;
            total_waste = total_waste + cur_strip.waste_area_;
        }

        new_stock.waste_area_ = new_stock.area_ - total_waste;
        new_stock.area_loss_ = new_stock.waste_area_ * params.unit_area_cost_;

        // =====================================================================
        // 判断母板模式是否为新模式
        // =====================================================================
        int stock_match_count = 0;
        int num_used_stocks = data.used_stocks_.size();

        if (num_used_stocks == 0) {
            // 第一个母板, 必然是新模式
            new_stock.pattern_ = stock_pattern;
            stock_pattern++;
            root_node.y_patterns_.push_back(new_stock);
        }

        if (num_used_stocks != 0) {
            // 与已有母板模式比较
            for (int s = 0; s < num_used_stocks; s++) {
                int cnt = 0;
                for (int k = 0; k < num_strip_types; k++) {
                    int count1 = data.used_stocks_[s].strip_types_[k].count_;
                    int count2 = new_stock.strip_types_[k].count_;

                    if (count1 == count2) {
                        cnt++;
                    }
                }

                stock_match_count = cnt;
                if (stock_match_count == num_strip_types) {
                    // 找到相同模式
                    break;
                }
            }

            if (stock_match_count < num_strip_types) {
                // 确认为新模式
                new_stock.pattern_ = stock_pattern;
                stock_pattern++;
                root_node.y_patterns_.push_back(new_stock);
            }
        }

        data.used_stocks_.push_back(new_stock);
        stock_id = stock_id + 1;
    }

    // =========================================================================
    // 构建模型矩阵
    // =========================================================================
    int num_y_cols = root_node.y_patterns_.size();
    int num_x_cols = root_node.x_patterns_.size();
    int num_j = num_strip_types;
    int num_n = num_item_types;

    // ----- 构建完整模型矩阵 -----
    for (int col = 0; col < num_y_cols + num_x_cols; col++) {
        vector<double> temp_col;

        for (int row = 0; row < num_j + num_n; row++) {
            if (col < num_y_cols) {
                // ===== Y 列 (母板模式) =====
                if (row < num_j) {
                    // C 矩阵: 条带类型产出数量
                    double val = root_node.y_patterns_[col].strip_types_[row].count_;
                    temp_col.push_back(val);
                }
                if (row >= num_j) {
                    // 0 矩阵: 零值
                    double val = 0;
                    temp_col.push_back(val);
                }
            }

            if (col >= num_y_cols) {
                // ===== X 列 (条带模式) =====
                if (row < num_j) {
                    // D 矩阵: 条带类型对应关系
                    int col_idx = col - num_y_cols;
                    int item_type_id = row + 1;
                    int strip_type_id = root_node.x_patterns_[col_idx].type_id_;

                    if (strip_type_id == item_type_id) {
                        double val = -1;
                        temp_col.push_back(val);
                    } else {
                        double val = 0;
                        temp_col.push_back(val);
                    }
                }
                if (row >= num_j) {
                    // B 矩阵: 子件类型产出数量
                    int col_idx = col - num_y_cols;
                    int row_idx = row - num_j;
                    double val = root_node.x_patterns_[col_idx].item_types_[row_idx].count_;
                    temp_col.push_back(val);
                }
            }
        }

        root_node.matrix_.push_back(temp_col);
    }

    // ----- 构建 Y 列列表 (第一阶段模式) -----
    for (int col = 0; col < num_y_cols; col++) {
        vector<double> temp_col;

        for (int row = 0; row < num_j + num_n; row++) {
            if (row < num_j) {
                double val = root_node.y_patterns_[col].strip_types_[row].count_;
                temp_col.push_back(val);
            }
            if (row >= num_j) {
                double val = 0;
                temp_col.push_back(val);
            }
        }

        root_node.y_cols_.push_back(temp_col);
    }

    // ----- 构建 X 列列表 (第二阶段模式) -----
    for (int col = num_y_cols; col < num_y_cols + num_x_cols; col++) {
        vector<double> temp_col;

        for (int row = 0; row < num_j + num_n; row++) {
            if (row < num_j) {
                int col_idx = col - num_y_cols;
                int item_type_id = row + 1;
                int strip_type_id = root_node.x_patterns_[col_idx].type_id_;

                if (strip_type_id == item_type_id) {
                    double val = -1;
                    temp_col.push_back(val);
                } else {
                    double val = 0;
                    temp_col.push_back(val);
                }
            }
            if (row >= num_j) {
                int col_idx = col - num_y_cols;
                int row_idx = row - num_j;
                double val = root_node.x_patterns_[col_idx].item_types_[row_idx].count_;
                temp_col.push_back(val);
            }
        }

        root_node.x_cols_.push_back(temp_col);
    }

    // ----- 初始化条带类型列表 -----
    for (int k = 0; k < num_item_types; k++) {
        StripType strip_type;
        strip_type.width_ = data.item_types_[k].width_;
        strip_type.length_ = params.stock_length_;

        data.strip_types_.push_back(strip_type);
    }

    cout << "[启发式] 初始解生成完成 (Y=" << num_y_cols << ", X=" << num_x_cols << ")\n";
}
