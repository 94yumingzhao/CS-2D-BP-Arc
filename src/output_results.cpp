// =============================================================================
// output_results.cpp - 结果输出模块
// =============================================================================
// 功能: 输出启发式求解的切割方案结果
//
// 输出内容:
//   1. 控制台输出: 每块母板的切割方案摘要
//   2. 文件输出: 切割方案坐标信息 (用于可视化绘图)
// =============================================================================

#include "2DBP.h"

using namespace std;


// 输出启发式求解结果
void ExportResults(ProblemParams& params, ProblemData& data) {
    int num_stocks = static_cast<int>(data.used_stocks_.size());
    int num_items = static_cast<int>(data.assigned_items_.size());
    int num_strips = static_cast<int>(data.strips_.size());

    // 边界检查: 确保有数据可输出
    if (num_stocks == 0) {
        cout << "[结果] 无切割方案可输出 (已使用母板数为0)\n";
        return;
    }

    // 控制台输出: 切割方案摘要
    cout << "[结果] 切割方案输出 (母板数: " << num_stocks << ")\n";

    for (int pos = 0; pos < num_stocks; pos++) {
        int stock_len = data.used_stocks_[pos].length_;
        int stock_wid = data.used_stocks_[pos].width_;

        // 统计当前母板上的条带和子件数量
        int strip_count = 0;
        int item_count = 0;

        for (size_t i = 0; i < num_strips; i++) {
            if (data.strips_[i].stock_id_ == pos) strip_count++;
        }
        for (size_t i = 0; i < num_items; i++) {
            if (data.assigned_items_[i].stock_id_ == pos) item_count++;
        }

        cout << "[结果] 母板_" << data.used_stocks_[pos].id_
             << " (" << stock_len << " x " << stock_wid << "): 条带=" << strip_count << ", 子件=" << item_count << "\n";
    }

    // 文件输出: 切割方案坐标 (用于可视化绘图)
    ostringstream s_in, s_out;
    string in_str, out_str;
    ofstream f_out;

    for (int pos = 0; pos < num_stocks; pos++) {
        // 设置输出文件路径
        s_out.str("");
        s_out << "D:/CuttingTXT/Stock_" << pos << ".txt";
        out_str = s_out.str();
        f_out.open(out_str, ios::out);

        if (!f_out.is_open()) {
            cerr << "[警告] 无法创建输出文件: " << out_str << "\n";
            continue;
        }

        int stock_len = data.used_stocks_[pos].length_;
        int stock_wid = data.used_stocks_[pos].width_;

        // 输出母板边界 (标识符: x)
        // 四个顶点按逆时针顺序: (0, 0) -> (0, W) -> (L, W) -> (L, 0)
        f_out << 0 << "\t" << 0 << "\t" << "x" << endl;
        f_out << 0 << "\t" << stock_wid << "\t" << "x" << endl;
        f_out << stock_len << "\t" << stock_wid << "\t" << "x" << endl;
        f_out << stock_len << "\t" << 0 << "\t" << "x" << endl;

        // 输出子件坐标 (标识符: I + 类型索引)
        for (size_t i = 0; i < num_items; i++) {
            if (data.assigned_items_[i].stock_id_ == pos) {
                int x = data.assigned_items_[i].x_;
                int y = data.assigned_items_[i].y_;
                int len = data.assigned_items_[i].length_;
                int wid = data.assigned_items_[i].width_;
                int item_type_id = data.assigned_items_[i].type_id_;

                // 四个顶点按逆时针顺序
                f_out << x << "\t" << y << "\t" << "I" << item_type_id << endl;
                f_out << x << "\t" << y + wid << "\t" << "I" << item_type_id << endl;
                f_out << x + len << "\t" << y + wid << "\t" << "I" << item_type_id << endl;
                f_out << x + len << "\t" << y << "\t" << "I" << item_type_id << endl;
            }
        }

        // 输出条带坐标 (标识符: S + 类型索引)
        for (size_t i = 0; i < num_strips; i++) {
            if (data.strips_[i].stock_id_ == pos) {
                int x = data.strips_[i].x_;
                int y = data.strips_[i].y_;
                int len = data.strips_[i].length_;
                int wid = data.strips_[i].width_;
                int strip_type_id = data.strips_[i].type_id_;

                // 四个顶点按逆时针顺序
                f_out << x << "\t" << y << "\t" << "S" << strip_type_id << endl;
                f_out << x << "\t" << y + wid << "\t" << "S" << strip_type_id << endl;
                f_out << x + len << "\t" << y + wid << "\t" << "S" << strip_type_id << endl;
                f_out << x + len << "\t" << y << "\t" << "S" << strip_type_id << endl;
            }
        }

        f_out.close();
    }
}
