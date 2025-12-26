// =============================================================================
// output_results.cpp - 结果输出模块
// =============================================================================
//
// 功能: 输出启发式求解的切割方案结果
//
// 输出内容:
//   1. 控制台输出: 每块母板的切割方案摘要
//   2. 文件输出: 切割方案坐标信息 (用于可视化绘图)
//
// 文件输出路径:
//   D:/CuttingTXT/Stock_N.txt (N = 母板索引, 从 0 开始)
//
// 坐标格式说明:
//   每个矩形输出四个顶点坐标, 按逆时针顺序:
//     (x, y)           - 左下角
//     (x, y+w)         - 左上角
//     (x+l, y+w)       - 右上角
//     (x+l, y)         - 右下角
//
// 标识符:
//   - x: 母板边界
//   - I: 子件 (Item)
//   - S: 条带 (Strip)
//
// 可视化示例:
//
// =============================================================================

#include "2DBP.h"

using namespace std;

// -----------------------------------------------------------------------------
// ExportResults - 输出启发式求解结果
// -----------------------------------------------------------------------------
// 功能: 将切割方案以坐标形式输出到控制台和文件
//
// 控制台输出:
//   - 总母板数量
//   - 每块母板的尺寸、条带数、子件数
//
// 文件输出:
//   - 每块母板一个文件
//   - 包含母板边界、子件、条带的顶点坐标
//
// 参数:
//   params - 全局参数
//   data   - 全局列表 (包含切割结果)
// -----------------------------------------------------------------------------
void ExportResults(ProblemParams& params, ProblemData& data) {

    int num_stocks = data.used_stocks_.size();
    int num_items = data.assigned_items_.size();
    int num_strips = data.strips_.size();

    // =========================================================================
    // 控制台输出: 切割方案摘要
    // =========================================================================
    cout << "[结果] 切割方案输出 (母板数: " << num_stocks << ")\n";

    for (int pos = 0; pos < num_stocks; pos++) {
        int stock_len = data.used_stocks_[0].length_;
        int stock_wid = data.used_stocks_[0].width_;

        // ----- 统计当前母板上的条带和子件数量 -----
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

    // =========================================================================
    // 文件输出: 切割方案坐标 (用于可视化绘图)
    // =========================================================================
    ostringstream s_in, s_out;
    string in_str, out_str;
    ofstream f_out;

    for (int pos = 0; pos < num_stocks; pos++) {
        // ----- 设置输出文件路径 -----
        s_out.str("");
        s_out << "D:/CuttingTXT/Stock_" << pos << ".txt";
        out_str = s_out.str();
        f_out.open(out_str, ios::out);

        int stock_len = data.used_stocks_[0].length_;
        int stock_wid = data.used_stocks_[0].width_;

        // =====================================================================
        // 输出母板边界 (标识符: x)
        // =====================================================================
        // 四个顶点按逆时针顺序:
        //   (0, 0) -> (0, W) -> (L, W) -> (L, 0)
        // =====================================================================
        f_out << 0 << "\t" << 0 << "\t" << "x" << endl;
        f_out << 0 << "\t" << stock_wid << "\t" << "x" << endl;
        f_out << stock_len << "\t" << stock_wid << "\t" << "x" << endl;
        f_out << stock_len << "\t" << 0 << "\t" << "x" << endl;

        // =====================================================================
        // 输出子件坐标 (标识符: I + 类型索引)
        // =====================================================================
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

        // =====================================================================
        // 输出条带坐标 (标识符: S + 类型索引)
        // =====================================================================
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
