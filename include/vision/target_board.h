#ifndef _TARGET_BOARD_H_
#define _TARGET_BOARD_H_

#include "vision/libdata_store.h"

// 目标板类别 (与 TFLite 模型 class_labels = {"materials","traffic","weapon"} 对应)
enum class TargetKind : int
{
    NONE      = -1,
    MATERIALS = 0,  // 物资 -> 跟随右边界作为新中线
    TRAFFIC   = 1,  // 交通工具 -> 保持原中线, 直行压过
    WEAPON    = 2,  // 武器 -> 跟随左边界作为新中线
};

// 循迹线 override 状态: 确认某类别后, GetDet() 据此替换 ImageDeal[].Center
struct TargetBoardOverride
{
    bool       active = false;  // 当前是否处于 override 有效期
    TargetKind kind   = TargetKind::NONE;
    int        remain = 0;      // 剩余有效帧数 (每帧调用后递减, 0 时失效)
};

// 全局 override 状态, 供 Image_Process.cpp::GetDet() 读取
extern TargetBoardOverride g_target_override;

/*
    每帧调用: 在 160x120 原图上做红色色块检测 + 分类确认 + 循迹线 override 设置
    必须在 ImageProcess() 写完 ImageDeal 边界之后调用

    @参数说明
    img_color   160x120 BGR 原图 (Img_Store.Img_Color)
    Data_Path_p 路径相关数据指针, 读取 JSON_TargetBoardConfigData_v
*/
void TargetBoardProcess(cv::Mat& img_color, Data_Path* Data_Path_p);

/*
    重置目标板检测状态 (确认计数器、override)
    用于停车、出界复位等场景
*/
void TargetBoardReset();

// ===== 测试桩注入 (仅 host 单测使用) =====
// 注入自定义推理回调, host 测试用。生产代码不调用。
// 回调签名: 输入 cv::Mat(40x40 CV_8UC3 BGR), 输出类别 0..2, 返回 -1 表示不推理
#ifdef TARGET_BOARD_TEST_HOOKS
using TargetInferFn = std::function<int(const cv::Mat&)>;
void TargetBoardSetInferHook(TargetInferFn hook);
#endif

#endif