#include "vision/target_board.h"
#include "vision/Image_Process.h"  // ImageDeal, ImageStatus

#if defined(TARGET_BOARD_USE_TFLM)
  #include "model/loong_cnn_model_simple.h"
  #include "tensorflow/lite/core/api/error_reporter.h"
  #include "tensorflow/lite/micro/micro_interpreter.h"
  #include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
  #include "tensorflow/lite/micro/system_setup.h"
  #include "tensorflow/lite/schema/schema_generated.h"
#endif

#include <opencv2/imgproc.hpp>
#include <vector>
#include <algorithm>

// 全局 override 状态
TargetBoardOverride g_target_override;

// 模型类别 (与 train.py 提示顺序一致)
// 0=materials(物资) 1=traffic(交通工具) 2=weapon(武器)
static constexpr int kModelInputW   = 40;
static constexpr int kModelInputH   = 40;
static constexpr int kModelInputCh  = 3;
static constexpr int kModelClassNum = 3;
static constexpr int kTensorArenaSz = 128 * 1024;

#if defined(TARGET_BOARD_USE_TFLM)
// ---- TFLM 单例 ----
static tflite::MicroMutableOpResolver<20>* g_resolver = nullptr;
static tflite::MicroInterpreter*           g_interpreter = nullptr;
static uint8_t*                             g_arena = nullptr;

static void EnsureTflm()
{
    if (g_interpreter) return;
    tflite::InitializeTarget();
    g_resolver = new tflite::MicroMutableOpResolver<20>();
    g_resolver->AddConv2D();
    g_resolver->AddDepthwiseConv2D();
    g_resolver->AddMaxPool2D();
    g_resolver->AddFullyConnected();
    g_resolver->AddRelu6();
    g_resolver->AddSoftmax();
    g_resolver->AddReshape();
    g_resolver->AddShape();
    g_resolver->AddQuantize();
    g_resolver->AddDequantize();
    g_resolver->AddCast();
    g_resolver->AddSqueeze();
    g_resolver->AddExpandDims();
    g_resolver->AddConcatenation();
    g_resolver->AddTranspose();
    g_resolver->AddStridedSlice();
    g_resolver->AddPack();
    g_resolver->AddLogistic();
    g_resolver->AddMean();
    g_resolver->AddAdd();

    const tflite::Model* model = ::tflite::GetModel(loong_cnn_model_simple_tflite);
    g_arena = new uint8_t[kTensorArenaSz];
    g_interpreter = new tflite::MicroInterpreter(model, *g_resolver, g_arena, kTensorArenaSz);
    g_interpreter->AllocateTensors();
}
#endif

// ---- 状态机 ----
struct DetectionState
{
    int         candidateKind      = -1;  // 当前候选类别 (连续计数中)
    int         candidateCount     = 0;   // 候选连续帧数
};

static DetectionState g_state;

#ifdef TARGET_BOARD_TEST_HOOKS
static TargetInferFn g_infer_hook;
void TargetBoardSetInferHook(TargetInferFn hook) { g_infer_hook = std::move(hook); }
#endif

// 推理封装: 输入 40x40 BGR cv::Mat -> 类别 0..2, 返回 -1 表示跳过/不可推理
static int RunInfer(const cv::Mat& roi_bgr_40)
{
#ifdef TARGET_BOARD_TEST_HOOKS
    if (g_infer_hook) return g_infer_hook(roi_bgr_40);
#endif
#if defined(TARGET_BOARD_USE_TFLM)
    EnsureTflm();
    cv::Mat fimg;
    roi_bgr_40.convertTo(fimg, CV_32FC3, 1.0);
    cv::Mat rgb;
    cv::cvtColor(fimg, rgb, cv::COLOR_BGR2RGB);
    cv::Mat cont = rgb.isContinuous() ? rgb : rgb.clone();
    float* in = g_interpreter->input(0)->data.f;
    std::memcpy(in, cont.ptr<float>(0), kModelInputW * kModelInputH * kModelInputCh * sizeof(float));
    g_interpreter->Invoke();
    float* out = g_interpreter->output(0)->data.f;
    int   best = 0;
    float maxv = out[0];
    for (int n = 1; n < kModelClassNum; ++n)
        if (out[n] > maxv) { maxv = out[n]; best = n; }
    return best;
#else
    (void)roi_bgr_40;
    return -1;  // host 无 TFLM, 跳过推理
#endif
}

void TargetBoardReset()
{
    g_state.candidateKind  = -1;
    g_state.candidateCount = 0;
    g_target_override.active = false;
    g_target_override.kind   = TargetKind::NONE;
    g_target_override.remain = 0;
}

/*
    检测到的红色色块记录
*/
struct Blob
{
    int  cx, cy;     // 中心坐标 (160x120 原图)
    int  area;       // 像素面积
    cv::Rect bbox;    // 外接矩形
};

static bool InTrack(int bx, int by)
{
    // 160x120 行 by -> 80x60 行 = by-30, 要求 by>=30
    if (by < 30) return false;
    int r = by - 30;
    if (r >= 60 || r < (int)ImageStatus.OFFLine) return false;
    int lb = ImageDeal[r].LeftBorder;
    int rb = ImageDeal[r].RightBorder;
    int lbx = lb * 2;   // 80x60 col -> 160x120 原 x
    int rbx = rb * 2;
    return bx >= lbx && bx <= rbx;
}

void TargetBoardProcess(cv::Mat& img_color, Data_Path* Data_Path_p)
{
    if (!Data_Path_p || Data_Path_p->JSON_TargetBoardConfigData_v.empty()) return;
    const JSON_TargetBoardConfigData& cfg = Data_Path_p->JSON_TargetBoardConfigData_v[0];

    // 过期 override 倒计时 (无论本帧是否检测, 已确认的 override 持续帧数递减)
    if (g_target_override.active)
    {
        if (--g_target_override.remain <= 0)
        {
            g_target_override.active = false;
            g_target_override.kind   = TargetKind::NONE;
        }
    }

    if (!cfg.enable) { return; }
    if (img_color.empty()) return;

    // ---- 步骤A: BGR 域 inRange 红色掩膜 (省去 cvtColor) ----
    cv::Scalar lo(cfg.bMin, cfg.gMin, cfg.rMin);
    cv::Scalar hi(cfg.bMax, cfg.gMax,  cfg.rMax);
    cv::Mat mask;
    cv::inRange(img_color, lo, hi, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ---- 步骤B: 筛选 色块: 面积>阈值 且 中心在赛道边界内; 从下往上记录 ----
    std::vector<Blob> valid;
    for (auto& c : contours)
    {
        double a = cv::contourArea(c);
        if (a < cfg.minArea) continue;
        cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);
        cv::Rect bb = cv::boundingRect(c);
        if (!InTrack(cx, cy)) continue;
        valid.push_back({cx, cy, static_cast<int>(a), bb});
    }
    // 从下往上 = cy 从大到小
    std::sort(valid.begin(), valid.end(),
              [](const Blob& a, const Blob& b){ return a.cy > b.cy; });

    int kind = -1;
    if (!valid.empty())
    {
        // 选最大色块作为目标
        const Blob* best = &valid[0];
        for (const auto& b : valid) if (b.area > best->area) best = &b;

        // ---- 步骤C: y>Y_GATE 才送分类 ----
        if (best->cy > cfg.yGate)
        {
            // ROI 几何中心 = 色块 bbox 中心; 按指定像素偏移; 以新中心画指定大小的矩形
            cv::Rect bb = best->bbox;
            int nCx = bb.x + bb.width  / 2 + cfg.roiOffsetX;
            int nCy = bb.y + bb.height / 2 + cfg.roiOffsetY;
            cv::Rect roi(nCx - cfg.roiW / 2, nCy - cfg.roiH / 2, cfg.roiW, cfg.roiH);
            // 要求矩形完整落在图像内, 防止部分越界导致 resize 拉伸失真
            if (roi.x >= 0 && roi.y >= 0 &&
                roi.x + roi.width  <= img_color.cols &&
                roi.y + roi.height <= img_color.rows &&
                roi.width > 0 && roi.height > 0)
            {
                cv::Mat crop(img_color, roi);
                cv::Mat resized;
                if (roi.width != kModelInputW || roi.height != kModelInputH) {
                    cv::resize(crop, resized, cv::Size(kModelInputW, kModelInputH), 0, 0, cv::INTER_LINEAR);
                    kind = RunInfer(resized);
                } else {
                    // 尺寸已匹配模型输入, 直接零拷贝推理
                    kind = RunInfer(crop);
                }
            }
        }
    }

    // ---- 步骤D: 连续同类计数 -> 确认 ----
    if (kind >= 0 && kind < kModelClassNum)
    {
        if (kind == g_state.candidateKind)
            g_state.candidateCount++;
        else
        {
            g_state.candidateKind  = kind;
            g_state.candidateCount = 1;
        }
        if (g_state.candidateCount >= cfg.confirmFrames)
        {
            // ---- 步骤E: 确认 -> 设置 override ----
            g_target_override.active = true;
            g_target_override.kind   = static_cast<TargetKind>(kind);
            g_target_override.remain = cfg.overrideTimeoutFrames;
            // 确认后重置候选, 防止连续累加误判
            g_state.candidateKind  = -1;
            g_state.candidateCount = 0;
        }
    }
    else if (kind == -1)
    {
        // 本帧无有效分类, 保持候选计数不变 (允许短暂丢失)
    }
    else
    {
        // 异常类别值, 重置
        g_state.candidateKind  = -1;
        g_state.candidateCount = 0;
    }
}