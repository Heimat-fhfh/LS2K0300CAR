/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library

@修改日期：2025-02-26
@修改内容：
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "LQ_YOLO.hpp"

using namespace cv;
using namespace std;
using namespace cv::dnn;

cv::Mat img;
cv::Rect m_select;

Net net;
Mat frame, resized_pic;
int main()
{
    // 加载模型
    net = readNetFromONNX("best.onnx");
    
    // 读取当前目录下的1.jpg图片
    frame = imread("1.jpg");
    if (frame.empty()) {
        cerr << "Could not read the image '1.jpg'" << endl;
        return -1;
    }
    
    /******************************图像初步处理*******************************/
    // 旋转图像180度
    rotate(frame, frame, ROTATE_180);
    // 缩放图像
    resize(frame, resized_pic, Size(64, 64));
    /******************************图像初步处理*******************************/
    
    // 执行YOLO检测
    YOLO_Detecion(resized_pic);
    return 0;
}
