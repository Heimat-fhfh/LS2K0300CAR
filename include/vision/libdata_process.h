#include "vision/libdata_store.h"

#ifndef _LIBDATA_PROCESS_H_
#define _LIBDATA_PROCESS_H_


class Judge
{
    public:

        /*
            电机速度决策
            @参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关数据指针
        */
        void MotorSpeed_Judge(Img_Store *Img_Store_p,Data_Path *Data_Path_p);


        /*
            跳变扫描赛道元素识别
            使用横向跳变扫描识别圆环和十字赛道元素
            @ 参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关数据指针
            Function_EN_p 函数使能指针
        */
        void TransitionScanDetect(Img_Store* Img_Store_p, Data_Path* Data_Path_p, Function_EN* Function_EN_p);

    
    private:
        /*
            边线拐点寻找
            @ 参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关数据指针
        */
        void InflectionPointSearch(Img_Store* Img_Store_p,Data_Path *Data_Path_p);


        /*
            边线弯点寻找
            @ 参数说明
            Img_Store_p 图像存储指针
            Data_Path_p 路径相关数据指针
        */
        void BendPointSearch(Img_Store* Img_Store_p,Data_Path *Data_Path_p);


};


class SYNC
{
    public:
        /*
            车辆上位机设置文件数据同步
            @参数说明
            Function_EN_p 函数使能指针
            Data_Path_p 路径相关数据指针
        */
        void ConfigData_SYNC(Data_Path *Data_Path_p,Function_EN *Function_EN_p);

        // 获取当前选择的配置文件路径（config/config_*.json）
        std::string GetConfigFilePath() const;
};


#endif

