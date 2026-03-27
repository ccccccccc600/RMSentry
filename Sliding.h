#ifndef KOSANN_UAVGIMBAL_SLIDING_H
#define KOSANN_UAVGIMBAL_SLIDING_H

#include <math.h>
#include <stddef.h>

// 采样周期宏定义
#define SAMPLE_PERIOD 0.002f
#define V_EORROR_INTEGRAL_MAX 2000.0f
#define P_EORROR_INTEGRAL_MAX 2000.0f

// 滑模控制模式枚举
typedef enum {
    EXPONENT,
    POWER,
    TFSMC,
    VELSMC,
    EISMC
} Rmode;

// 误差结构体
typedef struct {
    float tar_now;                // 当前目标值
    float tar_last;               // 上一次目标值
    float tar_differential;       // 目标值一阶微分
    float tar_differential_last;  // 上一次目标值一阶微分
    float tar_differential_second;// 目标值二阶微分

    float pos_get;                // 当前位置
    float vol_get;                // 当前速度

    float p_error;                // 位置误差
    float v_error;                // 速度误差（位置误差一阶微分）

    float p_error_integral;       // 位置误差积分
    float v_error_integral;       // 速度误差积分

    float pos_error_eps;          // 误差阈值
    float vol_error_eps;          // 误差阈值
    float error_last;
} RError;

// 滑模参数结构体
typedef struct {
    float J;
    float K;
    float c;

    float c1;   // EISMC参数
    float c2;   // EISMC参数

    float p;    // TFSMC参数 p>q
    float q;    // TFSMC参数
    float beta; // TFSMC参数
    float epsilon; // 滑模参数
} SlidingParam;

// 滑模控制核心结构体
typedef struct {
    float u; // 控制输出
    float s; // 滑模面计算变量

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max; // 输出限幅
    Rmode flag;  // 滑模模式标志
    float limit; // 饱和函数阈值
} Sliding;

// 滑模控制器结构体（替代C++的class）
typedef struct {
    Sliding smc;
} cSMC;

// 公有函数声明
void cSMC_Init(cSMC* smc_obj);

// 不同模式的参数设置
void cSMC_SetParam_ExponentPowerVel(cSMC* smc_obj, float J, float K, float c, 
                                    float epsilon, float limit, float u_max, 
                                    Rmode flag, float pos_esp); // EXPONENT/POWER/VELSMC
void cSMC_SetParam_TFSMC(cSMC* smc_obj, float J, float K, float p, float q, 
                         float beta, float epsilon, float limit, float u_max, 
                         Rmode flag, float pos_esp); // TFSMC
void cSMC_SetParam_EISMC(cSMC* smc_obj, float J, float K, float c1, float c2, 
                         float epsilon, float limit, float u_max, 
                         Rmode flag, float pos_esp); // EISMC

// 误差更新
void cSMC_ErrorUpdate_PosVel(cSMC* smc_obj, float target, float pos_now, float vol_now); // 位置+速度误差更新
void cSMC_ErrorUpdate_Vel(cSMC* smc_obj, float target, float vol_now);                  // 仅速度误差更新

void cSMC_Clear(cSMC* smc_obj);
void cSMC_Integval_Clear(cSMC* smc_obj);
float cSMC_SmcCalculate(cSMC* smc_obj); // 滑模控制计算
float cSMC_Out(cSMC* smc_obj);
void cSMC_SetOut(cSMC* smc_obj, float out);
const Sliding* cSMC_getSmc(const cSMC* smc_obj); // 获取滑模结构体

#endif // KOSANN_UAVGIMBAL_SLIDING_H
