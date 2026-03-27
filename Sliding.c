#include "Sliding.h"

// 私有函数声明（仅当前文件可见）
static void OutContinuation(cSMC* smc_obj);
static float Signal(float s);
static float Sat(cSMC* smc_obj, float s);

// 初始化函数
void cSMC_Init(cSMC* smc_obj) {
    if (smc_obj == NULL) return;

    smc_obj->smc.param.J = 0.0f;
    smc_obj->smc.param.K = 0.0f;
    smc_obj->smc.param.c = 0.0f;
    smc_obj->smc.param.epsilon = 0.0f;
    smc_obj->smc.flag = EXPONENT;
    smc_obj->smc.u_max = 0.0f;
    smc_obj->smc.limit = 0.0f;

    smc_obj->smc.error.tar_now = 0.0f;
    smc_obj->smc.error.tar_last = 0.0f;
    smc_obj->smc.error.tar_differential = 0.0f;

    smc_obj->smc.error.p_error = 0.0f;
    smc_obj->smc.error.v_error = 0.0f;
    smc_obj->smc.error.v_error_integral = 0.0f;
    smc_obj->smc.error.pos_error_eps = 0.0f;
    smc_obj->smc.error.vol_error_eps = 0.0f;
    smc_obj->smc.error.pos_get = 0.0f;
    smc_obj->smc.error.vol_get = 0.0f;
}

// 参数设置：EXPONENT/POWER/VELSMC模式
void cSMC_SetParam_ExponentPowerVel(cSMC* smc_obj, float J, float K, float c, 
                                    float epsilon, float limit, float u_max, 
                                    Rmode flag, float pos_esp) {
    if (smc_obj == NULL) return;

    smc_obj->smc.param.J = J;
    smc_obj->smc.param.K = K;
    smc_obj->smc.param.c = c;
    smc_obj->smc.error.pos_error_eps = pos_esp;
    smc_obj->smc.flag = flag;
    smc_obj->smc.param.epsilon = epsilon;
    smc_obj->smc.u_max = u_max;
    smc_obj->smc.limit = limit;
    OutContinuation(smc_obj);
}

// 参数设置：TFSMC模式
void cSMC_SetParam_TFSMC(cSMC* smc_obj, float J, float K, float p, float q, 
                         float beta, float epsilon, float limit, float u_max, 
                         Rmode flag, float pos_esp) {
    if (smc_obj == NULL) return;

    smc_obj->smc.param.J = J;
    smc_obj->smc.param.K = K;
    smc_obj->smc.param.p = p;
    smc_obj->smc.param.q = q;
    smc_obj->smc.error.pos_error_eps = pos_esp;
    smc_obj->smc.param.beta = beta;
    smc_obj->smc.flag = flag;
    smc_obj->smc.param.epsilon = epsilon;
    smc_obj->smc.u_max = u_max;
    smc_obj->smc.limit = limit;
    OutContinuation(smc_obj);
}

// 参数设置：EISMC模式
void cSMC_SetParam_EISMC(cSMC* smc_obj, float J, float K, float c1, float c2, 
                         float epsilon, float limit, float u_max, 
                         Rmode flag, float pos_esp) {
    if (smc_obj == NULL) return;

    smc_obj->smc.param.J = J;
    smc_obj->smc.param.K = K;
    smc_obj->smc.param.c1 = c1;
    smc_obj->smc.param.c2 = c2;
    smc_obj->smc.error.pos_error_eps = pos_esp;
    smc_obj->smc.flag = flag;
    smc_obj->smc.param.epsilon = epsilon;
    smc_obj->smc.u_max = u_max;
    smc_obj->smc.limit = limit;
    OutContinuation(smc_obj);
}

// 位置+速度误差更新
void cSMC_ErrorUpdate_PosVel(cSMC* smc_obj, float target, float pos_now, float vol_now) {
    if (smc_obj == NULL) return;

    smc_obj->smc.error.tar_now = target;
    smc_obj->smc.error.tar_differential = (target - smc_obj->smc.error.tar_last) / SAMPLE_PERIOD;
    smc_obj->smc.error.tar_differential_second = (smc_obj->smc.error.tar_differential - smc_obj->smc.error.tar_differential_last) / SAMPLE_PERIOD;

    smc_obj->smc.error.p_error = pos_now - target;
    smc_obj->smc.error.v_error = vol_now - smc_obj->smc.error.tar_differential;
    smc_obj->smc.error.tar_last = smc_obj->smc.error.tar_now;

    smc_obj->smc.error.p_error_integral += smc_obj->smc.error.p_error * SAMPLE_PERIOD;
    smc_obj->smc.error.tar_differential_last = smc_obj->smc.error.tar_differential;
}

// 仅速度误差更新
void cSMC_ErrorUpdate_Vel(cSMC* smc_obj, float target, float vol_now) {
    if (smc_obj == NULL) return;

    smc_obj->smc.error.tar_now = target;
    smc_obj->smc.error.tar_differential = (target - smc_obj->smc.error.tar_last) / SAMPLE_PERIOD;

    smc_obj->smc.error.v_error = vol_now - smc_obj->smc.error.tar_now;
    smc_obj->smc.error.v_error_integral += smc_obj->smc.error.v_error * SAMPLE_PERIOD;

    smc_obj->smc.error.tar_last = smc_obj->smc.error.tar_now;
}

// 清空所有误差和状态
void cSMC_Clear(cSMC* smc_obj) {
    if (smc_obj == NULL) return;

    smc_obj->smc.error.tar_now = 0.0f;
    smc_obj->smc.error.tar_last = 0.0f;
    smc_obj->smc.error.tar_differential = 0.0f;

    smc_obj->smc.error.p_error = 0.0f;
    smc_obj->smc.error.v_error = 0.0f;
    smc_obj->smc.error.v_error_integral = 0.0f;
    smc_obj->smc.error.pos_error_eps = 0.0f;
    smc_obj->smc.error.vol_error_eps = 0.0f;
    smc_obj->smc.error.pos_get = 0.0f;
    smc_obj->smc.error.vol_get = 0.0f;

    smc_obj->smc.error.tar_differential_second = 0.0f;
    smc_obj->smc.error.tar_differential_last = 0.0f;
    smc_obj->smc.error.p_error_integral = 0.0f;
}

// 清空积分项
void cSMC_Integval_Clear(cSMC* smc_obj) {
    if (smc_obj == NULL) return;

    smc_obj->smc.error.v_error_integral = 0.0f;
    smc_obj->smc.error.p_error_integral = 0.0f;
}

// 滑模控制核心计算
float cSMC_SmcCalculate(cSMC* smc_obj) {
    if (smc_obj == NULL) return 0.0f;

    float u, fun;
    Sliding* smc = &smc_obj->smc;

    switch (smc->flag) {
        case EXPONENT: // 指数滑模面，位置控制
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;// 死区内误差归零且输出0控制量
                return 0.0f;
            }

            smc->s = smc->param.c * smc->error.p_error + smc->error.v_error;
            fun = Sat(smc_obj, smc->s);
            u = smc->param.J * (-smc->param.c * smc->error.v_error - smc->param.K * smc->s - smc->param.epsilon * fun);
            break;

        case POWER: // 幂次滑模面，位置控制
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;
                return 0.0f;
            }

            smc->s = smc->param.c * smc->error.p_error + smc->error.v_error;
            fun = Sat(smc_obj, smc->s);
            u = smc->param.J * (-smc->param.c * smc->error.v_error - smc->param.K * smc->s - smc->param.K * powf(fabsf(smc->s), smc->param.epsilon) * fun);
            break;

        case TFSMC: // TFSMC滑模面
            static float pos_pow; // 静态变量保留
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;
                return 0.0f;
            }

            pos_pow = powf(fabsf(smc->error.p_error), smc->param.q / smc->param.p);
            if (smc->error.p_error <= 0.0f) pos_pow = -pos_pow;

            smc->s = smc->param.beta * pos_pow + smc->error.v_error;
            fun = Sat(smc_obj, smc->s);

            if (smc->error.p_error != 0.0f) {
                u = smc->param.J * (
                    smc->error.tar_differential_second - 
                    smc->param.K * smc->s - 
                    smc->param.epsilon * fun - 
                    smc->error.v_error * ((smc->param.q * smc->param.beta) * pos_pow) / (smc->param.p * smc->error.p_error)
                );
            } else {
                u = 0.0f;
            }
            break;

        case VELSMC: // 速度滑模面
            smc->s = smc->error.v_error + smc->param.c * smc->error.v_error_integral;
            fun = Sat(smc_obj, smc->s);
            u = smc->param.J * (smc->error.tar_differential - smc->param.c * smc->error.v_error - smc->param.K * smc->s - smc->param.epsilon * fun);
            break;

        case EISMC: // EISMC滑模面，位置控制
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;
                return 0.0f;
            }

            smc->s = smc->param.c1 * smc->error.p_error + smc->error.v_error + smc->param.c2 * smc->error.p_error_integral;
            fun = Sat(smc_obj, smc->s);
            u = smc->param.J * (-smc->param.c1 * smc->error.v_error - smc->param.c2 * smc->error.p_error - smc->param.K * smc->s - smc->param.epsilon * fun);
            break;

        default:
            u = 0.0f;
            break;
    }

    smc->error.error_last = smc->error.p_error;// 更新上次误差

    // 输出限幅
    if (u > smc->u_max) {
        u = smc->u_max;
    }
    if (u < -smc->u_max) {
        u = -smc->u_max;
    }
    smc->u = u;
    return u;
}

// 获取输出值
float cSMC_Out(cSMC* smc_obj) {
    return (smc_obj != NULL) ? smc_obj->smc.u : 0.0f;
}

// 设置输出值
void cSMC_SetOut(cSMC* smc_obj, float out) {
    if (smc_obj != NULL) {
        smc_obj->smc.u = out;
    }
}

// 获取滑模结构体
const Sliding* cSMC_getSmc(const cSMC* smc_obj) {
    return (smc_obj != NULL) ? &smc_obj->smc : NULL;
}

// 私有函数：参数续传
static void OutContinuation(cSMC* smc_obj) {
    if (smc_obj == NULL) return;

    Sliding* smc = &smc_obj->smc;
    if (smc->param.K != 0.0f && smc->param.c2 != 0.0f) {
        smc->error.p_error_integral = (smc->param_last.K / smc->param.K) * (smc->param_last.c2 / smc->param.c2) * smc->error.p_error_integral;
        smc->error.v_error_integral = (smc->param_last.K / smc->param.K) * (smc->param_last.c / smc->param.c) * smc->error.v_error_integral;
    }
    smc->param_last = smc->param;
}

// 私有函数：符号函数
static float Signal(float s) {
    if (s > 0.0f)
        return 1.0f;
    else if (s == 0.0f)
        return 0.0f;
    else
        return -1.0f;
}

// 私有函数：饱和函数
static float Sat(cSMC* smc_obj, float s) {
    if (smc_obj == NULL) return 0.0f;

    float y = s / smc_obj->smc.param.epsilon;
    if (fabsf(y) <= smc_obj->smc.limit)
        return y;
    else
        return Signal(y);
}
