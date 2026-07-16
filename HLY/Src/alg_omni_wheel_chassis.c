//
// Created by 34285 on 2026/7/7.
//
#include "alg_omni_wheel_chassis.h"

/**
 * @brief 4轮X型全向轮底盘逆运动学解算
 *
 * @param config 底盘物理参数 (需提供r_center)
 * @param target 期望的底盘三轴速度
 * @param output 解算后的四轮电机角速度输出
 */
void Omni_Wheel_Inverse_Kinematics(const Struct_Chassis_Config* config, const Struct_Target_Velocity* target,
                                   Struct_Wheel_Velocity* output)
{
    //基础运动学方程：计算车轮处所需的线速度 (m/s)
    float v_LF = - SQRT_2_OVER_2 * target->vx + SQRT_2_OVER_2 * target->vy + target->omega * config->r_center;
    float v_LB = - SQRT_2_OVER_2 * target->vx - SQRT_2_OVER_2 * target->vy + target->omega * config->r_center;
    float v_RB =   SQRT_2_OVER_2 * target->vx - SQRT_2_OVER_2 * target->vy + target->omega * config->r_center;
    float v_RF =   SQRT_2_OVER_2 * target->vx + SQRT_2_OVER_2 * target->vy + target->omega * config->r_center;

    //线速度转角速度：v = ω * R -> ω = v / R (rad/s)
    output->LF = v_LF / config->R;
    output->LB = v_LB / config->R;
    output->RB = v_RB / config->R;
    output->RF = v_RF / config->R;

    //关键安全防护：等比例缩放防饱和
    //寻找当前四个轮子中的最大转速绝对值
    float max_calc_omega = fabs(output->LF);
    if (fabs(output->LB) > max_calc_omega)
    {
        max_calc_omega = fabs(output->LB);
    }
    if (fabs(output->RB) > max_calc_omega)
    {
        max_calc_omega = fabs(output->RB);
    }
    if (fabs(output->RF) > max_calc_omega)
    {
        max_calc_omega = fabs(output->RF);
    }
    if (max_calc_omega > config->max_omega)
    {
        //缩放比例必须是正数，范围在(0,1)之间
        float scale = config->max_omega / max_calc_omega;
        output->LF *= scale;
        output->LB *= scale;
        output->RB *= scale;
        output->RF *= scale;
    }
}

/**
 * @brief 4轮X型全向轮底盘逆动力学解算 (基于伪逆矩阵力矩分配)
 *
 * @param config 底盘物理参数 (需提供 r_center, R 和 max_torque)
 * @param target_force 期望的底盘三轴合力与力矩
 * @param output_torque 解算后的四轮电机扭矩输出
 */
void Omni_Wheel_Inverse_Dynamics(const Struct_Chassis_Config* config, const Struct_Target_Force* target_force,
                                 Struct_Wheel_Torque* output_torque)
{
    //逆动力学分配系数：由于4轮对称均摊受力，各轮分担 1/4 的总需求
    //SQRT_2_OVER_2 / 2.0f 相当于 sqrt(2)/4
    float sqrt2_over_4 = SQRT_2_OVER_2 / 2.0f;

    //自转力矩分配部分：Mz / (4 * r_center)
    float mz_part = target_force->Mz / (4.0f * config->r_center);

    //计算各车轮所需的地面摩擦力 (单位: N)
    //符号矩阵与逆运动学雅可比矩阵完全对称对应
    float f_LF = - sqrt2_over_4 * target_force->Fx + sqrt2_over_4 * target_force->Fy + mz_part;
    float f_LB = - sqrt2_over_4 * target_force->Fx - sqrt2_over_4 * target_force->Fy + mz_part;
    float f_RB =   sqrt2_over_4 * target_force->Fx - sqrt2_over_4 * target_force->Fy + mz_part;
    float f_RF =   sqrt2_over_4 * target_force->Fx + sqrt2_over_4 * target_force->Fy + mz_part;

    //地面摩擦力转换为电机扭矩：τ = F * R (单位: N·m)
    output_torque->LF = f_LF * config->R;
    output_torque->LB = f_LB * config->R;
    output_torque->RB = f_RB * config->R;
    output_torque->RF = f_RF * config->R;

    //关键安全防护：等比例缩放防饱和限幅
    //如果直接截断（Clamp）会导致合力方向发生偏转，采用等比例缩放(Scale)能保持底盘运动轨迹不形变
    float max_calc_torque = fabs(output_torque->LF);
    if (fabs(output_torque->LB) > max_calc_torque)
    {
        max_calc_torque = fabs(output_torque->LB);
    }
    if (fabs(output_torque->RB) > max_calc_torque)
    {
        max_calc_torque = fabs(output_torque->RB);
    }
    if (fabs(output_torque->RF) > max_calc_torque)
    {
        max_calc_torque = fabs(output_torque->RF);
    }

    //防饱和处理
    if (max_calc_torque > config->max_torque)
    {
        float scale = config->max_torque / max_calc_torque;
        output_torque->LF *= scale;
        output_torque->LB *= scale;
        output_torque->RB *= scale;
        output_torque->RF *= scale;
    }
}

