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

/**
 * @brief 上坡扭矩补偿——低配版前馈解算
 *
 * @param config 底盘物理参数结构体指针
 * @param imu_data 底盘IMU数据结构体指针
 * @param total_mass 整车总质量 (单位: kg)，即课件中的 m
 * @param feedforward_torque 解算后输出的四轮电机扭矩前馈量
 */
void Calculate_Uphill_Feedforward_Torque(const Struct_Chassis_Config* config,
                                         Struct_IMU_Data* imu_data,
                                         float total_mass,
                                         Struct_Wheel_Torque* feedforward_torque)
{
    // 1. 获取底盘的倾角 βs 以及底盘相对于斜坡的转角 α
    float beta_s = Get_Slope_Pitch(imu_data);
    float alpha  = Get_Chassis_Yaw_Relative_To_Slope(imu_data);

    // 2. 计算克服重力分量的牵引力前馈向量 f 的大小
    // 公式: |f| = mg * sin(βs)
    float f_magnitude = total_mass * GRAVITY_CONSTANT * sinf(beta_s);

    // 3. 将牵引力向量 f 分解到底盘坐标系下
    // f 相对于底盘坐标系的方向为 -α
    // 根据底盘坐标系右手系定义（Fx向右为正，Fy向前为正，逆时针Yaw为正）：
    // 斜坡上坡方向 (0, 1) 从斜坡坐标系转换到底盘坐标系，需要乘以旋转矩阵 R(-α)
    // 推导得出:
    // Fx = f_mag * sin(α)
    // Fy = f_mag * cos(α)
    Struct_Target_Force feedforward_force;
    feedforward_force.Fx = f_magnitude * sinf(alpha);
    feedforward_force.Fy = f_magnitude * cosf(alpha);

    // 像底盘解算那一节一样，把 f 当作底盘目标牵引力，而底盘目标转矩为0
    feedforward_force.Mz = 0.0f;

    // 4. 通过动力学逆解算，得出上坡扭矩前馈
    Omni_Wheel_Inverse_Dynamics(config, &feedforward_force, feedforward_torque);
}

/**
 * @brief 上坡扭矩补偿——高配版前馈解算
 *
 * @param config 底盘物理参数结构体指针
 * @param imu_data 底盘IMU数据结构体指针
 * @param total_mass 整车总质量 (单位: kg)，即课件中的 m
 * @param cog_height 重心高度 (单位: m)，即课件中的 h
 * @param max_f 限制的最大补偿力 (单位: N)，用于防止某些特殊位姿时疯车
 * @param feedforward_torque 解算后输出的四轮电机扭矩前馈量
 */
void Calculate_Uphill_Feedforward_Torque_High_End(const Struct_Chassis_Config* config,
                                                  Struct_IMU_Data* imu_data,
                                                  float total_mass,
                                                  float cog_height,
                                                  float max_f,
                                                  Struct_Wheel_Torque* feedforward_torque)
{
    // 1. 获取底盘的旋转矩阵 C
    Struct_Matrix C = Get_Chassis_Rotation_Matrix(imu_data);

    // 2. 提取矩阵C的第三行，这代表世界坐标系Z轴（重力反方向）在底盘坐标系下的投影
    // C20, C21, C22 决定了重力向量在底盘三轴上的分量
    float c20 = C.data[2 * C.columns + 0];
    float c21 = C.data[2 * C.columns + 1];
    float c22 = C.data[2 * C.columns + 2];

    // 释放矩阵内存，防止内存泄漏
    Free_Matrix(&C);

    // 3. 计算重力投影点 P (px, py)
    // 重心高度 h 乘上 x/y 分量。严格几何推导需除以 c22 以投影到底盘平面。
    float px = 0.0f;
    float py = 0.0f;

    // 避免底盘翻车(c22 <= 0)或极度倾斜时的异常除0错误
    if (c22 > 0.05f)
    {
        px = -cog_height * (c20 / c22);
        py = -cog_height * (c21 / c22);
    }

    // 4. 最好限制重力投影点的范围：不要离四个轮组投影点太近
    // 轮子在 X/Y 轴上的投影距离 W_L
    float W_L = config->r_center * SQRT_2_OVER_2;
    float limit = 0.8f * W_L; // 限制在四个轮子连线矩形的 80% 范围内

    if (px > limit) px = limit;
    if (px < -limit) px = -limit;
    if (py > limit) py = limit;
    if (py < -limit) py = -limit;

    // 5. 计算四个轮组投影点到重力投影点P的距离 L0 ~ L3
    // 轮组坐标(X, Y)： LF(-W_L, W_L), LB(-W_L, -W_L), RB(W_L, -W_L), RF(W_L, W_L)
    float L0 = sqrtf((px - (-W_L)) * (px - (-W_L)) + (py - W_L) * (py - W_L));
    float L1 = sqrtf((px - (-W_L)) * (px - (-W_L)) + (py - (-W_L)) * (py - (-W_L)));
    float L2 = sqrtf((px - W_L) * (px - W_L) + (py - (-W_L)) * (py - (-W_L)));
    float L3 = sqrtf((px - W_L) * (px - W_L) + (py - W_L) * (py - W_L));

    // 6. 假设 N * L = 常数，计算支持力的权重系数 η0 ~ η3
    float inv_L0 = 1.0f / L0;
    float inv_L1 = 1.0f / L1;
    float inv_L2 = 1.0f / L2;
    float inv_L3 = 1.0f / L3;
    float sum_inv = inv_L0 + inv_L1 + inv_L2 + inv_L3;

    float eta0 = inv_L0 / sum_inv;
    float eta1 = inv_L1 / sum_inv;
    float eta2 = inv_L2 / sum_inv;
    float eta3 = inv_L3 / sum_inv;

    // 7. 计算低配版的总牵引力 f
    float beta_s = Get_Slope_Pitch(imu_data);
    float alpha = Get_Chassis_Yaw_Relative_To_Slope(imu_data);
    float f_magnitude = total_mass * GRAVITY_CONSTANT * sinf(beta_s);

    // 最好限制 f 大小范围 (防止疯车)
    if (fabsf(f_magnitude) > max_f)
    {
        f_magnitude = (f_magnitude > 0) ? max_f : -max_f;
    }

    Struct_Target_Force feedforward_force;
    feedforward_force.Fx = f_magnitude * sinf(alpha);
    feedforward_force.Fy = f_magnitude * cosf(alpha);
    feedforward_force.Mz = 0.0f;

    // 8. 动力学逆解算 (先得出低配版各轮平均分配的扭矩)
    Struct_Wheel_Torque low_end_torque;
    Omni_Wheel_Inverse_Dynamics(config, &feedforward_force, &low_end_torque);

    // 9. 牵引力重新分配 (高配版核心)
    // 注意：低配版的 Omni_Wheel_Inverse_Dynamics 函数中，四个轮子是平分总力的 (各占 1/4)
    // 高配版需要按照 eta 权重分配。为了保持总推力不变，我们要把原本的 1/4 替换成 eta
    // 即对低配版的扭矩乘上 (4.0 * eta)
    feedforward_torque->LF = low_end_torque.LF * (4.0f * eta0);
    feedforward_torque->LB = low_end_torque.LB * (4.0f * eta1);
    feedforward_torque->RB = low_end_torque.RB * (4.0f * eta2);
    feedforward_torque->RF = low_end_torque.RF * (4.0f * eta3);
}

/**
 * @brief 符号函数 sgn(x)
 */
static float Math_Sign(float val)
{
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

/**
 * @brief 计算单轮低配版阻力补偿扭矩 (内部调用)
 */
static float Calc_Low_End_Single_Wheel(float omega, float t, float cont_th)
{
    float abs_omega = fabsf(omega);
    float torque = 0.0f;

    if (abs_omega <= cont_th && cont_th > 0.0f)
    {
        // 零点振荡：一阶连续化，用一段正比例函数贴合
        torque = (t / cont_th) * abs_omega;
    }
    else
    {
        // 正常恒定阻力补偿
        torque = t;
    }

    return torque * Math_Sign(omega);
}

/**
 * @brief 计算单轮高配版阻力补偿扭矩 (内部调用)
 */
static float Calc_High_End_Single_Wheel(float omega, float a0, float a1, float a2, float cont_th, float limit_th)
{
    float abs_omega = fabsf(omega);
    float torque = 0.0f;

    if (abs_omega <= cont_th && cont_th > 0.0f)
    {
        // 零点振荡：一阶连续化
        // 1. 先算出连续化边界点 cont_th 处的前馈扭矩值
        float t_boundary = a0 + a1 * cont_th + a2 * cont_th * cont_th;
        // 2. 线性比例贴合
        torque = (t_boundary / cont_th) * abs_omega;
    }
    else if (abs_omega >= limit_th)
    {
        // 目标值太大发散：前馈限幅
        torque = a0 + a1 * limit_th + a2 * limit_th * limit_th;
    }
    else
    {
        // 正常二次拟合曲线段：T = a0 + a1*|w| + a2*w^2
        torque = a0 + a1 * abs_omega + a2 * abs_omega * abs_omega;
    }

    return torque * Math_Sign(omega);
}

/**
 * @brief 阻力扭矩补偿——低配版
 *
 * @param config 低配版阻力补偿物理参数
 * @param target_omega 底盘逆运动学解算出的四轮目标角速度
 * @param comp_torque 解算后输出的四轮阻力补偿扭矩 (注意: 最终需乘以扭矩转电流系数才能输入给电机)
 */
void Calculate_Resistance_Compensation_Low_End(const Struct_Resistance_Config_Low* config,
                                               const Struct_Wheel_Velocity* target_omega,
                                               Struct_Wheel_Torque* comp_torque)
{
    comp_torque->LF = Calc_Low_End_Single_Wheel(target_omega->LF, config->t,
                      config->continuous_threshold);
    comp_torque->LB = Calc_Low_End_Single_Wheel(target_omega->LB, config->t,
                      config->continuous_threshold);
    comp_torque->RB = Calc_Low_End_Single_Wheel(target_omega->RB, config->t,
                      config->continuous_threshold);
    comp_torque->RF = Calc_Low_End_Single_Wheel(target_omega->RF, config->t,
                      config->continuous_threshold);
}

/**
 * @brief 阻力扭矩补偿——高配版
 *
 * @param config 高配版阻力补偿曲线参数及限幅阈值
 * @param target_omega 底盘逆运动学解算出的四轮目标角速度
 * @param comp_torque 解算后输出的四轮阻力补偿扭矩 (注意: 最终需乘以扭矩转电流系数才能输入给电机)
 */
void Calculate_Resistance_Compensation_High_End(const Struct_Resistance_Config_High* config,
                                                const Struct_Wheel_Velocity* target_omega,
                                                Struct_Wheel_Torque* comp_torque)
{
    comp_torque->LF = Calc_High_End_Single_Wheel(target_omega->LF, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
    comp_torque->LB = Calc_High_End_Single_Wheel(target_omega->LB, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
    comp_torque->RB = Calc_High_End_Single_Wheel(target_omega->RB, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
    comp_torque->RF = Calc_High_End_Single_Wheel(target_omega->RF, config->a0, config->a1, config->a2,
                      config->continuous_threshold, config->limit_threshold);
}



