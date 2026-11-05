/**
 * @file  hexapod.c
 * @brief 硬件无关的六足机器人三自由度逆运动学（IK）与步态相位生成器。
 *
 * 在双板通信六足机器人系统（STM32F103VET6 + ESP32-S3，ESP32 经 UART4 下发命令）中，
 * 本文件是"纯算法层"：不依赖任何 HAL/舵机驱动/调度器，可整体移植到其他 MCU 或在
 * PC 上仿真验证。上层 app_motion.c（仅舵机任务）每个 20ms 周期依次调用：
 *   - hex_feet()：按步态相位表生成 6 条腿的足端目标位置（摆动/支撑轨迹 + 机身平移/转向）；
 *   - hex_pose()：对全部足端位置做整机 IK，换算成 18 路舵机目标角度；
 *   - hex_ik()：单腿三自由度逆解（由 hex_pose 内部调用）。
 * 坐标约定：
 *   - 机身坐标系：X 向前、Y 向左、Z 向下，单位 mm；
 *   - 每条腿的局部坐标系：X 沿该腿安装角方向水平向外、Z 向下；
 *     hex_ik 的足端输入/输出均在腿局部坐标系中。
 */
#include "hexapod.h"
#include <math.h>
#include <stddef.h>
/* 角度->弧度换算系数：pi/180 */
#define HEX_RAD 0.01745329252f
/* 弧度->角度换算系数：180/pi */
#define HEX_DEG 57.29577951f
/* Explicit comparison rejects NaN and infinity without fast-math assumptions. */
/* 用显式双边比较而非 isnan()/isinf()：NaN 与任何数比较均为假，因此 NaN 和无穷大
 * 都会被 n>=lo && n<=hi 自然拒绝，且不依赖编译器浮点行为（-ffast-math 下依然可靠）。 */
static bool bounded(float n, float lo, float hi) { return n >= lo && n <= hi; }
/**
 * @brief 单腿三自由度逆运动学（IK）：由腿局部坐标系的足端位置解出三个关节角。
 *
 * 腿结构为经典 coxa-femur-tibia（基节-股节-胫节）三段：
 *   关节 0（coxa）：水平面内旋转，决定整条腿在水平面上的指向；
 *   关节 1（femur）：在腿的竖直平面内俯仰，负责抬腿/支撑；
 *   关节 2（膝）：股节与胫节的夹角，由余弦定理直接求出。
 * 求解步骤（先在腿局部坐标系中完成，最后统一转成角度制）：
 *   1) radial = sqrt(x^2+y^2)：coxa 转轴到足端的水平距离；
 *   2) a[0] = atan2(y,x)：coxa 角，使腿的竖直平面恰好包含足端；
 *   3) r = radial - coxa_mm：减去基节长度后得到膝关节到足端的水平距离，
 *      把三维问题降维成腿竖直平面内的平面三角形问题；
 *   4) d2 = r^2 + z^2：膝关节到足端的距离平方；对"膝关节-足端-踝"三角形用余弦定理：
 *        cos(knee) = (d2 - femur^2 - tibia^2) / (2*femur*tibia)
 *      knee 为股节与胫节的内夹角（knee=pi 即腿完全伸直）；
 *   5) a[1] = atan2(z,r) - atan2(tibia*sin(knee), femur + tibia*cos(knee))：
 *      前项是"膝->足端"连线相对水平的俯角，后项是股节与该连线的夹角
 *      （由膝->足端向量沿股节方向/垂直方向的分量求得），差值即股节绝对俯仰角；
 *   6) a[2] = knee：膝关节角。
 * @param c 机器人配置（coxa_mm/femur_mm/tibia_mm 三段长度，单位 mm）
 * @param p 足端在腿局部坐标系中的目标位置（mm；X 向外、Z 向下，z>0 表示足端在髋下方）
 * @param a 输出：三个关节角（度）。注意这是"数学关节角"而非舵机命令角，
 *          两者的换算（安装方向、零点偏置）由 hex_pose() 完成。
 * @retval true  求解成功
 * @retval false 输入非法 / 足端离身体太近 / 超出工作空间（够不到、折叠死或完全伸直奇异）
 * @note 由 hex_pose() 每 20ms 周期对 6 条腿各调用一次；纯数学函数，无副作用。
 */
bool hex_ik(const hex_config_t *c, hex_point_t p, float a[3])
{
    float radial, r, d2, cosine, knee;
    /* 输入合法性检查：足端坐标限 ±1000mm；z 至少 0.1mm（Z 向下为正，足端必须严格
     * 位于髋关节下方，z=0 会使后续 atan2 退化）；三段腿长限 1~300mm，用于拒绝未校准
     * 或单位写错（如误填米）的配置。任一项失败即返回 false，防止 NaN 传播到舵机输出。 */
    if (c == NULL || a == NULL || !bounded(p.x, -1000, 1000) ||
        !bounded(p.y, -1000, 1000) || !bounded(p.z, 0.1f, 1000) ||
        !bounded(c->coxa_mm, 1, 300) || !bounded(c->femur_mm, 1, 300) ||
        !bounded(c->tibia_mm, 1, 300)) { return false; }
    radial = sqrtf(p.x*p.x + p.y*p.y);   /* 髋关节（coxa 转轴）到足端的水平距离 (mm) */
    r = radial - c->coxa_mm;             /* 膝关节到足端的水平距离 (mm) */
    /* r<=0.1mm 表示足端收回到身体轴线附近，femur 俯角 atan2(z,r) 数值病态，
     * 且物理上是不可达/无意义姿态，直接拒绝。 */
    if (r <= 0.1f) { return false; }
    d2 = r*r + p.z*p.z;                  /* 膝到足端距离的平方 d^2 = r^2 + z^2 (mm^2) */
    /* 余弦定理：对边长为 femur、tibia、d 的三角形，求 femur 与 tibia 的夹角（膝内角）。 */
    cosine = (d2 - c->femur_mm*c->femur_mm - c->tibia_mm*c->tibia_mm) /
             (2.0f*c->femur_mm*c->tibia_mm);
    /* |cosine|>0.9999 意味着：d 超出 femur+tibia（够不到）、小于 |femur-tibia|（折叠死），
     * 或腿接近完全伸直（运动学奇异点，微小扰动会引起关节角剧烈跳变）——一律拒绝，
     * 让上层进入故障保持而不是输出危险角度。 */
    if (!bounded(cosine, -0.9999f, 0.9999f)) { return false; }
    knee = acosf(cosine);                /* 膝内角（弧度），knee=pi 对应腿完全伸直 */
    a[0] = atan2f(p.y, p.x)*HEX_DEG;     /* coxa 水平摆角（度）：腿竖直平面的方位角 */
    /* femur 俯仰角（度）= 膝->足端连线的俯角 - 股节与该连线的夹角。
     * 第二个 atan2 的分量来自向量合成：膝->足端向量在股节方向上的投影为
     * femur + tibia*cos(knee)，垂直分量为 tibia*sin(knee)。 */
    a[1] = (atan2f(p.z, r) - atan2f(c->tibia_mm*sinf(knee),
             c->femur_mm + c->tibia_mm*cosine))*HEX_DEG;
    a[2] = knee*HEX_DEG;                 /* 膝关节角（度） */
    return true;
}
/**
 * @brief 校验机器人配置结构体：判断 hexapod_config.h 是否已正确校准且数据自洽。
 *
 * 这是所有运动学函数的守门员（hex_pose/hex_feet 内部都会先调用它）。未校准
 * （calibrated=false）或数据矛盾的配置会在这里被整体拒绝，上层 app_motion.c 据此
 * 回复 "calibrate hexapod_config.h first" 并拒绝站立/行走，防止机器人乱动。
 * 校验内容：
 *   1) calibrated 标志为 true，且中立足端位置 neutral 本身 IK 可解
 *      （即标定时的站姿足端点必须在工作空间内）；
 *   2) 18 个关节：舵机 id 必须 <18 且互不重复（用位掩码 used 查重）；direction 只能
 *      取 ±1（描述舵机正装/反装）；角度限位 min/max 必须落在 [45,135] 度内且
 *      min<=max（与 app_servo 的 45~135 度安全范围一致，防止机械干涉）；
 *      neutral_deg 必须位于自己的限位区间内；
 *   3) 6 条腿：安装角在 ±180 度内，髋关节机身偏移在 ±300mm 内（拒绝单位错误值）。
 * @retval true 配置有效，允许运动
 * @retval false 配置缺失、未校准或任一项数据越界
 */
bool hex_config_valid(const hex_config_t *c)
{
    uint32_t used = 0U;   /* 舵机 id 占用位掩码：bit n 置 1 表示 id n 已被某关节使用 */
    unsigned i;
    float a[3];
    if (c == NULL || !c->calibrated || !hex_ik(c, c->neutral, a)) { return false; }
    for (i=0; i<HEX_JOINTS; ++i) {
        /* 任一条件不满足即整体判无效：id 越界 / id 重复（used 位已置起）/
         * 方向不是 ±1 / 限位不在 45~135 度安全区 / max<min / 中立角超出限位 */
        if (c->id[i] >= HEX_JOINTS || (used & (1UL << c->id[i])) != 0U ||
            (c->direction[i] != 1 && c->direction[i] != -1) ||
            !bounded(c->min_deg[i], 45, 135) || !bounded(c->max_deg[i], c->min_deg[i], 135) ||
            !bounded(c->neutral_deg[i], c->min_deg[i], c->max_deg[i])) { return false; }
        used |= 1UL << c->id[i];   /* 登记该 id 已被占用 */
    }
    for (i=0; i<HEX_LEGS; ++i) {
        /* 腿部几何：安装角 ±180 度内；髋关节在机身坐标系的 X/Y 偏移 ±300mm 内 */
        if (!bounded(c->mount_deg[i], -180, 180) ||
            !bounded(c->hip_x_mm[i], -300, 300) || !bounded(c->hip_y_mm[i], -300, 300)) { return false; }
    }
    return true;
}
/**
 * @brief 整机逆解：6 个足端位置 -> 18 路舵机目标角度（度）。
 *
 * 关键思想：hex_ik 解出的是"数学关节角"，而实际舵机的安装方向和零点因机而异，
 * 因此采用相对"中立姿态"的差分法：
 *   1) 先求中立足端位置 c->neutral 对应的数学关节角 base[]；
 *   2) 对每条腿求目标足端位置的数学关节角 a[]；
 *   3) 关节增量 = a[j]-base[j]，乘上该关节的安装方向符号 direction[]（±1，对应舵机
 *      正装/反装），再叠加到标定好的中立舵机角 neutral_deg[] 上，得到真实舵机角。
 * 这样 neutral_deg 只需是实机在中立足端位置时实测的舵机命令角，与数学零点无关。
 * 结果先写入局部数组 frame[]（按舵机 id 定位），18 个关节全部通过限位检查后才整体
 * 拷贝到 out —— "全有或全无"：任一关节超限就放弃整帧，避免出现半更新姿态导致
 * 机器人扭曲；上层据此进入故障保持。
 * @param c    机器人配置（内部会先做 hex_config_valid 校验）
 * @param feet 6 条腿足端在各自腿局部坐标系的目标位置 (mm)，来自 hex_feet()
 * @param out  输出：18 路舵机目标角度（度），下标即舵机 id，可直接交 app_servo_frame()
 * @retval true  全部关节解算成功且在限位内
 * @retval false 配置无效 / 某足端 IK 无解 / 某关节超出 min_deg~max_deg
 */
bool hex_pose(const hex_config_t *c, const hex_point_t feet[HEX_LEGS], float out[HEX_JOINTS])
{
    float base[3], a[3], frame[HEX_JOINTS], value;
    unsigned leg, j, n;
    /* base[]：中立足端位置对应的数学关节角，作为差分基准 */
    if (feet == NULL || out == NULL || !hex_config_valid(c) || !hex_ik(c, c->neutral, base)) { return false; }
    for (leg=0; leg<HEX_LEGS; ++leg) {
        if (!hex_ik(c, feet[leg], a)) { return false; }   /* 该腿目标足端不可达 -> 整帧作废 */
        for (j=0; j<3; ++j) {
            n = leg*3U+j;   /* 关节全局下标：腿号*3 + 关节号(0=coxa,1=femur,2=膝) */
            /* 舵机角 = 中立舵机角 + 安装方向 * (目标数学角 - 中立数学角) */
            value = c->neutral_deg[n] + c->direction[n]*(a[j]-base[j]);
            /* 限位检查在写入 frame 之前完成，超限立即拒绝整帧（见函数头说明） */
            if (!bounded(value, c->min_deg[n], c->max_deg[n])) { return false; }
            frame[c->id[n]] = value;   /* 按舵机物理 id 落位，而非关节逻辑顺序 */
        }
    }
    for (n=0; n<HEX_JOINTS; ++n) { out[n] = frame[n]; }   /* 全部合法，整体提交 */
    return true;
}
/**
 * @brief 步态相位生成器：根据当前相位与速度指令计算 6 个足端的目标位置（腿局部坐标系）。
 *
 * 每条腿在一个步态周期内重复"摆动（抬起前移）/支撑（着地后蹬）"，不同步态的区别
 * 只在于摆动占比和每条腿的相位偏移：
 *   - Tripod（三角步态）：摆动占周期 1/2，两组对角腿交替摆动（RF/RR/LM 一组、
 *     RM/LF/LR 一组，相位差 0.5）——最快，任一时刻 3 腿支撑；
 *   - Ripple（涟漪步态）：摆动占 1/3，6 腿相位均匀错开，任一时刻最多 2 腿抬起——更稳；
 *   - Wave（波浪步态）：摆动占 1/6，任一时刻只有 1 腿抬起——最慢也最稳。
 * 摆动相轨迹设计（t 为摆动区间内的归一化进度 [0,1)）：
 *   - 水平：travel = t^2*(3-2t) - 0.5，即 smoothstep 函数减 0.5，从 -0.5（行程末端）
 *     平滑过渡到 +0.5（行程前端）。smoothstep 在两端一阶导为 0，保证抬腿/落腿瞬间
 *     水平速度为零，无冲击、不打滑；
 *   - 垂直：height = lift*sin(pi*t)，半正弦拱形抬腿，两端为 0、中点达到 lift (mm)。
 * 支撑相：travel 从 +0.5 线性减小到 -0.5（着地的脚相对机身匀速后移，等效推动机身前进）。
 * 平移与转向的叠加：
 *   - forward/left 为每周期机身平移行程 (mm/cycle)，按 travel 加权施加到足端；
 *   - yaw 为每周期机身转角 (deg/cycle)，采用小角度近似：机身上距机身中心半径为
 *     (bx,by) 的点，在整体旋转 yaw_rad（弧度）时的切向位移约为
 *     (-yaw_rad*by, +yaw_rad*bx)——即 omega x r 的一阶线性化，忽略二阶小量，
 *     因此仅适用于小角度慢速转向；
 *   - 最后把机身坐标系的位移 (dx,dy) 按该腿安装角 mount_deg 旋转到腿局部坐标系，
 *     叠加到该腿的中立足端位置上。
 * @param c       机器人配置（内部先做 hex_config_valid 校验）
 * @param gait    步态类型（HEX_TRIPOD/HEX_RIPPLE/HEX_WAVE）
 * @param phase   全局步态相位 [0,1)，由 app_motion.c 每 20ms 递增 20/2400
 * @param forward 每周期前进行程 (mm)，正为向前，限幅 ±20mm（上层 100% 指令对应 20mm）
 * @param left    每周期横移行程 (mm)，正为向左，限幅 ±20mm
 * @param yaw     每周期转角（度），正为逆时针（左转），限幅 ±5 度
 * @param lift    摆动相抬腿高度 (mm)，限幅 0~15mm（上层固定传 8mm）
 * @param feet    输出：6 条腿足端在腿局部坐标系中的目标位置 (mm)
 * @retval true 生成成功；false 参数越界或配置未校准
 * @note 腿序假定解剖学排列 RF RM RR LF LM LR（下标 0..5），相位偏移表按此编制。
 */
bool hex_feet(const hex_config_t *c, hex_gait_t gait, float phase,
              float forward, float left, float yaw, float lift, hex_point_t feet[HEX_LEGS])
{
    /* Assumed anatomical order RF RM RR LF LM LR. Tripods RF/RR/LM and RM/LF/LR. */
    /* 每条腿的相位偏移表 offset[步态][腿]（单位：周期的比例，腿序 RF RM RR LF LM LR）：
     * Tripod 行：RF/RR/LM=0，RM/LF/LR=0.5，两组对角腿各半周期交替摆动；
     * Ripple 行：六腿依次错开 1/6 周期（0, 1/3, 2/3, 1/2, 5/6, 1/6），配合 1/3 的
     *            摆动占比，保证任一时刻最多 2 腿抬起；
     * Wave 行：  依次错开 1/6 周期（0, 1/6, ..., 5/6），每腿只摆动 1/6 周期，
     *            任一时刻恰好 1 腿抬起、5 腿支撑。 */
    static const float offset[3][6] = {
        {0, .5f, 0, .5f, 0, .5f},
        {0, .33333333f, .66666667f, .5f, .83333333f, .16666667f},
        {0, .16666667f, .33333333f, .5f, .66666667f, .83333333f}
    };
    /* 各步态摆动时间占整周期的比例：tripod=1/2、ripple=1/3、wave=1/6，其余为支撑时间 */
    static const float swing[3] = {.5f, .33333333f, .16666667f};
    float p, t, travel, height, ca, sa, bx, by, dx, dy;
    unsigned i;
    /* 参数检查：步态枚举合法、相位必须在 [0,1)（NaN 同样被拒绝）、前后/横移限
     * ±20mm/周期、转向限 ±5 度/周期、抬腿限 0~15mm——与 hexapod.h 声明的量纲一致，
     * 越界直接返回 false，由上层进入故障保持。 */
    if (feet == NULL || !hex_config_valid(c) || (unsigned)gait > (unsigned)HEX_WAVE ||
        !(phase >= 0 && phase < 1) || !bounded(forward, -20, 20) ||
        !bounded(left, -20, 20) || !bounded(yaw, -5, 5) || !bounded(lift, 0, 15)) { return false; }
    for (i=0; i<HEX_LEGS; ++i) {
        p = phase + offset[gait][i];     /* 该腿局部相位 = 全局相位 + 该腿相位偏移 */
        if (p >= 1) { p -= 1; }          /* 回绕到 [0,1)，保持周期连续 */
        height = 0;                      /* 抬腿高度 (mm)，支撑相恒为 0（脚着地） */
        if (p < swing[gait]) {
            /* --- 摆动相：腿抬起，从行程末端(-0.5)平滑移到行程前端(+0.5) --- */
            t = p/swing[gait];                     /* 摆动区间内归一化进度 [0,1) */
            travel = t*t*(3-2*t)-.5f;              /* smoothstep(t)-0.5：两端速度为 0 的 S 形位移曲线 */
            height = lift*sinf(3.141592654f*t);    /* 半正弦抬腿：中点最高 lift，起落点归零避免磕碰 */
        } else {
            /* --- 支撑相：腿着地，相对机身线性后移，推动机身前进 --- */
            t = (p-swing[gait])/(1-swing[gait]);   /* 支撑区间内归一化进度 [0,1) */
            travel = .5f-t;                        /* 从 +0.5 线性回到 -0.5（归一化行程） */
        }
        ca=cosf(c->mount_deg[i]*HEX_RAD); sa=sinf(c->mount_deg[i]*HEX_RAD); /* 该腿安装角的 cos/sin，用于机身<->腿局部坐标系旋转 */
        /* (bx,by)：机身中心到该腿中立足端点的位置向量（机身坐标系，mm）——
         * 由髋偏移加上按安装角旋转后的中立足端坐标得到，是转向的"旋转臂"。 */
        bx=c->hip_x_mm[i]+ca*c->neutral.x-sa*c->neutral.y;
        by=c->hip_y_mm[i]+sa*c->neutral.x+ca*c->neutral.y;
        /* 小角度转向近似：半径 (bx,by) 处旋转 yaw_rad 的切向位移为 (-yaw_rad*by, +yaw_rad*bx)，
         * 与平移指令叠加后统一乘以 travel 加权——支撑相 travel 反向，正是脚蹬地推机身。
         * yaw*HEX_RAD 先把度/周期转成弧度/周期。 */
        dx=(forward-yaw*HEX_RAD*by)*travel;   /* 机身坐标系 X（前向）位移分量 (mm) */
        dy=(left+yaw*HEX_RAD*bx)*travel;      /* 机身坐标系 Y（左向）位移分量 (mm) */
        /* 将机体系位移 (dx,dy) 反向旋转 mount_deg 到该腿局部坐标系，叠加到中立足端位置 */
        feet[i].x=c->neutral.x+ca*dx+sa*dy;
        feet[i].y=c->neutral.y-sa*dx+ca*dy;
        feet[i].z=c->neutral.z-height;   /* Z 向下为正：抬腿即 z 减小，故减去 height */
    }
    return true;
}
