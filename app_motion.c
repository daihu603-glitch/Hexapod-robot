/**
 * @file  app_motion.c
 * @brief 六足机器人步态运动控制应用层：命令解析 + 运动状态机 + 周期性步态解算调度。
 *
 * 在双板通信系统（STM32F103VET6 + ESP32-S3）中，ESP32 通过 UART4 发送文本命令
 * （如 "gait walk tripod 50 0 0"）。通信任务只负责把命令文本入队，本文件的所有
 * 函数只能在舵机任务（20ms 周期）中调用——舵机任务独占全部舵机输出，避免多任务
 * 竞争 I2C 总线和运动状态。解算链路：
 *   app_motion_process() -> hex_feet()（步态相位->足端位置）-> hex_pose()（整机 IK
 *   ->18 路关节角）-> 每关节限速 -> app_servo_frame()（I2C 写 PCA9685 输出 PWM）。
 * 状态机（motion_state_t）：
 *   IDLE  ：从未站立/已取消，不产生任何输出；
 *   STAND ：收到 "gait stand"，把校准好的中立角写一帧后转入 HOLD；
 *   WALK  ：周期性步态解算，每 20ms 推进相位并发送一帧；
 *   HOLD  ：冻结保持——保留最后一帧 PWM，不动也不断电（继续支撑机身）；
 *   FAULT ：锁存故障（IK 无解/关节超限/I2C 失败），拒绝一切运动命令，
 *           必须断电检查硬件后重新上电才能解除。
 * 安全机制：
 *   - 1000ms 行走租约：walk 命令必须周期刷新，超时自动转 HOLD，防止 ESP32
 *     断连或卡死后机器人失控行走；
 *   - 目标限速：每个关节每 20ms 最多变化 1 度（最大角速度约 50 度/秒），防止跳变；
 *   - 无位置反馈：站立也是开环直达目标角；stop 与租约超时都保持最后 PWM，
 *     不会自动回站姿或断电。
 */
#include "app_motion.h"
#include "hexapod_config.h"
#include "app_servo.h"
#include "board_io.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* 运动控制周期：20ms，与舵机任务的调度节拍一致（50Hz），也是限速的时间基准 */
#define MOTION_PERIOD_MS 20U
/* 行走租约：1000ms。ESP32 发出 walk 后必须在 1000ms 内再次刷新，否则自动停走（冻结保持），
 * 防止上位机断连/卡死导致机器人失控。 */
#define MOTION_LEASE_MS 1000U
/* 完整步态周期：2400ms。相位每 20ms 周期递增 20/2400，120 个控制周期走完一个步态循环 */
#define MOTION_CYCLE_MS 2400U
/* 运动状态机状态，含义见文件头注释 */
typedef enum { MOTION_IDLE, MOTION_STAND, MOTION_WALK, MOTION_HOLD, MOTION_FAULT } motion_state_t;
static motion_state_t state;               /* 当前状态；静态变量初始为 0，即 MOTION_IDLE */
static hex_gait_t gait;                    /* 当前步态类型（tripod/ripple/wave），行走中禁止切换 */
static uint32_t last_step, last_command;   /* last_step：上次解算时刻（周期与死线监测）；last_command：上次 walk 命令时刻（租约计时） */
static float phase;                        /* 全局步态相位 [0,1)，仅在 WALK 状态推进 */
static float current[HEX_JOINTS];          /* 最近一次已下发的 18 路舵机角度（度），按舵机 id 索引；是限速的起点 */
static int forward_pct, left_pct, yaw_pct; /* 速度百分比 -100..100（前/横移/转向），在 process 中换算为每周期 mm 或度 */
/* 取消一切运动，回到 IDLE（例如急停或模式切换时由上层调用）。
 * 只停止状态机推进，不主动断电：舵机 PWM 保持最后一帧。 */
void app_motion_cancel(void) { state=MOTION_IDLE; }
/* Strict bounded signed integers, no sscanf overflow or trailing fields. */
/**
 * @brief 从字符串游标处解析一个严格有界的带符号十进制整数（速度百分比用）。
 *
 * 刻意不用 sscanf/atoi：它们对整数溢出不报错，且容忍尾随非法字符。本实现的约束：
 *   - 可选一个 '-' 号，随后最多 3 位数字（同时杜绝乘法累积溢出）；
 *   - 绝对值必须 <=100（速度百分比的物理上限，对应最大行程/转速）；
 *   - 成功时把游标推进到数字之后，由调用方继续校验分隔符（空格/结束符），
 *     因此任何多余字段都会被拒绝。
 * @param cursor 入/出参：当前解析位置；仅在解析成功时前移
 * @param value  输出：解析结果，范围 -100..100
 * @retval 1 成功；0 失败（无数字 / 超过 3 位 / 绝对值 >100）
 */
static int motion_number(const char **cursor, int *value)
{
    const char *p=*cursor;
    int sign=1, n=0;
    unsigned digits=0;
    if (*p=='-') { sign=-1; ++p; }              /* 至多一个负号 */
    while (*p>='0' && *p<='9') {
        if (++digits>3U) { return 0; }          /* 超过 3 位数字：拒绝，也防止 int 溢出 */
        n=n*10+(*p++-'0');                      /* 逐位累积数值 */
    }
    if (digits==0U || n>100) { return 0; }      /* 没有数字或绝对值超过 100% 上限：拒绝 */
    *value=sign*n; *cursor=p; return 1;
}
/**
 * @brief 解析 walk 命令的参数部分："<tripod|ripple|wave> <x> <y> <yaw>"。
 *
 * 先匹配步态名（匹配长度决定指针偏移量），再依次解析三个带符号整数：前两个之后
 * 必须恰好是一个空格，最后一个之后必须是字符串结束符——字段缺失或多余都拒绝，
 * 防止歧义命令被执行。
 * @param p    命令文本（已跳过 "gait walk " 前缀）
 * @param kind 输出：步态类型（HEX_TRIPOD/HEX_RIPPLE/HEX_WAVE）
 * @param v    输出：三个速度百分比，v[0]=前进、v[1]=横移、v[2]=转向，各 -100..100
 * @retval 1 解析成功且格式完整；0 步态名不识别或数字/分隔符不合法
 */
static int motion_walk_parse(const char *p, hex_gait_t *kind, int v[3])
{
    unsigned i;
    if (strncmp(p,"tripod ",7)==0) { *kind=HEX_TRIPOD; p+=7; }
    else if (strncmp(p,"ripple ",7)==0) { *kind=HEX_RIPPLE; p+=7; }
    else if (strncmp(p,"wave ",5)==0) { *kind=HEX_WAVE; p+=5; }
    else { return 0; }                          /* 步态名不匹配：拒绝 */
    for (i=0;i<3;++i) {
        if (!motion_number(&p,&v[i])) { return 0; }   /* 解析第 i 个速度整数 */
        if (i<2U && *p++!=' ') { return 0; }          /* 前两个数之后必须是单个空格分隔符 */
    }
    return *p=='\0';                            /* 最后一个数之后必须正好是字符串结束 */
}
/**
 * @brief 步态命令入口：解析来自 ESP32（经 UART4）的文本命令，执行状态机转换并写回复。
 *
 * 由舵机任务从通信命令队列取出后调用（只能舵机任务调用，见 app_motion.h 的约定）。
 * 支持的命令与状态转换：
 *   "gait status"                            -> 查询状态/校准标志/租约参数（不改变状态）；
 *   "gait stop"                              -> WALK/STAND 转 HOLD（冻结保持最后 PWM）；
 *   "gait stand"                             -> 写一帧校准中立角（需已校准且非 FAULT），转 STAND；
 *   "gait walk <tripod|ripple|wave> x y yaw" -> 启动或调速行走（需已站立），刷新租约；
 *   其他                                     -> 回复用法提示。
 * 注意 status 和 stop 在未校准时依然可用：stop 必须任何时候都能执行（安全优先），
 * status 用于排查校准状态；其余运动命令都被校准检查拦截。
 * @param text  完整命令文本（NUL 结尾）
 * @param reply 回复缓冲区，写入的文本经 UART4 发回 ESP32
 * @param cap   回复缓冲区容量（字节）
 * @param now   当前系统毫秒时间戳（uint32），用于租约与控制周期计时
 */
void app_motion_command(const char *text, char *reply, size_t cap, uint32_t now)
{
    hex_gait_t kind;
    int v[3];
    unsigned i;
    if (text==NULL || reply==NULL || cap==0U) { return; }   /* 防御性检查：无缓冲区则静默丢弃 */
    if (strcmp(text,"gait status")==0) {
        /* 状态查询：state 为状态机枚举值，calibrated 为配置校准标志（0=拒绝站立/行走） */
        (void)snprintf(reply,cap,"gait state=%u calibrated=%u lease=1000ms",(unsigned)state,
                       (unsigned)hex_config_valid(&hex_robot_config)); return;
    }
    if (strcmp(text,"gait stop")==0) {
        /* 停止 = 转 HOLD 冻结而非断电：系统没有位置反馈，突然断 PWM 会让机器人瘫倒；
         * HOLD 保持最后一帧输出继续支撑。IDLE/HOLD/FAULT 下收到 stop 无需转换。 */
        if (state==MOTION_WALK || state==MOTION_STAND) { state=MOTION_HOLD; }
        (void)snprintf(reply,cap,"ok gait hold (PWM retained)"); return;
    }
    /* 校准守门：从这里开始的所有运动命令都要求 hexapod_config.h 已实测校准，
     * 否则示例配置驱动真实机器人会产生不可预测的动作。 */
    if (!hex_config_valid(&hex_robot_config)) {
        (void)snprintf(reply,cap,"error gait: calibrate hexapod_config.h first"); return;
    }
    if (strcmp(text,"gait stand")==0) {
        /* FAULT 是锁存故障：出现过错（IK 超限/I2C 失败）后必须先断电检查硬件再上电，
         * 软件层面拒绝直接复位，避免带着故障的机器重新站起来。 */
        if (state==MOTION_FAULT) {
            (void)snprintf(reply,cap,"error gait fault: off and inspect hardware first"); return;
        }
        /* First enable targets calibrated neutral directly: support robot first.
         * No position feedback exists; do not claim a ramp from actual position. */
        /* 首次使能直接以校准中立角为目标：先让机器人站稳支撑。没有位置反馈，
         * 无法从"实际位置"做斜坡逼近，这里只预装目标角，由 process() 发出一帧。 */
        for(i=0;i<HEX_JOINTS;++i) { current[hex_robot_config.id[i]]=hex_robot_config.neutral_deg[i]; }
        /* 把 last_step 回拨一个周期，保证 process() 下一次 tick 立即发送站立帧，不用再等 20ms */
        state=MOTION_STAND; phase=0; last_step=now-MOTION_PERIOD_MS;
        (void)snprintf(reply,cap,"ok gait stand requested"); return;
    }
    /* 前缀 + 参数解析：任何不匹配 "gait walk <步态> x y yaw" 的文本都回复用法提示 */
    if (strncmp(text,"gait walk ",10)!=0 || !motion_walk_parse(text+10,&kind,v)) {
        (void)snprintf(reply,cap,"error gait: stand|stop|status|walk <tripod|ripple|wave> x y yaw"); return;
    }
    /* 行走前置条件：必须已站立（STAND）、正在行走（WALK）或冻结保持（HOLD）；
     * IDLE（从未站立）和 FAULT（锁存故障）下拒绝行走。 */
    if (state!=MOTION_STAND && state!=MOTION_WALK && state!=MOTION_HOLD) {
        (void)snprintf(reply,cap,"error gait: stand first"); return;
    }
    /* 行走中禁止直接切换步态类型：不同步态的相位偏移表互不连续，强行切换会导致
     * 各腿相位跳变（可能多腿同时抬起而失稳），必须先 stop 再换新步态。 */
    if (state==MOTION_WALK && kind!=gait) {
        (void)snprintf(reply,cap,"error gait: stop before changing pattern"); return;
    }
    /* 从 STAND/HOLD 起步时回拨计时让首帧立即执行；已在 WALK 中（调速）则保持原节拍，
     * 避免重置 last_step 打乱 20ms 周期节奏造成相位跳变。 */
    if (state!=MOTION_WALK) { last_step=now-MOTION_PERIOD_MS; }
    /* 记录步态与三路速度百分比，并把租约计时刷新到当前时刻（1000ms 内须再次刷新） */
    gait=kind; forward_pct=v[0]; left_pct=v[1]; yaw_pct=v[2]; last_command=now;
    /* 三路速度全为 0 等价于原地停走：直接进 HOLD，不再空跑步态相位 */
    state=(v[0]==0 && v[1]==0 && v[2]==0)?MOTION_HOLD:MOTION_WALK;
    (void)snprintf(reply,cap,"ok gait command accepted (refresh <1000ms)");
}
/**
 * @brief 进入锁存故障状态并记录原因（经 board_io_log 输出到调试通道）。
 *
 * IK 无解、关节超限、I2C 发送失败都会走到这里：状态一旦锁存为 FAULT，
 * app_motion_command 将拒绝所有运动命令，PWM 保持最后一帧（app_servo_frame 内部
 * 已尽力尝试关断输出），直到用户断电检查硬件后重新上电。这样设计是为了避免
 * 瞬时解算/总线故障后被自动恢复，导致机器人在不可预知的姿态下突然动作。
 */
static void motion_fault(const char *reason)
{
    state=MOTION_FAULT;
    board_io_log(reason);
}
/**
 * @brief 周期性运动解算：由舵机任务每 tick（名义 20ms）调用一次。
 *
 * 处理流程：
 *   1) 租约检查：WALK 状态下距上次 walk 命令超过 1000ms 则自动转 HOLD（防失控）；
 *   2) 周期闸门：距上次解算不足 20ms 直接返回（限速与相位推进都以 20ms 为基准）；
 *   3) 死线监测：距上次解算超过 100ms（错过 5 个周期，任务被饿死或系统卡顿）则
 *      放弃本帧转 HOLD——恢复后立即大步解算会造成相位/角度跳变，宁可冻结；
 *   4) STAND：发送一帧预装的中立角，成功即转 HOLD（站立完成，静态保持）；
 *   5) WALK：推进相位 -> hex_feet 生成足端轨迹 -> hex_pose 整机 IK -> 每关节
 *      ±1 度限速 -> app_servo_frame 经 I2C 写两块 PCA9685 输出 18 路 PWM。
 * @param now 当前系统毫秒时间戳（uint32；所有计时用无符号减法，天然抗回绕）
 */
void app_motion_process(uint32_t now)
{
    hex_point_t feet[HEX_LEGS];
    float target[HEX_JOINTS], delta;
    unsigned i;
    /* 行走租约超时：ESP32 可能已断连或卡死，自动冻结保持，防止机器人无限走下去 */
    if (state==MOTION_WALK && (uint32_t)(now-last_command)>=MOTION_LEASE_MS) {
        state=MOTION_HOLD; board_io_log("[GAIT] timeout: holding last PWM\r\n");
    }
    /* IDLE/HOLD/FAULT 不产生输出：HOLD 保持既有 PWM，FAULT 锁存待断电复位 */
    if (state!=MOTION_WALK && state!=MOTION_STAND) { return; }
    /* 未到 20ms 控制周期：跳过本 tick */
    if ((uint32_t)(now-last_step)<MOTION_PERIOD_MS) { return; }
    /* 超过 100ms（5 个周期）没有解算：调度异常，转 HOLD 保护，避免恢复时的相位跳变 */
    if ((uint32_t)(now-last_step)>100U) {
        state=MOTION_HOLD; board_io_log("[GAIT] deadline missed: hold\r\n"); return;
    }
    last_step=now;   /* 记录本帧解算时刻，作为下次周期/死线判断的基准 */
    if (state==MOTION_STAND) {
        /* 站立：current[] 已在 stand 命令中预装为校准中立角，这里只需发一帧。
         * I2C 失败则锁存故障；成功后转 HOLD 静态保持（仍持续供电支撑，不断 PWM）。 */
        if (!app_servo_frame(current)) { motion_fault("[GAIT] I2C fault\r\n"); }
        else { state=MOTION_HOLD; }
        return;
    }
    /* 行走解算：速度百分比换算为每周期物理量——
     * 0.2f mm/(周期·%)：100% 对应 20mm/周期 的前进/横移行程；
     * 0.05f 度/(周期·%)：100% 对应 5 度/周期 的机身转角（小角度转向）；
     * 8：抬腿高度固定 8mm（hex_feet 允许 0~15mm，取 8mm 兼顾越障与稳定）。
     * hex_feet 或 hex_pose 任一失败（目标不可达/关节超限）都进入锁存故障并保持 PWM。 */
    if (!hex_feet(&hex_robot_config,gait,phase,forward_pct*.2f,left_pct*.2f,yaw_pct*.05f,8,feet) ||
        !hex_pose(&hex_robot_config,feet,target)) {
        motion_fault("[GAIT] unreachable/limit: holding last PWM\r\n"); return;
    }
    /* 目标限速：每关节每 20ms 周期最多变化 ±1 度（约 50 度/秒），
     * 平滑速度指令突变，降低舵机冲击电流与机械抖动。 */
    for(i=0;i<HEX_JOINTS;++i) {
        delta=target[i]-current[i];
        if(delta>1) { delta=1; } else if(delta< -1) { delta=-1; }   /* 增量钳位到 ±1 度 */
        current[i]+=delta;
    }
    /* 下发一帧 18 路角度；失败时 app_servo_frame 内部已尽力尝试关断 PWM，随后锁存故障 */
    if (!app_servo_frame(current)) { motion_fault("[GAIT] I2C fault: off attempted\r\n"); return; }
    /* 推进全局步态相位：每周期增加 20ms/2400ms ≈ 0.00833，120 个控制周期走完一整圈；
     * 到达 1 时回绕到 0，保持相位连续。 */
    phase+=(float)MOTION_PERIOD_MS/MOTION_CYCLE_MS;
    if(phase>=1) { phase-=1; }
}
