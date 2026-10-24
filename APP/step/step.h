#ifndef __STEP_H
#define __STEP_H

#include "system.h"

/* ============================ 计步算法参数 ============================
 * 算法链路（对应实训教材 3.4 节）：
 *   三轴加速度 -> 合加速度 mag -> 滑动平均滤波 -> 阈值上升沿 -> 死区时间 -> 步数 +1
 *
 * STEP_FS              采样率 50Hz（20ms 一个样本）。人走路 1~3Hz、跑步 3~4Hz，
 *                      50Hz 下每个步态周期有 15~25 个样本，峰值位置不会漏；
 *                      同时 50Hz 远高于 300ms 死区的要求（死区内还有 15 个样本）。
 * STEP_FILTER_WIN      滑动平均窗口 5 点 = 100ms。手抖、路面颠簸是 5Hz 以上的
 *                      高频毛刺，100ms 窗把它们压下去，而 1~2Hz 的步态波峰只衰减约 6%。
 * STEP_TH_MG           峰值阈值 1200mg（1.2g）。静止时合加速度约 1000mg（1g），
 *                      走路时波峰 1300~2500mg，取 1.2g 在静止基线和步行波峰之间留足余量。
 * STEP_REARM_MG        回落阈值 1050mg（TH-150）。必须跌破它才允许识别下一次峰值，
 *                      相当于施密特迟滞，防止信号在 TH 附近抖动被连续计成多步。
 * STEP_DEAD_TIME_MS    死区时间 300ms。人体最快步频约 200 步/分钟（3.3 步/秒，
 *                      间隔 300ms），因此 300ms 内不可能出现第二步，超出的峰值一律忽略。
 * STEP_MAG_MIN/MAX     样本合理区间 300~6000mg。±16g 量程理论上限 16000mg，
 *                      人体运动不会超过 6000mg；接近 0 说明是 I2C 全 0 / 自由落体异常值。
 * STEP_ERR_LIMIT       连续 25 个异常样本（50Hz 下 0.5s）判定传感器故障并复位检测器。
 * STEP_CADENCE_NUM     步频用最近 4 次步间隔做平均，响应快且不抖。
 * STEP_MOVING_MS       3000ms 内没有新步判定为静止，步频输出 0。
 * STEP_MAX_INTERVAL_MS 步间隔上限 2000ms。超过说明中间停顿过，这一拍不计入步频，
 *                      但仍然算一步（与教材"峰值 + 死区"两个条件一致）。
 * ==================================================================== */
#define STEP_FS					50
#define STEP_SAMPLE_MS			(1000 / STEP_FS)	/* 20ms */

#define STEP_FILTER_WIN			5
#define STEP_TH_MG				1200
#define STEP_REARM_MG			(STEP_TH_MG - 150)
#define STEP_DEAD_TIME_MS		300

#define STEP_MAG_MIN			300
#define STEP_MAG_MAX			6000
#define STEP_ERR_LIMIT			25

#define STEP_CADENCE_NUM		4
#define STEP_MOVING_MS			3000
#define STEP_MAX_INTERVAL_MS	2000

/* 传感器状态 */
#define STEP_STATUS_OK			0
#define STEP_STATUS_ERROR		1		/* 连续异常样本，传感器/总线故障 */

/* ============================ 接口 ============================
 * 数据来源：MPU6050 合加速度（由 STEP_Poll 内部采样，也可用 STEP_FeedMG 外部喂入）
 * 触发条件：主循环每轮调用 STEP_Poll()，内部按 20ms 节拍采样
 * ============================================================== */
void STEP_Init(void);				/* 初始化 TIM2 时间基准并清零 */
void STEP_Reset(void);				/* 步数清零 + 检测器复位（换人/重新开始计步） */

u8   STEP_Poll(void);				/* 主循环调用，返回 1 表示本次新增了一步 */
u8   STEP_FeedMG(s16 mag);			/* 外部已有合加速度数据时直接喂入，返回 1 表示计一步 */

u32  STEP_GetCount(void);			/* 累计步数 */
u16  STEP_GetCadence(void);			/* 步频（步/分钟），静止时为 0 */
s16  STEP_GetFilteredMG(void);		/* 滤波后的合加速度（mg） */
s16  STEP_GetRawMG(void);			/* 原始合加速度（mg） */
u8   STEP_IsMoving(void);			/* 1=运动中（3 秒内有步） */
u8   STEP_GetStatus(void);			/* STEP_STATUS_OK / STEP_STATUS_ERROR */

#endif
