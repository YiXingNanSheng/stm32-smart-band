#ifndef __HR_H
#define __HR_H

#include "system.h"

/* ============================ 心率检测参数 ============================
 * 采样率：100Hz / 4 次平均 = 25Hz
 * IBI 合理范围：30~125 bpm
 * ==================================================================== */
#define HR_FS			25			/* 采样率 Hz */
#define HR_MIN_IBI		12			/* 最短 IBI（样本）0.48s -> 125 bpm */
#define HR_MAX_IBI		50			/* 最长 IBI（样本）2.00s -> 30 bpm */
#define HR_FINGER_DC	50000		/* 直流分量超过此值认为手指在位 */
#define HR_IBI_AVG_NUM	4			/* IBI 滑动平均窗口 */

#define HR_WAVE_LEN		128			/* 波形环形缓冲深度（25Hz 下约 5 秒） */

void HR_Init(void);
void HR_Feed(u32 ir);				/* 直接喂 25Hz 有效样本（保持原有用法） */
void HR_Feed100Hz(u32 ir);			/* 喂 100Hz 原始样本，内部 4 点平均后抽取到 25Hz */
u16  HR_GetRate(void);				/* 心率 bpm，0=尚未测出 */
u16  HR_GetIBI(void);				/* 最近一次 IBI（样本数） */
u8   HR_IsFingerOn(void);			/* 1=手指在位 */
u32  HR_GetDC(void);				/* 当前直流估计值 */
u32  HR_GetPeakMax(void);			/* 当前自适应阈值参考值（调试用） */
u8   HR_IsStable(void);				/* 1=DC 已收敛（约贴上后 6 秒），正在检测 */

/* 界面用：最近 5 秒的交流波形，idx=0 是最旧的一点 */
s16  HR_GetWave(u8 idx);
u8   HR_GetBeat(void);				/* 检测到一次心跳返回 1，读后自动清零 */

#endif
