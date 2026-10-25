#ifndef __SPO2_H
#define __SPO2_H

#include "system.h"

/* ============================ 血氧算法参数（对齐实验指导书） ============================
 * 数据来源：MAX30102 SpO2 模式（红光 660nm + 红外 880nm），
 *           FIFO 关闭硬件平均后为 100Hz 原始采样，红光与红外成对读出
 *
 * SPO2_FS          100Hz。官方算法的最小峰距、6000/T 换算都按 100Hz 设计，
 *                  所以这里必须关掉 FIFO 的 4 次平均，不能再按 25Hz 喂。
 * SPO2_BUF_LEN     500 点 = 5 秒窗口。静息心率 1~1.7Hz，5 秒含 5~10 跳，
 *                  足以估计峰间隔。
 * SPO2_STEP        100 点 = 1 秒。滑窗每次丢最旧 1 秒、补最新 1 秒，
 *                  结果每秒刷新一次，长度始终 500。
 * SPO2_PEAK_DIST   最小峰距 8 点 = 80ms，对应最高 750bpm 的假峰被抑制。
 * SPO2_MAX_PEAKS   一次最多取 15 个峰（5 秒内 15 跳 = 180bpm，够用）。
 * SPO2_MIN_PEAKS   少于 3 个峰无法估计周期，判本次无效。
 * SPO2_MIN_DC      红外直流低于 5 万视为手指不在位（贴合时十万量级，悬空千级）。
 * AC 统计窗        不写死，按半个搏动周期（峰间隔的一半）自适应，钳位在 10~60 点，
 *                  保证 60~180bpm 都能覆盖一个完整搏动，又不会跨到相邻搏动。
 * ================================================================================== */
#define SPO2_FS				100
#define SPO2_BUF_LEN		500
#define SPO2_STEP			100

#define SPO2_PEAK_DIST		8
#define SPO2_MAX_PEAKS		15
#define SPO2_MIN_PEAKS		3

#define SPO2_MIN_DC			50000

/* 结果保持：成功算出一组后，此后 300 个样本（3 秒）内的偶发失败沿用它，
 * 避免弱信号下 valid 在 1/0 之间跳、界面每秒闪一次 "--" */
#define SPO2_HOLD_SAMPLES	300

/* R 值滑动平均次数：单次计算的 R 在弱信号下会抖 ±50%（实测 0.99~1.68），
 * 平均后 SpO2 才不会在几个值之间来回跳。3 次约等于 3 秒响应。 */
#define SPO2_R_AVG_NUM		3

/* R 值增益校准
 * 本项目没有商用血氧仪做对照标定。实测校准前 R 停在 1.7~2.1，
 * 比生理预期（0.5~0.7）高出 3 倍左右，直接代入公式只有 70~80%。
 * 这里乘 0.40 把 R 拉回生理区间：
 *   R 实测 1.45 -> 0.58 -> SpO2 约 97%
 *   R 实测 1.70 -> 0.68 -> SpO2 约 94%
 * 注意 0.40 是在"两路 LED 电流相同、DC 基本相等"的前提下定的；
 * 如果改了 LED 档位策略或光学结构，必须重新确定这个系数。
 * 这是按"正常人血氧 95~99%"反推的经验补偿，不是标定，见 README。 */
#define SPO2_R_GAIN_NUM		2
#define SPO2_R_GAIN_DEN		5

/* 血氧有效范围（器件与算法典型精度 ±5%，超范围一律判无效而不是钳位） */
#define SPO2_MIN_VALID		70
#define SPO2_MAX_VALID		100
#define SPO2_R_MIN			300		/* R x 1000，即 R >= 0.30 */
#define SPO2_R_MAX			1800	/* R x 1000，即 R <= 1.80 */

void SPO2_Init(void);
void SPO2_Reset(void);

/* 每来一组 FIFO 样本调一次（100Hz），内部满 500 点后自动计算并滑窗 */
void SPO2_Feed(u32 red, u32 ir);

u8   SPO2_GetValue(void);			/* 血氧百分比，0=无效/未就绪 */
u8   SPO2_GetHR(void);				/* 本算法独立算出的心率，用于交叉校验 */
u8   SPO2_IsValid(void);			/* 1=本次结果有效 */
u8   SPO2_IsReady(void);			/* 1=已经至少算过一次（界面显示"测量中"用） */
u16  SPO2_GetR1000(void);			/* R 值 x1000，调试用 */
u8   SPO2_GetPeaks(void);			/* 最近一次找出的峰数，调试用 */
u32  SPO2_GetDCRed(void);			/* 红光直流分量，调试用 */
u32  SPO2_GetDCIr(void);			/* 红外直流分量，调试用 */

#endif
