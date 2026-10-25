#include "spo2.h"

/* ============================ 算法说明 ============================
 * 完全按实验指导书第九节实现，全部整数运算：
 *   心率：去直流 -> 4 点滑动平均 -> 差分 -> 5 点 Hamming 卷积
 *         -> 自适应阈值峰值检测（最小峰距 8 点）-> 峰间隔换算 bpm
 *   血氧：R = (ACred/DCred) / (ACir/DCir)
 *         SpO2 = -45.060*R^2 + 30.354*R + 94.845（定点实现，等价于 184 项查表）
 *
 * 内存：500 点 x 4 字节 x 4 个数组 = 8KB（RAM 20KB，当前工程约 3.4KB，余量足够）
 * 耗时：一次计算约 0.5~1ms，每秒触发一次，不影响主循环实时性
 * ================================================================= */

static u32 s_red[SPO2_BUF_LEN];			/* 红光原始值（18 位有效） */
static u32 s_ir[SPO2_BUF_LEN];			/* 红外原始值（18 位有效） */
static s32 s_d[SPO2_BUF_LEN];			/* 差分信号 */
static s32 s_c[SPO2_BUF_LEN];			/* Hamming 卷积结果 */

static u16 s_pos;						/* 下一个写入位置 */
static u8  s_ready;						/* 是否已经算过至少一次 */
static u8  s_valid;						/* 当前结果是否有效（含保持窗口） */
static u8  s_spo2;						/* 血氧百分比 */
static u8  s_hr;						/* 本算法算出的心率 */
static u16 s_r1000;						/* R 值 x1000 */
static u32 s_dcRed;
static u32 s_dcIr;

static u32 s_samples;					/* 累计送入的样本数 */
static u32 s_ok_samples;				/* 最近一次成功计算时的样本数 */
static u8  s_peaks;						/* 最近一次找出的峰数（诊断用） */
static u16 s_pi10000;					/* 灌注指数 AC/DC x10000，信号质量依据 */

static u16 s_r_hist[SPO2_R_AVG_NUM];	/* R 值滑动平均 */
static u8  s_r_pos;
static u8  s_r_cnt;


/* 整数开方（用于 RMS），避免引入 math.h */
static u32 SPO2_ISqrt(u32 n)
{
	u32 r = 0, bit = 1UL << 30;

	while(bit > n)
		bit >>= 2;

	while(bit)
	{
		if(n >= r + bit)
		{
			n -= r + bit;
			r = (r >> 1) + bit;
		}
		else
		{
			r >>= 1;
		}
		bit >>= 2;
	}

	return r;
}


/* ============================ 核心计算 ============================ */
static void SPO2_Calc(void)
{
	u32 i, sum_r = 0, sum_i = 0, dc_r, dc_i, sum_abs, th, interval, hr;
	u32 ac_r, ac_i, rr, ri, rq;
	s32 sq, v;
	u16 peaks[SPO2_MAX_PEAKS];
	u8  n = 0;
	s32 last = -1000;

	/* 先置无效；失败时是否沿用上一次结果，由 SPO2_Feed 的保持逻辑决定，
	 * 所以这里不能把 s_spo2 / s_hr 清掉 */
	s_valid = 0;

	/* ---------- 1. 两路直流分量：整窗平均 ---------- */
	for(i = 0; i < SPO2_BUF_LEN; i++)
	{
		sum_r += s_red[i];
		sum_i += s_ir[i];
	}
	dc_r = sum_r / SPO2_BUF_LEN;
	dc_i = sum_i / SPO2_BUF_LEN;

	s_dcRed = dc_r;
	s_dcIr  = dc_i;

	/* 手指不在位：贴合时直流十万量级，悬空只有千级 */
	if(dc_i < SPO2_MIN_DC || dc_r < SPO2_MIN_DC)
		return;

	/* ---------- 2. 4 点滑动平均 + 差分（合并成一步） ----------
	 * 平滑值 sm[i] = (ir[i]+ir[i+1]+ir[i+2]+ir[i+3])/4
	 * 差分   d[i]  = sm[i+1] - sm[i] = (ir[i+3] - ir[i-1]) / 4 */
	s_d[0] = 0;
	for(i = 1; i + 3 < SPO2_BUF_LEN; i++)
		s_d[i] = ((s32)s_ir[i + 3] - (s32)s_ir[i - 1]) / 4;

	for(i = SPO2_BUF_LEN - 3; i < SPO2_BUF_LEN; i++)
		s_d[i] = 0;

	/* ---------- 3. 5 点 Hamming 卷积 [41,276,512,276,41]，归一化系数 1146 ---------- */
	for(i = 2; i + 2 < SPO2_BUF_LEN; i++)
		s_c[i] = (s32)(41L  * s_d[i - 2] + 276L * s_d[i - 1] + 512L * s_d[i]
		             + 276L * s_d[i + 1] + 41L  * s_d[i + 2]) / 1146;

	s_c[0] = s_c[1] = 0;
	s_c[SPO2_BUF_LEN - 2] = s_c[SPO2_BUF_LEN - 1] = 0;

	/* ---------- 4. 自适应阈值：卷积结果绝对值的平均 ---------- */
	sum_abs = 0;
	for(i = 2; i + 2 < SPO2_BUF_LEN; i++)
		sum_abs += (u32)((s_c[i] < 0) ? -s_c[i] : s_c[i]);

	th = sum_abs / (SPO2_BUF_LEN - 4);
	th = th + th * 3 / 10;				/* 1.3 倍均值：峰检测现在只用于输出 HR2，取折中值 */
	if(th < 1)
		th = 1;

	/* ---------- 5. 峰值检测：越过阈值 + 局部极大 + 最小峰距 8 点 ---------- */
	for(i = 3; i + 3 < SPO2_BUF_LEN; i++)
	{
		if(s_c[i] <= (s32)th)					continue;
		if(s_c[i] <= s_c[i - 1])				continue;
		if(s_c[i] <  s_c[i + 1])				continue;

		if(n < SPO2_MAX_PEAKS)
		{
			if(n == 0 || ((s32)i - last) >= SPO2_PEAK_DIST)
			{
				peaks[n++] = (u16)i;
				last = (s32)i;
			}
		}
	}

	s_peaks = n;							/* 记录峰数，用于串口诊断 */

	/* ---------- 6. 峰间隔 -> 心率（仅作交叉校验，失败不影响血氧） ---------- */
	hr = 0;
	if(n >= SPO2_MIN_PEAKS)
	{
		interval = (u32)(peaks[n - 1] - peaks[0]);
		if(interval != 0)
		{
			hr = 6000UL * (u32)(n - 1) / interval;
			if(hr < 30 || hr > 200)			/* 峰里混进假峰时 hr 会异常，丢弃即可 */
				hr = 0;
		}
	}

	/* ---------- 7. 血氧：整窗 AC 峰峰值，不依赖峰值检测 ----------
	 * 原来按"每个峰附近取一段窗口"统计，但峰检测在弱信号下时有时无
	 * （实测 5 秒内峰数在 0~12 之间跳），一旦不足 3 个峰整次计算就作废。
	 * 血氧本质上只需要 DC 与 AC 的比值，5 秒窗口里本来就有 5~8 个完整搏动，
	 * 直接统计整窗的峰峰值更稳，也把血氧和心率解耦了。
	 *
	 * 统计前做 4 点滑动平均（40ms 窗）：抑制 50/100Hz 工频和 LED 驱动纹波，
	 * 心率支路一直有这一步，血氧原来漏了。
	 * 再剔除偏离直流 5% 以上的点：那是环境光/串扰造成的尖峰，
	 * 会让 max-min 被单个点放大好几倍（实测红光出现过 12% 的跳变）。 */
	{
		s32 lim_r = (s32)(dc_r / 50);		/* 剔除偏离直流 2% 以上的点 */
		s32 lim_i = (s32)(dc_i / 50);
		u32 sq_r = 0, sq_i = 0, cnt = 0;

		for(i = 0; i + 3 < SPO2_BUF_LEN; i++)
		{
			u32 rv = (s_red[i] + s_red[i + 1] + s_red[i + 2] + s_red[i + 3]) / 4;
			u32 iv = (s_ir[i]  + s_ir[i + 1]  + s_ir[i + 2]  + s_ir[i + 3])  / 4;
			s32 dr = (s32)rv - (s32)dc_r;
			s32 di = (s32)iv - (s32)dc_i;

			if(dr > lim_r || dr < -lim_r)	continue;	/* 环境光/串扰尖峰 */
			if(di > lim_i || di < -lim_i)	continue;

			/* 先缩小 4 倍再平方：偏差最大 2% x 11 万 = 2200，/4 后 550，
			 * 平方 30 万，500 点累加约 1.5e8，不会溢出 u32 */
			dr /= 4;
			di /= 4;

			sq_r += (u32)(dr * dr);
			sq_i += (u32)(di * di);
			cnt++;
		}

		if(cnt < 50)						/* 有效点不足 1/10，说明信号被噪声占满 */
			return;

		/* RMS：比峰峰值对单个尖峰鲁棒得多。R 是两路之比，
		 * RMS 与峰峰值的换算系数会被约掉，期望值不变，只减小方差 */
		ac_r = SPO2_ISqrt(sq_r / cnt) * 4;
		ac_i = SPO2_ISqrt(sq_i / cnt) * 4;
	}

	if(ac_r == 0 || ac_i == 0)
		return;

	/* 限幅，保证下面 ac * 100000 不溢出 u32 */
	if(ac_r > 20000) ac_r = 20000;
	if(ac_i > 20000) ac_i = 20000;

	/* 灌注指数 PI = AC/DC，放大 1e4：
	 * 它正比于搏动血量占组织总量的比例，不受 LED 电流影响，
	 * 是判断"这次信号值不值得信"最直接的指标（AGC 调电流时尤其有用） */
	s_pi10000 = (u16)(ac_i * 10000UL / dc_i);

	/* ---------- 8. R = (ACred/DCred) / (ACir/DCir)，结果定点为 R x1000 ---------- */
	rr = ac_r * 100000UL / dc_r;			/* AC/DC 放大 1e5，量级千 */
	ri = ac_i * 100000UL / dc_i;
	if(ri == 0)
		return;

	rq = rr * 1000UL / ri;

	/* R 值滑动平均：单次计算的 R 在弱信号下会抖 ±50%，平均后再送进公式 */
	s_r_hist[s_r_pos] = (u16)rq;
	if(++s_r_pos >= SPO2_R_AVG_NUM)
		s_r_pos = 0;
	if(s_r_cnt < SPO2_R_AVG_NUM)
		s_r_cnt++;

	{
		u32 sum = 0;
		u8  m;

		for(m = 0; m < s_r_cnt; m++)
			sum += s_r_hist[m];

		rq = sum / s_r_cnt;
	}

	/* 经验增益校准：把系统性偏高的 R 拉回生理区间（系数依据见 spo2.h） */
	rq = rq * SPO2_R_GAIN_NUM / SPO2_R_GAIN_DEN;

	if(rq < SPO2_R_MIN || rq > SPO2_R_MAX)
		return;

	/* ---------- 9. SpO2 = -45.060*R^2 + 30.354*R + 94.845（结果 x1000 再取整） ---------- */
	sq = (s32)(rq * rq / 1000);				/* R^2 x1000 */
	v  = 94845L + (s32)(30354L * rq) / 1000 - (s32)(45060L * sq) / 1000;
	v /= 1000;

	/* 超出 70~100 说明信号或 R 值不可信，判无效而不是钳位到边界 */
	if(v < SPO2_MIN_VALID || v > SPO2_MAX_VALID)
		return;

	/* R 与 SpO2 必须同一次计算一起更新，否则串口里会出现
	 * "SPO2: 92% 配 R=0.958"（R 是本次、SpO2 是上次的残留值）这种对不上的情况 */
	s_spo2  = (u8)v;
	s_hr    = (u8)hr;
	s_r1000 = (u16)rq;
	s_valid = 1;
}


/* ============================ 对外接口 ============================ */
void SPO2_Reset(void)
{
	u16 i;

	for(i = 0; i < SPO2_BUF_LEN; i++)
	{
		s_red[i] = 0;
		s_ir[i]  = 0;
		s_d[i]   = 0;
		s_c[i]   = 0;
	}

	s_pos        = 0;
	s_ready      = 0;
	s_valid      = 0;
	s_spo2       = 0;
	s_hr         = 0;
	s_r1000      = 0;
	s_dcRed      = 0;
	s_dcIr       = 0;
	s_samples    = 0;
	s_ok_samples = 0;
	s_peaks      = 0;
	s_pi10000    = 0;
	s_r_pos      = 0;
	s_r_cnt      = 0;

	for(i = 0; i < SPO2_R_AVG_NUM; i++)
		s_r_hist[i] = 0;
}

void SPO2_Init(void)
{
	SPO2_Reset();
}

/* 100Hz 每来一组样本调一次；攒满 500 点计算一次，然后滑窗腾出 100 点 */
void SPO2_Feed(u32 red, u32 ir)
{
	u16 i;

	if(s_pos >= SPO2_BUF_LEN)
		s_pos = 0;

	s_red[s_pos] = red;
	s_ir[s_pos]  = ir;
	s_pos++;
	s_samples++;

	if(s_pos < SPO2_BUF_LEN)
		return;

	SPO2_Calc();

	/* 弱信号下峰数会时多时少，计算偶发失败很常见。
	 * 成功一次后 3 秒内失败都沿用它，超过 3 秒才真正失效并清空数值 */
	if(s_valid)
	{
		s_ok_samples = s_samples;
	}
	else if(s_spo2 != 0 && (u32)(s_samples - s_ok_samples) <= SPO2_HOLD_SAMPLES)
	{
		/* 必须真的有过有效值才保持，否则会出现 "0% 但 valid=1" 的怪状态 */
		s_valid = 1;
	}
	else
	{
		s_spo2 = 0;
		s_hr   = 0;
	}

	s_ready = 1;

	/* 滑窗：丢掉最旧 100 点，保留最近 400 点，后面再补 100 点凑满 500 */
	for(i = SPO2_STEP; i < SPO2_BUF_LEN; i++)
	{
		s_red[i - SPO2_STEP] = s_red[i];
		s_ir[i - SPO2_STEP]  = s_ir[i];
	}

	s_pos = SPO2_BUF_LEN - SPO2_STEP;
}

u8 SPO2_GetValue(void)
{
	return s_valid ? s_spo2 : 0;
}

u8 SPO2_GetHR(void)
{
	return s_valid ? s_hr : 0;
}

u8 SPO2_IsValid(void)
{
	return s_valid;
}

u8 SPO2_IsReady(void)
{
	return s_ready;
}

u16 SPO2_GetR1000(void)
{
	return s_r1000;
}

u8 SPO2_GetPeaks(void)
{
	return s_peaks;
}

u16 SPO2_GetPI10000(void)
{
	return s_pi10000;
}

/* 信号质量 0~100：把 PI 线性映射到 SPO2_PI_MIN~SPO2_PI_MAX 区间。
 * 无效时直接给 0，避免"数值有效但信号很差"时显示满格信号条 */
u8 SPO2_GetQuality(void)
{
	if(!s_valid)
		return 0;

	if(s_pi10000 <= SPO2_PI_MIN)
		return 5;

	if(s_pi10000 >= SPO2_PI_MAX)
		return 100;

	return (u8)((u32)(s_pi10000 - SPO2_PI_MIN) * 100 / (SPO2_PI_MAX - SPO2_PI_MIN));
}

u32 SPO2_GetDCRed(void)
{
	return s_dcRed;
}

u32 SPO2_GetDCIr(void)
{
	return s_dcIr;
}
