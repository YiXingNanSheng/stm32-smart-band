#include "hr.h"

/* ============================ 内部状态 ============================ */
static u32 s_dc;				/* 直流分量估计 */
static s32 s_ac_prev;			/* 上一个交流分量（带符号） */
static s32 s_ac_prev2;			/* 上上个交流分量（带符号） */
static u32 s_peak_max;			/* 近期峰值幅度，用于自适应阈值 */
static u16 s_idx;				/* 样本计数 */
static u16 s_last_peak;			/* 上一个峰值的位置 */
static u8  s_peak_valid;		/* 是否已经有过一个峰值 */
static u16 s_ibi;				/* 最近一次 IBI */
static u16 s_hr;				/* 心率 bpm */
static u8  s_finger;			/* 手指在位标志 */
static u8  s_stable;			/* DC 收敛计数，够数才开始检测 */

static u16 s_ibi_buf[HR_IBI_AVG_NUM];
static u8  s_ibi_cnt;
static u8  s_ibi_pos;

/* FIFO 关闭硬件平均后是 100Hz，这里再做 4 点平均并抽取到 25Hz，
   与改动前的等效（原来是芯片内部做 4 次平均） */
static u32 s_dec_sum;
static u8  s_dec_cnt;


void HR_Init(void)
{
	s_dc         = 0;
	s_ac_prev    = 0;
	s_ac_prev2   = 0;
	s_peak_max   = 0;
	s_idx        = 0;
	s_last_peak  = 0;
	s_peak_valid = 0;
	s_ibi        = 0;
	s_hr         = 0;
	s_finger     = 0;
	s_stable     = 0;
	s_ibi_cnt    = 0;
	s_ibi_pos    = 0;
	s_dec_sum    = 0;
	s_dec_cnt    = 0;
}


/*******************************************************************************
* 函 数 名 : HR_Feed
* 函数功能 : 送入一个红外通道原始值，内部完成去直流、峰值检测、心率计算
* 说    明 : 25Hz 采样下每 40ms 调一次；只用 IR 通道，红光通道留给血氧
*******************************************************************************/
void HR_Feed(u32 ir)
{
	s32 ac;
	s32 d;
	u8  was_finger;

	s_idx++;

	/* 首个样本直接当作直流初值，避免从 0 慢慢爬 */
	if(s_dc == 0)
	{
		s_dc = ir;
		return;
	}

	/* 一阶低通跟踪直流分量，时间常数约 32 个样本（1.3s）
	   取 1.3s 是为了让手指贴上后 4~6 秒内 DC 收敛，
	   同时截止频率 0.12Hz 远低于心跳 1~2Hz，不会把心跳当直流滤掉 */
	d = (s32)ir - (s32)s_dc;
	s_dc = (u32)((s32)s_dc + d / 32);

	/* 手指在位判断：贴住时直流在十万量级，悬空只有千级 */
	was_finger = s_finger;
	s_finger = (s_dc > HR_FINGER_DC) ? 1 : 0;

	if(!s_finger)
	{
		/* 手指离开：清掉峰值相关状态，心率归零 */
		s_peak_max   = 0;
		s_ac_prev    = 0;
		s_ac_prev2   = 0;
		s_peak_valid = 0;
		s_hr         = 0;
		s_ibi        = 0;
		s_ibi_cnt    = 0;
		s_ibi_pos    = 0;
		s_stable     = 0;
		return;
	}

	/* 手指刚贴上的那几秒，DC 正从千级爬到十万级，
	   这期间 ac = |ir - dc| 会高达十几万，如果拿去更新 peak_max，
	   阈值会被顶到五万，而真实心跳的 ac 只有几百 —— 之后几十秒都检测不到。
	   所以：贴上瞬间先清空状态，等 DC 收敛了再开始检测。 */
	if(!was_finger)
	{
		s_peak_max   = 0;
		s_ac_prev    = 0;
		s_ac_prev2   = 0;
		s_peak_valid = 0;
		s_stable     = 0;
		return;
	}

	/* DC 收敛判断：单次变化小于当前直流的 2% 视为稳定。
	   注意不能用 0.5%（dc/200）——IR 的正常搏动幅度就有 700 左右，
	   门槛卡在噪声线上会导致 s_stable 被反复清零，永远进不了检测。 */
	if(d < 0) d = -d;
	if(d < (s32)(s_dc / 50))
	{
		if(s_stable < 60) s_stable++;
	}
	else
	{
		s_stable = 0;
	}

	if(s_stable < 60)
		return;

	/* 交流分量：必须保留符号，只检测正峰。
	   如果取绝对值，每个心跳周期会出现正负两个峰，算出的 IBI 只有真值一半，
	   HR 直接翻倍，而且 IBI 会掉到下限以下被过滤掉 —— 表现为永远测不出。 */
	ac = (s32)ir - (s32)s_dc;

	/* 兜底：交流超过直流 10% 说明还在过渡（或手指在动），不当成心跳 */
	if(ac > (s32)(s_dc / 10) || ac < -(s32)(s_dc / 10))
		return;

	if(ac > 0 && (u32)ac > s_peak_max)
		s_peak_max = (u32)ac;

	/* 局部极大：前一个点高于它前一个点、也高于当前点，且幅度超过峰值的 1/3
	 * 阈值从 1/2 放松到 1/3：AC/DC 只有 0.3% 时，噪声尖峰会临时抬高 s_peak_max，
	 * 卡在 1/2 会让真实心跳永远够不到阈值，表现为 IBI 恒为 0 */
	if(s_ac_prev > s_ac_prev2 && s_ac_prev > ac && s_ac_prev > (s32)(s_peak_max / 3))
	{
		if(s_peak_valid)
		{
			u16 ibi = (u16)(s_idx - s_last_peak);	/* 无符号减法，计数溢出也正确 */

			if(ibi >= HR_MIN_IBI && ibi <= HR_MAX_IBI)
			{
				s_ibi = ibi;

				/* IBI 滑动平均，抑制单次误检带来的跳变 */
				s_ibi_buf[s_ibi_pos] = ibi;
				s_ibi_pos = (u8)((s_ibi_pos + 1) % HR_IBI_AVG_NUM);
				if(s_ibi_cnt < HR_IBI_AVG_NUM)
					s_ibi_cnt++;

				{
					u32 sum = 0;
					u8  i;

					for(i = 0; i < s_ibi_cnt; i++)
						sum += s_ibi_buf[i];

					if(sum > 0)
						s_hr = (u16)(60UL * HR_FS * s_ibi_cnt / sum);
				}
			}
		}

		s_last_peak  = s_idx;
		s_peak_valid = 1;
	}

	s_ac_prev2 = s_ac_prev;
	s_ac_prev  = ac;

	/* 峰值幅度每 32 个样本（1.28 秒）衰减 1/8，避免阈值被一次噪声尖峰长期卡住 */
	if((s_idx & 0x1F) == 0 && s_peak_max > 200)
		s_peak_max -= (s_peak_max >> 3);
}


/*******************************************************************************
* 函 数 名 : HR_Feed100Hz
* 函数功能 : 送入 100Hz 原始红外样本，内部 4 点平均后按 25Hz 送给 HR_Feed
* 说    明 : MAX30102 的 FIFO 硬件平均已关闭（血氧算法需要 100Hz），
*            这里用软件补齐原来的 4 次平均，HR_FS 仍为 25Hz，算法参数无需改动
*******************************************************************************/
void HR_Feed100Hz(u32 ir)
{
	s_dec_sum += ir;

	if(++s_dec_cnt < 4)
		return;

	ir        = s_dec_sum / 4;
	s_dec_sum = 0;
	s_dec_cnt = 0;

	HR_Feed(ir);
}


u16 HR_GetRate(void)
{
	return s_hr;
}

u16 HR_GetIBI(void)
{
	return s_ibi;
}

u8 HR_IsFingerOn(void)
{
	return s_finger;
}

u32 HR_GetDC(void)
{
	return s_dc;
}


u32 HR_GetPeakMax(void)
{
	return s_peak_max;
}

u8 HR_IsStable(void)
{
	return (s_stable >= 60) ? 1 : 0;
}
