#include "step.h"
#include "mpu6050.h"

/* ============================ 时间基准：TIM2 自由计数器 ============================
 * 计步必须用真实时间（300ms 死区不能靠"循环次数 × 10ms"估算，主循环里
 * OLED 刷屏、DHT11 位操作都会让循环周期抖动）。
 *
 * TIM2 挂在 APB1，APB1 二分频 36MHz，定时器时钟倍频回 72MHz：
 *   预分频 7200  -> 计数频率 10kHz（1 个计数 = 0.1ms）
 *   周期 0xFFFF  -> 6.55s 回绕一次，靠增量累加自动处理回绕
 * 这里不开中断，只用"两次读取的计数差"累加，避免 1ms 中断打断
 * DHT11 / DS18B20 的微秒级位操作。
 * ============================================================================ */
#define STEP_TIM			TIM2
#define STEP_TIM_RCC		RCC_APB1Periph_TIM2
#define STEP_TIM_PSC		(7200 - 1)		/* 72MHz / 7200 = 10kHz */
#define STEP_TIM_ARR		0xFFFF

static u16 s_tim_last;					/* 上一次读到的计数器值 */
static u32 s_tick01;					/* 0.1ms 累计，32 位约 4.9 天，做差运算不受回绕影响 */

/* 读取并累加时间：调用间隔必须小于 6.5s（STEP_Poll 每 20ms 调一次） */
static u32 STEP_Ticks(void)
{
	u16 cnt = TIM_GetCounter(STEP_TIM);

	s_tick01 += (u16)(cnt - s_tim_last);	/* 16 位自然回绕，差值即真实增量 */
	s_tim_last = cnt;

	return s_tick01;
}

static u32 STEP_NowMS(void)
{
	return STEP_Ticks() / 10;
}


/* ============================ 算法状态 ============================ */
typedef struct
{
	u32 steps;							/* 累计步数 */

	u16 win[STEP_FILTER_WIN];			/* 滑动平均窗口（原始 mg） */
	u32 win_sum;						/* 窗口内求和，避免每次重新累加 */
	u8  win_idx;
	u8  win_cnt;

	s16 mag_raw;						/* 本次原始合加速度 */
	s16 mag_flt;						/* 滤波后合加速度 */

	u8  armed;							/* 1=信号已回落到阈值以下，可识别下一次上升沿 */
	u8  has_last;						/* 是否已有上一步（用于算步间隔） */
	u32 last_step_ms;					/* 上一步时刻 */

	u32 last_sample_ms;					/* 上次采样时刻（控制 20ms 节拍） */

	u8  err_cnt;						/* 连续异常样本计数 */
	u8  fault;							/* 1=传感器数据异常 */

	u16 ibi[STEP_CADENCE_NUM];			/* 最近几次步间隔（ms） */
	u8  ibi_idx;
	u8  ibi_cnt;
} STEP_State_t;

static STEP_State_t s_step;


/* 检测器复位：清滤波窗口和峰值状态，步数保持不动 */
static void STEP_DetectorReset(u32 now)
{
	u8 i;

	for(i = 0; i < STEP_FILTER_WIN; i++)
		s_step.win[i] = 0;

	s_step.win_sum   = 0;
	s_step.win_idx   = 0;
	s_step.win_cnt   = 0;
	s_step.armed     = 1;				/* 允许识别上升沿 */
	s_step.has_last  = 0;
	s_step.last_step_ms = now;			/* 复位后死区从当前时刻算起，避免立刻误计一步 */
	s_step.ibi_idx   = 0;
	s_step.ibi_cnt   = 0;
}


/* 第一步：合加速度由调用方给出（MPU6050_ReadMagnitudeMG 已算好 sqrt(ax^2+ay^2+az^2)）
 * 第三步 / 第四步 / 第五步：阈值上升沿 + 300ms 死区，两个条件同时满足才计一步 */
static u8 STEP_Detect(s32 flt, u32 now)
{
	u32 dt;

	if(flt >= STEP_TH_MG)
	{
		if(!s_step.armed)				/* 还没回落到阈值以下，同一次波峰不重复计数 */
			return 0;

		s_step.armed = 0;

		dt = (u32)(now - s_step.last_step_ms);		/* 与上一次有效峰值的间隔 */

		if(s_step.has_last && dt < STEP_DEAD_TIME_MS)
			return 0;					/* 死区内，判为同一步的抖动，丢弃 */

		/* 记录步间隔用于步频；间隔过长说明中间停顿过，不参与平均 */
		if(s_step.has_last && dt <= STEP_MAX_INTERVAL_MS)
		{
			s_step.ibi[s_step.ibi_idx] = (u16)dt;
			if(++s_step.ibi_idx >= STEP_CADENCE_NUM)
				s_step.ibi_idx = 0;
			if(s_step.ibi_cnt < STEP_CADENCE_NUM)
				s_step.ibi_cnt++;
		}
		else
		{
			s_step.ibi_cnt = 0;			/* 停顿后重新开始统计步频 */
			s_step.ibi_idx = 0;
		}

		s_step.last_step_ms = now;
		s_step.has_last     = 1;
		s_step.steps++;

		return 1;
	}

	if(flt <= STEP_REARM_MG)			/* 回落到回落阈值以下，重新武装 */
		s_step.armed = 1;

	return 0;
}


/* ============================ 采样处理（核心） ============================ */
static u8 STEP_Process(s16 mag, u32 now)
{
	s32 flt;

	s_step.mag_raw = mag;

	/* ---------- 边界处理 1：异常样本 ---------- */
	if(mag < STEP_MAG_MIN || mag > STEP_MAG_MAX)
	{
		if(s_step.err_cnt < 0xFF)
			s_step.err_cnt++;

		if(s_step.err_cnt >= STEP_ERR_LIMIT)	/* 连续 0.5s 异常 -> 判传感器故障 */
		{
			s_step.fault = 1;
			STEP_DetectorReset(now);
		}

		return 0;						/* 异常样本不参与滤波，避免污染窗口 */
	}

	s_step.err_cnt = 0;
	s_step.fault   = 0;

	/* ---------- 第二步：滑动平均滤波，滤掉手抖/颠簸引起的高频毛刺 ---------- */
	s_step.win_sum -= s_step.win[s_step.win_idx];
	s_step.win[s_step.win_idx] = (u16)mag;
	s_step.win_sum += (u16)mag;

	if(++s_step.win_idx >= STEP_FILTER_WIN)
		s_step.win_idx = 0;

	if(s_step.win_cnt < STEP_FILTER_WIN)
		s_step.win_cnt++;

	flt = (s32)(s_step.win_sum / s_step.win_cnt);
	s_step.mag_flt = (s16)flt;

	/* ---------- 第三~五步：阈值上升沿 + 死区时间 ---------- */
	return STEP_Detect(flt, now);
}


/* ============================ 对外接口 ============================ */
void STEP_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(STEP_TIM_RCC, ENABLE);

	TIM_TimeBaseStructure.TIM_Period        = STEP_TIM_ARR;
	TIM_TimeBaseStructure.TIM_Prescaler     = STEP_TIM_PSC;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(STEP_TIM, &TIM_TimeBaseStructure);

	TIM_SetCounter(STEP_TIM, 0);
	TIM_Cmd(STEP_TIM, ENABLE);

	s_tim_last = TIM_GetCounter(STEP_TIM);
	s_tick01   = 0;

	STEP_Reset();
}

void STEP_Reset(void)
{
	u32 now = STEP_NowMS();

	s_step.steps    = 0;
	s_step.err_cnt  = 0;
	s_step.fault    = 0;
	s_step.mag_raw  = 0;
	s_step.mag_flt  = 0;

	STEP_DetectorReset(now);
}

/* 主循环调用：内部按 50Hz 节拍采样，返回 1 表示这一步被计入 */
u8 STEP_Poll(void)
{
	s16 mag;
	u32 now = STEP_NowMS();

	if((u32)(now - s_step.last_sample_ms) < STEP_SAMPLE_MS)	/* 未到下一个采样点 */
		return 0;

	s_step.last_sample_ms = now;

	/* 数据来源：MPU6050 三轴加速度 -> 合加速度（mg） */
	if(MPU6050_ReadMagnitudeMG(&mag) != 0)
	{
		/* I2C 读失败：当作异常样本，绝不沿用上一次的旧值 */
		mag = 0;
	}

	return STEP_Process(mag, now);
}

/* 外部数据源（例如已经把 MPU6050 数据预处理好的模块）可直接喂入合加速度 */
u8 STEP_FeedMG(s16 mag)
{
	return STEP_Process(mag, STEP_NowMS());
}

u32 STEP_GetCount(void)
{
	return s_step.steps;
}

u16 STEP_GetCadence(void)
{
	u32 sum = 0;
	u8  i;

	if(s_step.ibi_cnt == 0)
		return 0;

	if((u32)(STEP_NowMS() - s_step.last_step_ms) > STEP_MOVING_MS)
		return 0;						/* 3 秒没动，步频清零 */

	for(i = 0; i < s_step.ibi_cnt; i++)
		sum += s_step.ibi[i];

	if(sum == 0)
		return 0;

	return (u16)(60000UL * s_step.ibi_cnt / sum);	/* 平均步间隔 -> 步/分钟 */
}

s16 STEP_GetFilteredMG(void)
{
	return s_step.mag_flt;
}

s16 STEP_GetRawMG(void)
{
	return s_step.mag_raw;
}

u8 STEP_IsMoving(void)
{
	if(!s_step.has_last)
		return 0;

	return ((u32)(STEP_NowMS() - s_step.last_step_ms) <= STEP_MOVING_MS) ? 1 : 0;
}

u8 STEP_GetStatus(void)
{
	return s_step.fault ? STEP_STATUS_ERROR : STEP_STATUS_OK;
}
