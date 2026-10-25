#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "buzzer.h"
#include "relay.h"
#include "key.h"
#include "dht11.h"
#include "ds18b20.h"
#include "mpu6050.h"
#include "step.h"
#include "max30102.h"
#include "hr.h"
#include "spo2.h"
#include "usart.h"
#include "usart2.h"
#include "oled.h"

#define UI_PAGE_NUM		5		/* 页码指示器按这个数量画方块，加页只改这里 */

/* ============================ 心率异常报警 ============================
 * 判定条件：手指在位、心率有效，且超出 [LOW, HIGH] 区间
 * 确认时间：连续 CONFIRM 次 2 秒任务（约 10 秒），避免单次误检就报警
 * 报警动作：蜂鸣器 + LED3/LED4 间歇闪烁，覆盖手动开关状态
 * ==================================================================== */
#define HR_ALARM_LOW		50			/* 低于此值：心动过缓 */
#define HR_ALARM_HIGH		110		/* 高于此值：心动过速 */
#define HR_ALARM_CONFIRM	5			/* 连续 5 次（约 10 秒）才确认 */

/* ============================ 低血氧报警 ============================
 * 判定条件：血氧结果有效（算法判有效 + 手指在位）、未处于运动状态，
 *           且 SpO2 低于阈值；连续 CONFIRM 次 2 秒任务（约 10 秒）确认
 * 阈值取 90%：消费级器件精度 ±5%，再低才具备提示意义
 * ==================================================================== */
#define SPO2_ALARM_LOW		88			/* 校准后正常值在 93~98，低于 88 才有提示意义 */
#define SPO2_ALARM_CONFIRM	15			/* 连续 15 次（约 30 秒）：未标定的数值抖动大，确认要更久 */

/* 界面数据集中管理，避免 UI_Refresh 参数过多 */
typedef struct
{
    u8  page;                           /* 当前页 0/1 */
    s16 body;                           /* 体温 0.1℃ */
    u8  body_ok;
    s8  temp;                           /* 环境温度 ℃ */
    u8  humi;                           /* 环境湿度 %RH */
    u8  dht_ok;
    u8  dht_err;
    u8  key_last;
    u8  led3, led4, buz, rly;
    s16 ax, ay, az;                     /* 加速度原始值 */
    s16 gx, gy, gz;                     /* 陀螺原始值 */
    s16 mag;                            /* 合加速度 mg */
    u16 hr;                             /* 心率 bpm */
    u16 ibi;                            /* 心跳间隔（样本数） */
    u16 peak;                           /* 交流峰值幅度 */
    u8  finger;                         /* 手指在位 */
    u8  alarm;                          /* 1=心率异常报警中 */
    u32 steps;                          /* 累计步数 */
    u16 cadence;                        /* 步频 步/分钟 */
    s16 step_mag;                       /* 合加速度原始值 mg */
    s16 step_flt;                       /* 滤波后合加速度 mg */
    u8  moving;                         /* 1=运动中 */
    u8  step_err;                       /* 1=计步传感器数据异常 */
    u8  spo2;                           /* 血氧百分比，0=无效 */
    u8  spo2_hr;                        /* 血氧算法独立算出的心率 */
    u8  spo2_valid;                     /* 1=本次血氧结果有效 */
    u8  spo2_ready;                     /* 1=已完成首次 5 秒积累 */
    u16 spo2_r;                         /* R 值 x1000 */
    u8  spo2_alarm;                     /* 1=低血氧报警中 */
    u32 dc_ir;                          /* 红外直流，用于判断贴合压力是否过大 */
    u8  heart_on;                       /* 1=显示心跳图标（随心跳闪现） */
    u8  sig;                            /* 血氧信号质量 0~100 */
    u8  imu_err;                        /* 1=MPU6050 掉线（供电跌落/总线异常） */
} UI_Data_t;

static UI_Data_t g_ui;

/* 最新一组 MAX30102 原始值，供界面和蓝牙上报使用 */
static u32 g_max_red = 0;
static u32 g_max_ir  = 0;


/* 加速度改用双向条形图、陀螺改用整数显示后，原来的两个格式化函数不再需要 */


/* ============================ 第 1 页：体温 + 环境 ============================
 * 布局：
 *   y=0..7   状态栏：页名 + 页码指示器
 *   y=9..40  体温大字（16x32）+ 单位
 *   y=42..49 体温条（36~40℃ 映射，长度即直观判断）
 *   y=52..59 环境温湿度（水滴 / 温度计图标 + 数值）
 * 执行器状态（LED/BUZ/RLY）已从前台移除：按键时实体设备本身有反馈，
 * 首页空间优先给体温这个核心指标。
 * ============================================================================ */

/* 8x8 图标 */
static const u8 ICON_DROP[8] = { 0x18, 0x3C, 0x3C, 0x7E, 0x7E, 0x7E, 0x3C, 0x18 };
static const u8 ICON_TEMP[8] = { 0x18, 0x18, 0x18, 0x1E, 0x1E, 0x3F, 0x3F, 0x1E };

static void UI_Page1(void)
{
    OLED_ShowString(2, 0, "BODY", OLED_FONT_8);
    OLED_DrawPageDots(94, 2, UI_PAGE_NUM, g_ui.page);

    /* ---------- 体温大字：整数 2 位 + 小数点 + 1 位小数 ---------- */
    if(g_ui.body_ok)
    {
        OLED_ShowNum2x(4, 9, g_ui.body / 10);
        OLED_ShowChar2x(40, 9, '.');
        OLED_ShowChar2x(56, 9, (char)('0' + g_ui.body % 10));
    }
    else
    {
        OLED_ShowChar2x(4, 9, '-');
        OLED_ShowChar2x(20, 9, '-');
        OLED_ShowChar2x(36, 9, '.');
        OLED_ShowChar2x(52, 9, '-');
    }
    OLED_ShowString(78, 22, "C", OLED_FONT_16);

    /* ---------- 体温条：36.0~40.0℃ ----------
     * 条形长度本身就是判断依据，不需要再读数字 */
    OLED_DrawRect(4, 42, 123, 49, OLED_WHITE);

    if(g_ui.body_ok)
    {
        s32 v = (s32)g_ui.body - 360;
        u8  fill;

        if(v < 0)  v = 0;
        if(v > 40) v = 40;

        fill = (u8)(v * 116 / 40);
        if(fill > 0)
            OLED_FillRect(5, 43, (u8)(4 + fill), 48, OLED_WHITE);
    }

    /* ---------- 环境温湿度 ---------- */
    OLED_DrawIcon8x8(2, 52, ICON_DROP);

    if(g_ui.dht_ok)
    {
        OLED_ShowNum(12, 52, g_ui.humi, OLED_FONT_8);
        OLED_ShowString(24, 52, "%", OLED_FONT_8);
    }
    else
    {
        OLED_ShowString(12, 52, "--", OLED_FONT_8);
    }

    OLED_DrawIcon8x8(44, 52, ICON_TEMP);

    if(g_ui.dht_ok)
    {
        OLED_ShowNum(54, 52, g_ui.temp, OLED_FONT_8);
        OLED_ShowString(66, 52, "C", OLED_FONT_8);
    }
    else
    {
        OLED_ShowString(54, 52, "--", OLED_FONT_8);
    }
}

/* ============================ 第 2 页：六轴姿态 ============================
 * 布局：
 *   y=0..7    状态栏
 *   y=9..34   三轴加速度双向条形图（中点为零位，左右延伸表示正负）
 *   y=37..52  合加速度大字（8x16）+ 单位
 *   y=55..63  三轴陀螺（整数近似，°/s）
 * 加速度用条而不是数字：判断"屏幕朝上还是侧倾"扫一眼就够，不需要读数。
 * ============================================================================ */

/* 双向条：中点为零位，正值向右、负值向左 */
static void UI_BiBar(u8 x, u8 y, u8 w, u8 h, s16 val, s16 range)
{
    s32 mid, len;

    OLED_DrawRect(x, y, (u8)(x + w - 1), (u8)(y + h - 1), OLED_WHITE);

    mid = (s32)x + w / 2;
    OLED_DrawLine((u8)mid, y, (u8)mid, (u8)(y + h - 1), OLED_WHITE);

    if(val > range)  val = range;
    if(val < -range) val = -range;

    /* 留 2 像素余量，避免填充盖住边框 */
    len = (s32)val * (w / 2 - 2) / range;

    if(len > 0)
        OLED_FillRect((u8)(mid + 1), (u8)(y + 1), (u8)(mid + len), (u8)(y + h - 2), OLED_WHITE);
    else if(len < 0)
        OLED_FillRect((u8)(mid + len), (u8)(y + 1), (u8)(mid - 1), (u8)(y + h - 2), OLED_WHITE);
}

static void UI_Page2(void)
{
    char buf[24];

    OLED_ShowString(2, 0, "MOTION", OLED_FONT_8);
    OLED_DrawPageDots(94, 2, UI_PAGE_NUM, g_ui.page);

    /* ---------- 三轴加速度：±2g ----------
     * 标签与条形框顶部对齐（原来是 y+1，会差 1 像素） */
    OLED_ShowString(0, 9, "AX", OLED_FONT_8);
    UI_BiBar(14, 9, 112, 8, g_ui.ax, 2000);

    OLED_ShowString(0, 19, "AY", OLED_FONT_8);
    UI_BiBar(14, 19, 112, 8, g_ui.ay, 2000);

    OLED_ShowString(0, 29, "AZ", OLED_FONT_8);
    UI_BiBar(14, 29, 112, 8, g_ui.az, 2000);

    /* ---------- 合加速度 ----------
     * 标签不能用 "|A|"：6x8 字模只到 'z'，竖线 0x7C 会被替换成 '?'。
     * 标签与数字统一用 8x16，避免一小一大显得字号不协调；
     * 单位 mg 用小字放在右下角当下标 */
    OLED_ShowString(0, 40, "MAG", OLED_FONT_16);
    OLED_ShowNum(28, 40, g_ui.mag, OLED_FONT_16);
    OLED_ShowString(66, 48, "mg", OLED_FONT_8);

    /* ---------- 三轴陀螺（整数近似，0.1°/s -> °/s） ---------- */
    if(g_ui.imu_err)
    {
        OLED_ShowString(0, 57, "IMU OFFLINE", OLED_FONT_8);
    }
    else
    {
        sprintf(buf, "G %d %d %d", g_ui.gx / 10, g_ui.gy / 10, g_ui.gz / 10);
        OLED_ShowString(0, 57, buf, OLED_FONT_8);
    }
}

/* ============================ 第 4 页：心率（新版界面） ============================
 * 布局（128x64）：
 *   y=0..7   状态栏：页名 + 心跳图标 + 页码指示器
 *   y=10..41 心率大字（16x32）+ 单位
 *   y=43     分隔线
 *   y=45..55 实时 PPG 波形（滚动，局部刷新）
 *   y=56..63 底栏：IBI / 峰值 / 状态
 * ============================================================================ */

/* 8x8 心形图标，bit7 在最左 */
static const u8 ICON_HEART[8] =
{
    0x00, 0x66, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x00
};

/* 画最近 5 秒的交流波形。只改显存，刷屏由调用方决定刷哪几页 */
static void UI_DrawWave(void)
{
    u8  x0 = 2, y0 = 45, w = 124, h = 11;
    u8  i, py, last_py = 0;
    s32 amp, mid, v;

    OLED_FillRect(x0, y0, (u8)(x0 + w - 1), (u8)(y0 + h - 1), OLED_BLACK);

    mid = (s32)y0 + h / 2;

    /* 纵向比例：peak_max 只是正峰幅度，负峰往往更大，直接当满量程会削顶，
     * 所以放宽 1.5 倍；信号弱时给一个下限，避免把噪声放大成毛刺 */
    amp = (s32)HR_GetPeakMax() * 3 / 2;
    if(amp < 200)  amp = 200;
    if(amp > 3000) amp = 3000;

    for(i = 0; i < w; i++)
    {
        u8  idx = (u8)(HR_WAVE_LEN - w + i);
        s16 w0  = HR_GetWave((u8)((idx + HR_WAVE_LEN - 1) % HR_WAVE_LEN));
        s16 w1  = HR_GetWave(idx);
        s16 w2  = HR_GetWave((u8)((idx + 1) % HR_WAVE_LEN));

        /* 显示前做 3 点加权平滑（只影响观感，不动算法数据），
         * 25Hz 原始波形带 5~12Hz 残留噪声，不平滑会画成密集毛刺 */
        v  = (s32)(w0 + w1 * 2 + w2) / 4;
        py = (u8)(mid - v * (h / 2) / amp);

        if(py < y0)          py = y0;
        if(py > y0 + h - 1)  py = (u8)(y0 + h - 1);

        if(i == 0)
        {
            OLED_DrawPoint(x0, py, OLED_WHITE);
        }
        else
        {
            /* 与上一点之间补竖线，否则快速变化处波形会断成散点 */
            u8 a = last_py, b = py, t;

            if(a > b) { t = a; a = b; b = t; }
            for(t = a; t <= b; t++)
                OLED_DrawPoint((u8)(x0 + i), t, OLED_WHITE);
        }

        last_py = py;
    }
}

/* 只重画心跳页的状态栏（8px 高，刷 1 页约 5ms），
 * 心跳闪烁时用它局部刷新，不必整屏重绘 */
static void UI_DrawStatusBar(void)
{
    OLED_FillRect(0, 0, 127, 7, OLED_BLACK);
    OLED_ShowString(2, 0, "HEART", OLED_FONT_8);

    if(g_ui.heart_on)                       /* 随心跳闪现 */
        OLED_DrawIcon8x8(66, 0, ICON_HEART);

    OLED_DrawPageDots(94, 2, UI_PAGE_NUM, g_ui.page);
}

static void UI_Page3(void)
{
    UI_DrawStatusBar();

    /* ---------- 心率大字 ---------- */
    if(g_ui.hr > 0)
    {
        OLED_ShowNum2x(4, 10, g_ui.hr);
        OLED_ShowString(56, 24, "bpm", OLED_FONT_8);
    }
    else
    {
        OLED_ShowChar2x(4, 10, '-');
        OLED_ShowChar2x(20, 10, '-');
        OLED_ShowString(56, 24, g_ui.finger ? "wait" : "bpm", OLED_FONT_8);
    }

    OLED_DrawLine(0, 43, 127, 43, OLED_WHITE);

    /* ---------- 波形 ---------- */
    UI_DrawWave();

    /* ---------- 底栏：用竖线把三块内容分开，避免标签和数值糊在一起 ---------- */
    OLED_ShowString(0, 56, "IBI", OLED_FONT_8);
    OLED_ShowNum(20, 56, g_ui.ibi, OLED_FONT_8);

    OLED_DrawLine(38, 56, 38, 63, OLED_WHITE);

    OLED_ShowString(44, 56, "PK", OLED_FONT_8);
    OLED_ShowNum(62, 56, g_ui.peak, OLED_FONT_8);

    OLED_DrawLine(80, 56, 80, 63, OLED_WHITE);

    if(g_ui.alarm)
        OLED_ShowString(84, 56, "ALARM", OLED_FONT_8);
    else if(!g_ui.finger)
        OLED_ShowString(84, 56, "NO FING", OLED_FONT_8);
    else if(g_ui.hr == 0)
        OLED_ShowString(84, 56, "MEASURE", OLED_FONT_8);
    else
        OLED_ShowString(84, 56, "STABLE", OLED_FONT_8);
}

/* ============================ 第 4 页：计步 ============================ */
/* ============================ 第 3 页：计步 ============================
 * 布局：
 *   y=0..7   状态栏
 *   y=9..40  步数大字（16x32）
 *   y=42     分隔线
 *   y=45..52 万步目标进度条
 *   y=56..63 底栏：步频 + 运动状态
 * ============================================================================ */
static void UI_Page4(void)
{
    u32 steps = g_ui.steps;
    u8  fill;

    OLED_ShowString(2, 0, "STEPS", OLED_FONT_8);
    OLED_DrawPageDots(94, 2, UI_PAGE_NUM, g_ui.page);

    /* 五位数字宽 80 像素，再多就画不全了 */
    if(steps > 99999)
        steps = 99999;

    OLED_ShowNum2x(4, 9, (s32)steps);

    OLED_DrawLine(0, 42, 127, 42, OLED_WHITE);

    /* ---------- 每日万步目标进度 ---------- */
    OLED_ShowString(0, 46, "TGT", OLED_FONT_8);
    OLED_DrawRect(24, 45, 123, 52, OLED_WHITE);

    if(g_ui.steps >= 10000)
        fill = 98;
    else
        fill = (u8)(g_ui.steps * 98 / 10000);

    if(fill > 0)
        OLED_FillRect(25, 46, (u8)(24 + fill), 51, OLED_WHITE);

    /* ---------- 底栏 ---------- */
    OLED_ShowString(0, 56, "CAD", OLED_FONT_8);
    OLED_ShowNum(20, 56, g_ui.cadence, OLED_FONT_8);
    OLED_ShowString(34, 56, "spm", OLED_FONT_8);

    if(g_ui.step_err)
        OLED_ShowString(70, 56, "I2C ERR", OLED_FONT_8);
    else if(g_ui.moving)
        OLED_ShowString(70, 56, "WALK", OLED_FONT_8);
    else
        OLED_ShowString(70, 56, "STAY", OLED_FONT_8);
}

/* ============================ 第 5 页：血氧 ============================
 * 布局：
 *   y=0..7   状态栏：页名 + 页码指示器
 *   y=9..40  SpO2 大字（16x32）+ 单位
 *   y=42     分隔线
 *   y=45..52 信号质量条（SIG）：灌注指数 PI 映射，判断这次测量值不值得信
 *   y=56..63 底栏：心率 + 状态
 * ============================================================================ */
static void UI_Page5(void)
{
    u8  fill;
    u8  px;

    OLED_ShowString(2, 0, "SpO2", OLED_FONT_8);
    OLED_DrawPageDots(94, 2, UI_PAGE_NUM, g_ui.page);

    /* ---------- SpO2 大字 ---------- */
    if(g_ui.spo2_valid)
    {
        OLED_ShowNum2x(4, 9, g_ui.spo2);
    }
    else
    {
        OLED_ShowChar2x(4, 9, '-');
        OLED_ShowChar2x(20, 9, '-');
    }

    /* 100% 时数字占 3 位，百分号要往右让开 */
    px = (g_ui.spo2_valid && g_ui.spo2 >= 100) ? 56 : 44;
    OLED_ShowString(px, 22, "%", OLED_FONT_16);

    OLED_DrawLine(0, 42, 127, 42, OLED_WHITE);

    /* ---------- 信号质量条 ----------
     * 外框 24..123，内部可填充宽度 98 像素 */
    OLED_ShowString(0, 46, "SIG", OLED_FONT_8);
    OLED_DrawRect(24, 45, 123, 52, OLED_WHITE);

    fill = (u8)((u16)g_ui.sig * 98 / 100);
    if(fill > 0)
        OLED_FillRect(25, 46, (u8)(24 + fill), 51, OLED_WHITE);

    /* ---------- 底栏 ---------- */
    OLED_ShowString(0, 56, "HR", OLED_FONT_8);
    OLED_ShowNum(16, 56, g_ui.hr, OLED_FONT_8);

    if(g_ui.spo2_alarm)
        OLED_ShowString(52, 56, "LOW O2", OLED_FONT_8);
    else if(!g_ui.finger)
        OLED_ShowString(52, 56, "NO FING", OLED_FONT_8);
    else if(!g_ui.spo2_ready)
        OLED_ShowString(52, 56, "FILL 5s", OLED_FONT_8);
    else if(g_ui.moving)
        OLED_ShowString(52, 56, "MOVE!", OLED_FONT_8);
    else if(g_ui.dc_ir > MAX30102_AGC_HI)
        OLED_ShowString(52, 56, "LOOSE", OLED_FONT_8);	/* 压太紧：表面反射强、血流被压闭 */
    else if(g_ui.spo2_valid)
        OLED_ShowString(52, 56, "REF VALID", OLED_FONT_8);
    else
        OLED_ShowString(52, 56, "WEAK SIG", OLED_FONT_8);
}

static void UI_Refresh(void)
{
    OLED_ClearBuffer();

    /* 页面顺序：P1 体温环境 / P2 六轴 / P3 计步 / P4 心率 / P5 血氧
     * UI_Page3 与 UI_Page4 是历史命名，与当前页码已不对应，按内容调用即可 */
    if(g_ui.page == 0)
        UI_Page1();
    else if(g_ui.page == 1)
        UI_Page2();
    else if(g_ui.page == 2)
        UI_Page4();                     /* 计步 */
    else if(g_ui.page == 3)
        UI_Page3();                     /* 心率 */
    else
        UI_Page5();

    OLED_Refresh();
}


int main(void)
{
    u8 key;
    u8 ret;
    char bt_cmd[24];
    u8 ui_dirty = 1;
    u8 ds_busy = 0;
    u16 tick10 = 0;
    u16 ds_wait = 0;
    u8 alarm_cnt = 0;                   /* 异常持续计数（2 秒一次） */
    u8 spo2_alarm_cnt = 0;              /* 低血氧持续计数（2 秒一次） */
    u8 alarm_blink = 0;                 /* 报警闪烁节拍 */
    u8 heart_flash = 0;                 /* 心跳图标点亮剩余轮数（10ms/轮） */
    u8 wave_tick = 0;                   /* 波形刷新节拍（60ms 一次） */

    SysTick_Init(72);
    USART1_Init(115200);
    USART2_Init(9600);                  /* 蓝牙：PA2/PA3，J1 短接 3-5、4-6 */
    OLED_Init();
    LED_Init();
    BUZZER_Init();
    RELAY_Init();
    KEY_Init();
    DHT11_Init();
    g_ui.body_ok = (DS18B20_Init() == DS18B20_OK) ? 1 : 0;

    MPU6050_Init();
    printf("MPU6050 ID = 0x%02X\r\n", MPU6050_GetID());

    STEP_Init();                        /* 计步：TIM2 时间基准 + 检测器清零 */

    MAX30102_Init();
    HR_Init();
    SPO2_Init();                        /* 血氧：500 点窗口 + 滑窗 */
    printf("MAX30102 ID = 0x%02X (%s)\r\n", MAX30102_GetPartID(),
           MAX30102_IsOnline() ? "OK" : "FAIL, check J1/I2C2/PB10-PB11");

    LED_Ctrl(LED3, LED_OFF);
    LED_Ctrl(LED4, LED_OFF);
    BUZZER_Ctrl(BUZZER_OFF);
    RELAY_Ctrl(RELAY_OFF);

    g_ui.page = 0;

    printf("\r\n==== Smart Band Demo ====\r\n");
    printf("KEY1=page  KEY2=LED  KEY3=BUZ  KEY4=RELAY\r\n");
    printf("P5 = SpO2: keep finger still 5s, REF value only\r\n");

    USART2_SendString("BT READY\r\n");
    USART2_SendString("[1]page [2]led [3]buz [4]rly\r\n");
    USART2_SendString("[0]stepclr DATA read h help\r\n");

    while(1)
    {
        /* 每轮把 MAX30102 硬件 FIFO 的样本搬进软件缓冲，保证数据新鲜 */
        MAX30102_Poll();

        /* 新样本立刻喂给心率算法，不在缓冲里积压 */
        {
            MAX30102_Sample_t s;

            while(MAX30102_Pop(&s) == 0)
            {
                /* 单一采样源、双消费者，绝不重复读 FIFO：
                 *   心率：100Hz -> 内部 4 点平均抽取到 25Hz，算法参数不变
                 *   血氧：100Hz 原始双通道，攒满 500 点做一次滑窗计算 */
                HR_Feed100Hz(s.ir);
                SPO2_Feed(s.red, s.ir);

                g_max_red = s.red;
                g_max_ir  = s.ir;
            }
        }

        /* ---------- 计步：每轮调用，内部按 50Hz 节拍读 MPU6050 ---------- */
        if(STEP_Poll())
        {
            g_ui.steps   = STEP_GetCount();
            g_ui.cadence = STEP_GetCadence();
            ui_dirty = 1;
            printf("[STEP] %lu  cadence=%d spm\r\n",
                   (unsigned long)g_ui.steps, (int)g_ui.cadence);
        }

        key = KEY_Scan();

        /* ---------- 蓝牙命令：1~4 等价按键，? 帮助，DATA 读一次当前数据 ---------- */
        if(key == KEY_NONE && USART2_GetLine(bt_cmd, 24))
        {
            printf("[BT] \"%s\" first=0x%02X\r\n", bt_cmd, (u8)bt_cmd[0]);

            if(bt_cmd[0] == '1')      key = KEY1;
            else if(bt_cmd[0] == '2') key = KEY2;
            else if(bt_cmd[0] == '3') key = KEY3;
            else if(bt_cmd[0] == '4') key = KEY4;
            else if(bt_cmd[0] == '0')      /* 步数清零 */
            {
                STEP_Reset();
                g_ui.steps   = 0;
                g_ui.cadence = 0;
                g_ui.step_mag = 0;
                g_ui.step_flt = 0;
                g_ui.moving  = 0;
                g_ui.step_err = 0;
                ui_dirty = 1;
                USART2_SendString("STEP CLEARED\r\n");
                printf("[STEP] cleared\r\n");
            }
            /* 帮助：半角 ?、字母 h/H，以及全角问号（GBK A3BF / UTF-8 EFBC9F）都认 */
            else if(bt_cmd[0] == '?' || bt_cmd[0] == 'h' || bt_cmd[0] == 'H'
                 || ((u8)bt_cmd[0] == 0xA3 && (u8)bt_cmd[1] == 0xBF)
                 || ((u8)bt_cmd[0] == 0xEF && (u8)bt_cmd[1] == 0xBC && (u8)bt_cmd[2] == 0x9F))
            {
                /* 帮助拆成两行短句：9600bps 下一行超过 40 字符，手机 App 容易折行错位 */
                USART2_SendString("[1] page  [2] led  [3] buz  [4] rly\r\n");
                USART2_SendString("[0] stepclr  DATA read  h help\r\n");
            }
            else if(bt_cmd[0] == 'D' || bt_cmd[0] == 'd')
            {
                /* 数据分两行发：一行体温环境，一行运动与血氧 */
                char tx[64];

                sprintf(tx, "BODY:%d.%dC ENV:%dC/%d%%\r\n",
                        g_ui.body / 10, g_ui.body % 10,
                        (int)g_ui.temp, (int)g_ui.humi);
                USART2_SendString(tx);

                sprintf(tx, "A:%dmg HR:%d STEP:%lu CAD:%d SPO2:%d%s%s\r\n",
                        g_ui.mag, (int)g_ui.hr,
                        (unsigned long)g_ui.steps, (int)g_ui.cadence,
                        (int)g_ui.spo2,                       /* 0 表示无效 */
                        g_ui.alarm ? " ALARM" : "",
                        g_ui.spo2_alarm ? " LOWO2" : "");
                USART2_SendString(tx);
            }
            else
            {
                /* 不再回显十六进制（ERR:35 这种看不懂），直接给可操作提示 */
                USART2_SendString("? unknown cmd, send h for help\r\n");
            }
        }

        switch(key)
        {
            case KEY1:                      /* 翻页 */
                g_ui.page = (u8)((g_ui.page + 1) % UI_PAGE_NUM);
                ui_dirty = 1;
                printf("[PAGE] -> %d\r\n", g_ui.page + 1);
                break;

            case KEY2:                      /* LED3/LED4 一起切换 */
                g_ui.led3 = !g_ui.led3;
                g_ui.led4 = g_ui.led3;
                ui_dirty = 1;
                break;

            case KEY3:                      /* 蜂鸣器 */
                g_ui.buz = !g_ui.buz;
                ui_dirty = 1;
                break;

            case KEY4:                      /* 继电器 */
                g_ui.rly = !g_ui.rly;
                ui_dirty = 1;
                break;

            default:
                break;
        }

        if(key != KEY_NONE)
        {
            g_ui.key_last = key;
            printf("[KEY%d] LED=%d BUZ=%d RELAY=%d PAGE=%d\r\n",
                   key, g_ui.led3, g_ui.buz, g_ui.rly, g_ui.page + 1);
        }

        tick10++;

        /* ---------- 每 2 秒：读环境 + 六轴，并启动体温转换 ---------- */
        if(tick10 >= 200)
        {
            tick10 = 0;

            ret = DHT11_Read_Data(&g_ui.temp, &g_ui.humi);
            g_ui.dht_err = ret;
            g_ui.dht_ok  = (ret == DHT11_OK);

            if(g_ui.dht_ok)
                printf("ENV  : %d C, %d %%RH\r\n", (int)g_ui.temp, (int)g_ui.humi);
            else
            {
                printf("DHT11 error, code = %d\r\n", ret);
                if(g_ui.buz == 0)
                {
                    BUZZER_Ctrl(BUZZER_ON);
                    delay_ms(100);
                    BUZZER_Ctrl(BUZZER_OFF);
                }
            }

            /* ---------- MPU6050 在线守护 ----------
             * 蓝牙模块收发时峰值电流可达几十毫安，会把 3.3V 拉低，
             * MPU6050 一旦掉电复位就回到 sleep 模式，数据冻结但 I2C 仍能应答。
             * 这里每 2 秒查一次 WHO_AM_I，发现不对就重新初始化，让它能自愈。 */
            if(!MPU6050_IsOnline())
            {
                MPU6050_BusUnlock();            /* 先从机可能把 SDA 拉死，先解锁总线 */
                MPU6050_Init();                 /* 重新写 PWR_MGMT_1 等配置，解除 sleep */
                g_ui.imu_err = 1;
                printf("[IMU] offline -> unlock+re-init, ID=0x%02X\r\n", MPU6050_GetID());
            }
            else
            {
                g_ui.imu_err = 0;
            }

            MPU6050_GetData(&g_ui.ax, &g_ui.ay, &g_ui.az,
                            &g_ui.gx, &g_ui.gy, &g_ui.gz);
            g_ui.mag = MPU6050_GetAccelMagnitudeMG();
            printf("ACC: %d %d %d | GYR: %d %d %d | |A| = %d mg\r\n",
                   g_ui.ax, g_ui.ay, g_ui.az, g_ui.gx, g_ui.gy, g_ui.gz, g_ui.mag);

            /* 计步结果（步数本身在 STEP_Poll 里实时更新，这里同步界面数据和状态） */
            g_ui.steps    = STEP_GetCount();
            g_ui.cadence  = STEP_GetCadence();
            g_ui.step_mag = STEP_GetRawMG();
            g_ui.step_flt = STEP_GetFilteredMG();
            g_ui.moving   = STEP_IsMoving();
            g_ui.step_err = (STEP_GetStatus() == STEP_STATUS_ERROR) ? 1 : 0;

            printf("STEP: %lu steps, %d spm, |A|=%d/%d mg, %s%s\r\n",
                   (unsigned long)g_ui.steps, (int)g_ui.cadence,
                   (int)g_ui.step_mag, (int)g_ui.step_flt,
                   g_ui.moving ? "MOVING" : "STATIC",
                   g_ui.step_err ? " SENSOR-ERR" : "");

            /* MAX30102 + 心率 */
            g_ui.hr     = HR_GetRate();
            g_ui.ibi    = HR_GetIBI();
            g_ui.peak   = (u16)HR_GetPeakMax();
            g_ui.finger = HR_IsFingerOn();

            printf("MAX : RED=%lu IR=%lu DC=%lu peak=%lu | HR=%d IBI=%d f=%d s=%d\r\n",
                   g_max_red, g_max_ir, HR_GetDC(), HR_GetPeakMax(),
                   (int)HR_GetRate(), (int)HR_GetIBI(),
                   (int)HR_IsFingerOn(), (int)HR_IsStable());

            /* ---------- 血氧：算法每秒刷新一次，运动时暂停采信 ---------- */
            g_ui.spo2_ready = SPO2_IsReady();
            g_ui.spo2       = SPO2_GetValue();
            g_ui.spo2_hr    = SPO2_GetHR();
            g_ui.spo2_r     = SPO2_GetR1000();
            g_ui.spo2_valid = (SPO2_IsValid() && !g_ui.moving) ? 1 : 0;
            g_ui.dc_ir      = SPO2_GetDCIr();
            g_ui.sig        = SPO2_GetQuality();

            printf("SPO2: %d%% HR2=%d P=%d R=%d.%03d PI=%d.%02d SIG=%d DCr=%lu DCi=%lu valid=%d\r\n",
                   (int)g_ui.spo2, (int)g_ui.spo2_hr, (int)SPO2_GetPeaks(),
                   (int)(g_ui.spo2_r / 1000), (int)(g_ui.spo2_r % 1000),
                   (int)(SPO2_GetPI10000() / 100), (int)(SPO2_GetPI10000() % 100),
                   (int)g_ui.sig,
                   SPO2_GetDCRed(), SPO2_GetDCIr(),
                   (int)g_ui.spo2_valid);

            /* LED 电流自动增益：压太紧 DC 冲高到接近饱和，拿远又太暗，
             * 两种情况都会让 AC 提取失败，这里按实测 DC 把工作点拉回量程中段 */
            if(MAX30102_AgcStep(HR_GetDC()))
            {
                SPO2_Reset();               /* 电流变了，旧窗口的 DC 基准不一致，重新积累 5 秒 */
                g_ui.dc_ir = 0;
                printf("[AGC] IR LED = 0x%02X, DC=%lu\r\n",
                       MAX30102_GetLedPa(), HR_GetDC());
            }

            /* 心率异常判断：连续异常约 10 秒才确认，恢复正常立即解除 */
            if(g_ui.finger && g_ui.hr > 0 &&
               (g_ui.hr < HR_ALARM_LOW || g_ui.hr > HR_ALARM_HIGH))
            {
                if(alarm_cnt < HR_ALARM_CONFIRM)
                    alarm_cnt++;
            }
            else
            {
                alarm_cnt = 0;
            }

            if(alarm_cnt >= HR_ALARM_CONFIRM)
            {
                if(!g_ui.alarm)
                {
                    g_ui.alarm = 1;
                    ui_dirty = 1;
                    printf("[ALARM] HR=%d bpm, out of %d~%d\r\n",
                           (int)g_ui.hr, HR_ALARM_LOW, HR_ALARM_HIGH);
                }
            }
            else if(g_ui.alarm)
            {
                g_ui.alarm = 0;
                ui_dirty = 1;
                printf("[ALARM] cleared, HR=%d bpm\r\n", (int)g_ui.hr);
            }

            /* 低血氧判断：只采信"算法有效 + 未运动"的结果，连续约 10 秒才确认 */
            if(g_ui.spo2_valid && g_ui.spo2 > 0 && g_ui.spo2 < SPO2_ALARM_LOW)
            {
                if(spo2_alarm_cnt < SPO2_ALARM_CONFIRM)
                    spo2_alarm_cnt++;
            }
            else
            {
                spo2_alarm_cnt = 0;
            }

            if(spo2_alarm_cnt >= SPO2_ALARM_CONFIRM)
            {
                if(!g_ui.spo2_alarm)
                {
                    g_ui.spo2_alarm = 1;
                    ui_dirty = 1;
                    printf("[ALARM] SpO2=%d%%, below %d%%\r\n",
                           (int)g_ui.spo2, SPO2_ALARM_LOW);
                }
            }
            else if(g_ui.spo2_alarm)
            {
                g_ui.spo2_alarm = 0;
                ui_dirty = 1;
                printf("[ALARM] SpO2 cleared, now %d%%\r\n", (int)g_ui.spo2);
            }

            if(!ds_busy)
            {
                if(DS18B20_StartConvert() == DS18B20_OK)
                {
                    ds_busy = 1;
                    ds_wait = 0;
                }
                else
                {
                    g_ui.body_ok = 0;
                }
            }

            ui_dirty = 1;
        }

        /* ---------- 体温转换满 800ms 后读取 ---------- */
        if(ds_busy)
        {
            ds_wait++;
            if(ds_wait >= 80)
            {
                ds_busy = 0;
                g_ui.body_ok = (DS18B20_ReadTemp(&g_ui.body) == DS18B20_OK) ? 1 : 0;

                if(g_ui.body_ok)
                    printf("BODY : %d.%d C\r\n", g_ui.body / 10, g_ui.body % 10);

                ui_dirty = 1;
            }
        }

        /* ---------- 驱动执行器 ---------- */
        if(g_ui.alarm || g_ui.spo2_alarm)
        {
            /* 报警优先：蜂鸣器 + 两颗 LED 同步间歇闪烁（约 320ms 一拍）
             * 心率异常与低血氧共用同一套闪烁，避免两个报警互相抢节拍 */
            alarm_blink++;
            if(alarm_blink & 0x20)
            {
                BUZZER_Ctrl(BUZZER_ON);
                LED_Ctrl(LED3, LED_ON);
                LED_Ctrl(LED4, LED_ON);
            }
            else
            {
                BUZZER_Ctrl(BUZZER_OFF);
                LED_Ctrl(LED3, LED_OFF);
                LED_Ctrl(LED4, LED_OFF);
            }
        }
        else
        {
            if(g_ui.led3) LED_Ctrl(LED3, LED_ON); else LED_Ctrl(LED3, LED_OFF);
            if(g_ui.led4) LED_Ctrl(LED4, LED_ON); else LED_Ctrl(LED4, LED_OFF);
            if(g_ui.buz)  BUZZER_Ctrl(BUZZER_ON); else BUZZER_Ctrl(BUZZER_OFF);
        }

        if(g_ui.rly)  RELAY_Ctrl(RELAY_ON);   else RELAY_Ctrl(RELAY_OFF);

        if(ui_dirty)
        {
            ui_dirty = 0;
            UI_Refresh();
        }

        /* ---------- 心率页动态元素（只在该页刷新，避免拖慢 100Hz 采集） ----------
         * 心跳图标：每次检测到心跳点亮 80ms，只刷状态栏所在 1 页（约 5ms）
         * 波形：60ms 滚动一次，只刷波形所在 2 页（约 10ms） */
        if(g_ui.page == 3)
        {
            if(HR_GetBeat())
            {
                heart_flash = 8;
                if(!g_ui.heart_on)
                {
                    g_ui.heart_on = 1;
                    UI_DrawStatusBar();
                    OLED_RefreshPages(0, 0);
                }
            }
            else if(heart_flash > 0)
            {
                if(--heart_flash == 0)
                {
                    g_ui.heart_on = 0;
                    UI_DrawStatusBar();
                    OLED_RefreshPages(0, 0);
                }
            }

            if(++wave_tick >= 6)
            {
                wave_tick = 0;
                UI_DrawWave();
                OLED_RefreshPages(5, 6);
            }
        }
        else
        {
            heart_flash   = 0;
            wave_tick     = 0;
            g_ui.heart_on = 0;
        }

        delay_ms(10);
    }
}
