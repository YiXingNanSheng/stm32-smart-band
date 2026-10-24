#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "buzzer.h"
#include "relay.h"
#include "key.h"
#include "dht11.h"
#include "ds18b20.h"
#include "mpu6050.h"
#include "max30102.h"
#include "hr.h"
#include "usart.h"
#include "usart2.h"
#include "oled.h"

#define UI_PAGE_NUM		3
/* 两层宏：先把 UI_PAGE_NUM 展开成 3，再字符串化成 "3"，
   这样改页数只需要改上面一个数字，页码显示自动跟着变 */
#define UI_STR_(x)		#x
#define UI_STR(x)		UI_STR_(x)
#define UI_PAGE_TOTAL	UI_STR(UI_PAGE_NUM)

/* ============================ 心率异常报警 ============================
 * 判定条件：手指在位、心率有效，且超出 [LOW, HIGH] 区间
 * 确认时间：连续 CONFIRM 次 2 秒任务（约 10 秒），避免单次误检就报警
 * 报警动作：蜂鸣器 + LED3/LED4 间歇闪烁，覆盖手动开关状态
 * ==================================================================== */
#define HR_ALARM_LOW		50			/* 低于此值：心动过缓 */
#define HR_ALARM_HIGH		110		/* 高于此值：心动过速 */
#define HR_ALARM_CONFIRM	5			/* 连续 5 次（约 10 秒）才确认 */

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
} UI_Data_t;

static UI_Data_t g_ui;

/* 最新一组 MAX30102 原始值，供界面和蓝牙上报使用 */
static u32 g_max_red = 0;
static u32 g_max_ir  = 0;


/* 把 mg 转成 "1.13" 形式的字符串（单位 g，两位小数） */
static void Fmt_G(char *s, s16 mg)
{
    u8 i = 0;
    u16 a, h;

    if(mg < 0) { s[i++] = '-'; a = (u16)(-mg); }
    else       { a = (u16)mg; }

    h = (u16)((a + 5) / 10);            /* 0.01g 为单位，四舍五入 */

    if(h >= 1000) s[i++] = (char)('0' + (h / 1000));
    s[i++] = (char)('0' + (h / 100) % 10);
    s[i++] = '.';
    s[i++] = (char)('0' + (h / 10) % 10);
    s[i++] = (char)('0' + h % 10);
    s[i]   = 0;
}

/* 把 0.1°/s 转成 "-5.1" 形式的字符串 */
static void Fmt_DPS1(char *s, s16 dps10)
{
    u8 i = 0;
    u16 a;

    if(dps10 < 0) { s[i++] = '-'; a = (u16)(-dps10); }
    else          { a = (u16)dps10; }

    if(a >= 1000) s[i++] = (char)('0' + (a / 1000));
    s[i++] = (char)('0' + (a / 100) % 10);
    s[i++] = '.';
    s[i++] = (char)('0' + a % 10);
    s[i]   = 0;
}


/* ============================ 第 1 页：体温 + 环境 + 执行器 ============================ */
static void UI_Page1(void)
{
    OLED_ShowString(0, 0, "BODY", OLED_FONT_8);
    OLED_ShowString(100, 0, "P1/" UI_PAGE_TOTAL, OLED_FONT_8);

    if(g_ui.body_ok)
    {
        OLED_ShowNum(0, 8, g_ui.body / 10, OLED_FONT_16);
        OLED_ShowString(16, 8, ".", OLED_FONT_16);
        OLED_ShowNum(24, 8, g_ui.body % 10, OLED_FONT_16);
    }
    else
    {
        OLED_ShowString(0, 8, "--.-", OLED_FONT_16);
    }
    OLED_ShowString(32, 8, "C", OLED_FONT_16);

    OLED_DrawLine(0, 26, 127, 26, OLED_WHITE);

    OLED_ShowString(0, 30, "ENV", OLED_FONT_8);
    OLED_ShowNum(18, 30, g_ui.temp, OLED_FONT_8);
    OLED_ShowString(30, 30, "C", OLED_FONT_8);
    OLED_ShowString(42, 30, "H", OLED_FONT_8);
    OLED_ShowNum(48, 30, g_ui.humi, OLED_FONT_8);
    OLED_ShowString(60, 30, "%", OLED_FONT_8);

    if(g_ui.dht_ok)
        OLED_ShowString(100, 30, "OK", OLED_FONT_8);
    else
    {
        OLED_ShowString(100, 30, "E", OLED_FONT_8);
        OLED_ShowNum(106, 30, g_ui.dht_err, OLED_FONT_8);
    }

    OLED_ShowString(0, 40, "LED3:", OLED_FONT_8);
    OLED_ShowNum(30, 40, g_ui.led3, OLED_FONT_8);
    OLED_ShowString(42, 40, "LED4:", OLED_FONT_8);
    OLED_ShowNum(72, 40, g_ui.led4, OLED_FONT_8);

    OLED_ShowString(0, 50, "RLY:", OLED_FONT_8);
    OLED_ShowNum(24, 50, g_ui.rly, OLED_FONT_8);
    OLED_ShowString(36, 50, "BUZ:", OLED_FONT_8);
    OLED_ShowNum(60, 50, g_ui.buz, OLED_FONT_8);

    OLED_DrawRect(116, 49, 126, 57, OLED_WHITE);
    if(g_ui.rly)
        OLED_FillRect(117, 50, 125, 56, OLED_WHITE);
}

/* ============================ 第 2 页：MPU6050 六轴 ============================ */
static void UI_Page2(void)
{
    char buf[10];

    OLED_ShowString(0, 0, "MPU6050", OLED_FONT_8);
    OLED_ShowString(100, 0, "P2/" UI_PAGE_TOTAL, OLED_FONT_8);

    /* 加速度，单位 g */
    OLED_ShowString(0, 9, "A(g) X", OLED_FONT_8);
    Fmt_G(buf, g_ui.ax);
    OLED_ShowString(36, 9, buf, OLED_FONT_8);
    OLED_ShowString(72, 9, "Y", OLED_FONT_8);
    Fmt_G(buf, g_ui.ay);
    OLED_ShowString(80, 9, buf, OLED_FONT_8);

    OLED_ShowString(0, 18, "Z", OLED_FONT_8);
    Fmt_G(buf, g_ui.az);
    OLED_ShowString(8, 18, buf, OLED_FONT_8);

    /* 陀螺，单位 °/s */
    OLED_ShowString(0, 27, "G(dps)X", OLED_FONT_8);
    Fmt_DPS1(buf, g_ui.gx);
    OLED_ShowString(42, 27, buf, OLED_FONT_8);
    OLED_ShowString(72, 27, "Y", OLED_FONT_8);
    Fmt_DPS1(buf, g_ui.gy);
    OLED_ShowString(80, 27, buf, OLED_FONT_8);

    OLED_ShowString(0, 36, "Z", OLED_FONT_8);
    Fmt_DPS1(buf, g_ui.gz);
    OLED_ShowString(8, 36, buf, OLED_FONT_8);

    /* 合加速度 */
    OLED_ShowString(0, 45, "|A|", OLED_FONT_8);
    OLED_ShowNum(24, 45, g_ui.mag, OLED_FONT_8);
    OLED_ShowString(54, 45, "mg", OLED_FONT_8);

    /* 提示：静止时应接近 1000mg */
    OLED_ShowString(0, 55, "static ~1000mg", OLED_FONT_8);
}

/* ============================ 第 3 页：心率 ============================ */
static void UI_Page3(void)
{
    OLED_ShowString(0, 0, "HEART RATE", OLED_FONT_8);
    OLED_ShowString(100, 0, "P3/" UI_PAGE_TOTAL, OLED_FONT_8);

    if(g_ui.hr > 0)
    {
        OLED_ShowNum(28, 8, g_ui.hr, OLED_FONT_16);
        OLED_ShowString(64, 16, "bpm", OLED_FONT_8);
    }
    else if(g_ui.finger)
    {
        OLED_ShowString(16, 12, "measuring", OLED_FONT_8);
    }
    else
    {
        OLED_ShowString(28, 12, "--", OLED_FONT_16);
    }

    OLED_DrawLine(0, 28, 127, 28, OLED_WHITE);

    if(g_ui.alarm)
        OLED_ShowString(0, 32, "!! HR ALARM !!", OLED_FONT_8);
    else if(g_ui.finger)
        OLED_ShowString(0, 32, "FINGER ON", OLED_FONT_8);
    else
        OLED_ShowString(0, 32, "PUT FINGER ON", OLED_FONT_8);

    OLED_ShowString(0, 42, "IBI:", OLED_FONT_8);
    OLED_ShowNum(24, 42, g_ui.ibi, OLED_FONT_8);
    OLED_ShowString(48, 42, "smp", OLED_FONT_8);

    OLED_ShowString(0, 52, "Peak:", OLED_FONT_8);
    OLED_ShowNum(36, 52, g_ui.peak, OLED_FONT_8);
}

static void UI_Refresh(void)
{
    OLED_ClearBuffer();

    if(g_ui.page == 0)
        UI_Page1();
    else if(g_ui.page == 1)
        UI_Page2();
    else
        UI_Page3();

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
    u8 alarm_blink = 0;                 /* 报警闪烁节拍 */

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

    MAX30102_Init();
    HR_Init();
    printf("MAX30102 ID = 0x%02X (%s)\r\n", MAX30102_GetPartID(),
           MAX30102_IsOnline() ? "OK" : "FAIL, check J1/I2C2/PB10-PB11");

    LED_Ctrl(LED3, LED_OFF);
    LED_Ctrl(LED4, LED_OFF);
    BUZZER_Ctrl(BUZZER_OFF);
    RELAY_Ctrl(RELAY_OFF);

    g_ui.page = 0;

    printf("\r\n==== Smart Band Demo ====\r\n");
    printf("KEY1=page  KEY2=LED  KEY3=BUZ  KEY4=RELAY\r\n");

    USART2_SendString("BT READY\r\n");
    USART2_SendString("cmd: 1/2/3/4  ?=help  DATA=read\r\n");

    while(1)
    {
        /* 每轮把 MAX30102 硬件 FIFO 的样本搬进软件缓冲，保证数据新鲜 */
        MAX30102_Poll();

        /* 新样本立刻喂给心率算法，不在缓冲里积压 */
        {
            MAX30102_Sample_t s;

            while(MAX30102_Pop(&s) == 0)
            {
                HR_Feed(s.ir);
                g_max_red = s.red;
                g_max_ir  = s.ir;
            }
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
            /* 帮助：半角 ?、字母 h/H，以及全角问号（GBK A3BF / UTF-8 EFBC9F）都认 */
            else if(bt_cmd[0] == '?' || bt_cmd[0] == 'h' || bt_cmd[0] == 'H'
                 || ((u8)bt_cmd[0] == 0xA3 && (u8)bt_cmd[1] == 0xBF)
                 || ((u8)bt_cmd[0] == 0xEF && (u8)bt_cmd[1] == 0xBC && (u8)bt_cmd[2] == 0x9F))
            {
                USART2_SendString("1:page 2:led 3:buz 4:rly DATA:read  h:help\r\n");
            }
            else if(bt_cmd[0] == 'D' || bt_cmd[0] == 'd')
            {
                char tx[64];
                sprintf(tx, "BODY:%d.%d ENV:%dC/%d%% A:%dmg HR:%d%s\r\n",
                        g_ui.body / 10, g_ui.body % 10,
                        (int)g_ui.temp, (int)g_ui.humi, g_ui.mag, (int)g_ui.hr,
                        g_ui.alarm ? " ALARM" : "");
                USART2_SendString(tx);
            }
            else
            {
                char tx[16];
                sprintf(tx, "ERR:%02X\r\n", (u8)bt_cmd[0]);   /* 回显首字节，便于看编码 */
                USART2_SendString(tx);
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

            MPU6050_GetData(&g_ui.ax, &g_ui.ay, &g_ui.az,
                            &g_ui.gx, &g_ui.gy, &g_ui.gz);
            g_ui.mag = MPU6050_GetAccelMagnitudeMG();
            printf("ACC: %d %d %d | GYR: %d %d %d | |A| = %d mg\r\n",
                   g_ui.ax, g_ui.ay, g_ui.az, g_ui.gx, g_ui.gy, g_ui.gz, g_ui.mag);

            /* MAX30102 + 心率 */
            g_ui.hr     = HR_GetRate();
            g_ui.ibi    = HR_GetIBI();
            g_ui.peak   = (u16)HR_GetPeakMax();
            g_ui.finger = HR_IsFingerOn();

            printf("MAX : RED=%lu IR=%lu DC=%lu peak=%lu | HR=%d IBI=%d f=%d s=%d\r\n",
                   g_max_red, g_max_ir, HR_GetDC(), HR_GetPeakMax(),
                   (int)HR_GetRate(), (int)HR_GetIBI(),
                   (int)HR_IsFingerOn(), (int)HR_IsStable());

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
        if(g_ui.alarm)
        {
            /* 报警优先：蜂鸣器 + 两颗 LED 同步间歇闪烁（约 320ms 一拍） */
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

        delay_ms(10);
    }
}
