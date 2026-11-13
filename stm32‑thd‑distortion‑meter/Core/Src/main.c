/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 1024
#define FFT_SIZE 1024
#define SAMPLE_RATE 100000

/* SH1106 OLED (I2C) */
#define OLED_ADDR 0x3C
#define OLED_I2C_ADDR (OLED_ADDR << 1)
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES (OLED_HEIGHT / 8)

/* 1 = 软件自测链路（假信号�? adc_buf），0 = 真实 ADC 采样 */
#define USE_ADC_SELF_TEST 0
#define USE_HANN_WINDOW 1

/* 调试打印开关：ADC raw / VREFINT 等联调打印，出报告时关掉 */
#define USE_DEBUG_PRINT 1
/* 0 = 一键测量（按 PA0 按键或串口发任意字符，才测一次）；1 = 连续测量（台上调试方便） */
#define USE_CONTINUOUS 0
/* 开机读一次内部 VREFINT 自检。注意：它内部会 HAL_ADC_Stop()，
 * 实测会干扰后面的 DMA 采集（导致 adc_buf 恒 0），平时务必保持 0！ */
#define USE_VREFINT_SELFTEST 0

/* 【基础/发挥 切换开关】这里只决定“上电默认模式”：
 *   0 = 基础部分：基频固定 1kHz、采样率固定约 102.4kHz（相干采样），多帧平均
 *   1 = 发挥部分：500Hz~50kHz 任意频率，两遍自适应采样率精测（走 measure_thd_auto）
 * 运行中可长按 PA0（≥0.8s）在两种模式间来回切换（见 wait_for_trigger）。 */
#define USE_ADVANCED_MODE 1

/* 蓝牙发送一周期波形。发挥部分手机显示波形需要它；
 * 若只想看 THD/频率/谐波结果，改成 0 可省掉约 0.8s 的串口发送时间。 */
#define BT_SEND_WAVE 1

/* 蓝牙波形格式：
 *   1 = bt_mcu App 二进制数据包（A5 + float32 + 校验 + 5A），
 *       手机 App 必须切到“专业调试”，配好接收数据包和 Y-T 波形控件；
 *   0 = 旧 ASCII 文本帧 "W,v0,...,v63\r\n"（普通串口助手/对话模式可读）。 */
#define BT_WAVE_FORMAT 1
/* 每周期发 50 点：App 默认窗口 100 点 → 显示约两个周期；
 * 每个点都要连带 6 个结果 float（共 31 字节），50 点约 1.6s，100 点会超过 3s。 */
#define BT_WAVE_POINTS 50

/* 多帧功率谱平均(Welch)：采 AVG_FRAMES 帧、平均“功率谱”后再算 THD，
 * 随机噪声地板降 1/√K（K=16 → 降 4 倍）。大信号(600mV)读数不变，
 * 收益主要在小信号档(30mV)。 */
#define AVG_FRAMES 16

/* 【Phase E 一键启动】板载按键：默认按常见 KEY_UP 接法（PA0，按下=高电平）。
 * 若你的板子按键在别的脚 / 以低电平为按下，改下面两行 + BTN_PRESSED 即可。 */
#define BTN_PORT     GPIOA
#define BTN_PIN      GPIO_PIN_0
#define BTN_PRESSED  GPIO_PIN_SET
#define LONG_PRESS_MS 800u   /* 按住超过这个时间 = 长按，切换基础/发挥模式 */

/* 波形还原显示区：OLED 底部 32 像素（page 4~7） */
#define WAVE_Y0 32
#define WAVE_H  32
#define WAVE_X  128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_buf[ADC_BUF_SIZE];
uint16_t adc_snap[ADC_BUF_SIZE];        /* 计算用快照：DMA 暂停时整帧拷出，算频谱/波形都读它 */
float fft_re[FFT_SIZE];
float fft_im[FFT_SIZE];
float fft_mag[FFT_SIZE / 2];
float g_thd;
float g_fund_freq;
float g_harm[5];
uint32_t g_sample_rate = SAMPLE_RATE;   /* 当前实际采样率：被 set_sample_rate 改，频率换算用它 */
int g_f0_bin = 0;                        /* 最近一次检测到的基波整数 bin（诊断用） */
volatile uint8_t g_advanced_mode = USE_ADVANCED_MODE;  /* 运行模式：1=发挥，0=基础；长按切换 */
static uint8_t oled_buf[OLED_PAGES][OLED_WIDTH];
static uint8_t oled_tx_buf[OLED_WIDTH + 1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* ==================== SH1106 OLED 驱动（I2C�? ==================== */

/* 8x8 点阵字库，ASCII 0x20~0x7E，每行一个字节，bit7 在最左边 */
static const uint8_t font8x8[95][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
  {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, /* '!' */
  {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, /* '"' */
  {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, /* '#' */
  {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, /* '$' */
  {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, /* '%' */
  {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, /* '&' */
  {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, /* ''' */
  {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, /* '(' */
  {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, /* ')' */
  {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, /* '*' */
  {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, /* '+' */
  {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, /* ',' */
  {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, /* '-' */
  {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, /* '.' */
  {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, /* '/' */
  {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, /* '0' */
  {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, /* '1' */
  {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, /* '2' */
  {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, /* '3' */
  {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, /* '4' */
  {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, /* '5' */
  {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, /* '6' */
  {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, /* '7' */
  {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, /* '8' */
  {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, /* '9' */
  {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, /* ':' */
  {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, /* ';' */
  {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, /* '<' */
  {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, /* '=' */
  {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, /* '>' */
  {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, /* '?' */
  {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, /* '@' */
  {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, /* 'A' */
  {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, /* 'B' */
  {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, /* 'C' */
  {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, /* 'D' */
  {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, /* 'E' */
  {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, /* 'F' */
  {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, /* 'G' */
  {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, /* 'H' */
  {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'I' */
  {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, /* 'J' */
  {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, /* 'K' */
  {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, /* 'L' */
  {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, /* 'M' */
  {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, /* 'N' */
  {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, /* 'O' */
  {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, /* 'P' */
  {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, /* 'Q' */
  {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, /* 'R' */
  {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, /* 'S' */
  {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'T' */
  {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, /* 'U' */
  {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 'V' */
  {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, /* 'W' */
  {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, /* 'X' */
  {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, /* 'Y' */
  {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, /* 'Z' */
  {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, /* '[' */
  {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, /* '\' */
  {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, /* ']' */
  {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, /* '^' */
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, /* '_' */
  {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, /* '`' */
  {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, /* 'a' */
  {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, /* 'b' */
  {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, /* 'c' */
  {0x38,0x30,0x30,0x3E,0x33,0x33,0x6E,0x00}, /* 'd' */
  {0x00,0x00,0x1E,0x33,0x3F,0x03,0x1E,0x00}, /* 'e' */
  {0x1C,0x36,0x06,0x0F,0x06,0x06,0x0F,0x00}, /* 'f' */
  {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, /* 'g' */
  {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, /* 'h' */
  {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'i' */
  {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, /* 'j' */
  {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, /* 'k' */
  {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'l' */
  {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, /* 'm' */
  {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, /* 'n' */
  {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, /* 'o' */
  {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, /* 'p' */
  {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, /* 'q' */
  {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, /* 'r' */
  {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, /* 's' */
  {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, /* 't' */
  {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, /* 'u' */
  {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 'v' */
  {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, /* 'w' */
  {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, /* 'x' */
  {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, /* 'y' */
  {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, /* 'z' */
  {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, /* '{' */
  {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, /* '|' */
  {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, /* '}' */
  {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, /* '~' */
};

/* 发一条命令：控制字节 0x00 + 命令字节 */
static void oled_write_cmd(uint8_t cmd)
{
  uint8_t data[2];
  data[0] = 0x00;
  data[1] = cmd;
  HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, data, 2, 50);
}

static void oled_clear(void)
{
  memset(oled_buf, 0, sizeof(oled_buf));
}

/* 把整个帧缓冲刷到屏上�?8 �? page，每�? page �?列起始地�? + 128 字节数据 */
static void oled_update(void)
{
  for (uint8_t page = 0; page < OLED_PAGES; page++)
  {
    oled_write_cmd(0xB0 | page);   /* 页地�? */
    oled_write_cmd(0x02);          /* 列低 4 �? = 2（SH1106 左边�? 2 列空�? */
    oled_write_cmd(0x10);          /* 列高 4 �? = 0 */
    oled_tx_buf[0] = 0x40;         /* 控制字节：数�? */
    memcpy(&oled_tx_buf[1], oled_buf[page], OLED_WIDTH);
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, oled_tx_buf, OLED_WIDTH + 1, 50);
  }
}

/* �? (x, page) 画一�? 8x8 字符；page �?"�?"�?0~7），每行 8 像素�? */
static void oled_draw_char(uint8_t x, uint8_t page, char c)
{
  if (c < 0x20 || c > 0x7E) c = ' ';
  const uint8_t *g = font8x8[c - 0x20];
  for (uint8_t row = 0; row < 8; row++)
  {
    uint8_t bits = g[row];               /* 这一行，bit0 在最�? */
    for (uint8_t col = 0; col < 8; col++)
    {
      if (bits & (0x01u << col))
      {
        oled_buf[page][x + col] |= (uint8_t)(1u << row);
      }
    }
  }
}

static void oled_draw_string(uint8_t x, uint8_t page, const char *s)
{
  while (*s)
  {
    oled_draw_char(x, page, *s++);
    x += 8;
    if (x >= OLED_WIDTH - 8) break;
  }
}

/* �? float 转成字符串（保留 decimals 位小数）。nano �? printf 不支�? %f，所以自己写 */
static void oled_ftoa(float value, int decimals, char *out)
{
  char buf[16];
  long pow10[6] = {1, 10, 100, 1000, 10000, 100000};
  if (decimals > 5) decimals = 5;

  int neg = (value < 0.0f);
  if (neg) value = -value;

  long scale = pow10[decimals];
  long ipart = (long)value;
  long fpart = (long)((value - (float)ipart) * (float)scale + 0.5f);
  if (fpart >= scale) { ipart++; fpart = 0; }

  int idx = 0;
  if (neg) buf[idx++] = '-';
  idx += sprintf(&buf[idx], "%ld", ipart);
  if (decimals > 0)
  {
    buf[idx++] = '.';
    for (int i = decimals - 1; i >= 0; i--)
    {
      long d = pow10[i];
      buf[idx++] = (char)('0' + (fpart / d) % 10);
    }
  }
  buf[idx] = '\0';
  strcpy(out, buf);
}

static void oled_init(void)
{
  static const uint8_t init_cmd[] = {
    0xAE,        /* 关显�? */
    0xD5, 0x80,  /* 时钟分频 */
    0xA8, 0x3F,  /* 复用�? 64 */
    0xD3, 0x00,  /* 显示偏移 0 */
    0x40,        /* 起始�? 0 */
    0xAD, 0x8B,  /* 内部 DC-DC �?（SH1106 特有�? */
    0x33,        /* 泵电�? 9.0V */
    0xA0,        /* 段重映射 */
    0xC0,        /* COM 扫描方向 */
    0xDA, 0x12,  /* COM 引脚配置 */
    0x81, 0xCF,  /* 对比�? */
    0xD9, 0xF1,  /* 预充�? */
    0xDB, 0x40,  /* VCOM �?�? */
    0xA4,        /* 关闭全显 */
    0xA6,        /* 正常显示 */
    0xAF,        /* �?显示 */
  };
  for (uint32_t i = 0; i < sizeof(init_cmd); i++)
  {
    oled_write_cmd(init_cmd[i]);
  }
  oled_clear();
  oled_update();
}

/* 迭代�?? radix-2 FFT：输�?? re[]/im[]（复数的实部虚部），原地变换 */
void fft(float *re, float *im, int n)
{
  /* �?? 1 步：位反转重排，把数据按 FFT �??要的顺序摆好 */
  for (int i = 1, j = 0; i < n; i++)
  {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j)
    {
      float tr = re[i]; re[i] = re[j]; re[j] = tr;
      float ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
  }
  /* �?? 2 步：蝶形运算，�?�层合并 */
  for (int len = 2; len <= n; len <<= 1)
  {
    float ang = -2.0f * 3.14159265358979f / len;
    float w_re = cosf(ang), w_im = sinf(ang);
    for (int i = 0; i < n; i += len)
    {
      float cur_re = 1.0f, cur_im = 0.0f;
      for (int k = 0; k < len / 2; k++)
      {
        int a = i + k;
        int b = a + len / 2;
        float tre = re[b] * cur_re - im[b] * cur_im;
        float tim = re[b] * cur_im + im[b] * cur_re;
        re[b] = re[a] - tre;
        im[b] = im[a] - tim;
        re[a] = re[a] + tre;
        im[a] = im[a] + tim;
        float nre = cur_re * w_re - cur_im * w_im;
        cur_im = cur_re * w_im + cur_im * w_re;
        cur_re = nre;
      }
    }
  }
}

/* 自测：软件�?�一个�?�基�??+2~5次谐波�?�的正弦，跑 FFT �?? THD，验证算法对不对 */
void run_fft_self_test(void)
{
  const float fs = 100000.0f;   /* 采样�?? 100kHz */
  const int f0_bin = 10;        /* 基波在第 10 �?? bin，即 976.56Hz */
  const float A1 = 1.0f, A2 = 0.05f, A3 = 0.03f, A4 = 0.02f, A5 = 0.01f;

  for (int n = 0; n < FFT_SIZE; n++)
  {
    float t = 2.0f * 3.14159265358979f * n / FFT_SIZE;
    fft_re[n] = A1 * sinf(f0_bin * t)
              + A2 * sinf(2 * f0_bin * t)
              + A3 * sinf(3 * f0_bin * t)
              + A4 * sinf(4 * f0_bin * t)
              + A5 * sinf(5 * f0_bin * t);
    fft_im[n] = 0.0f;
  }

  fft(fft_re, fft_im, FFT_SIZE);

  for (int k = 0; k < FFT_SIZE / 2; k++)
    fft_mag[k] = sqrtf(fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k]);

  int f0 = 2;
  for (int k = 3; k < FFT_SIZE / 2; k++)
    if (fft_mag[k] > fft_mag[f0]) f0 = k;

  float fund = fft_mag[f0];
  float harm_power = 0.0f;
  for (int h = 2; h <= 5; h++)
    harm_power += fft_mag[f0 * h] * fft_mag[f0 * h];
  float thd = sqrtf(harm_power) / fund;

  printf("=== FFT 自测（软件造正弦）===\r\n");
  printf("基波 bin=%d 频率=%dHz\r\n", f0, (int)(f0 * fs / FFT_SIZE));
  printf("幅度(基波~5次): %d %d %d %d %d\r\n",
         (int)fft_mag[f0], (int)fft_mag[f0*2], (int)fft_mag[f0*3],
         (int)fft_mag[f0*4], (int)fft_mag[f0*5]);
  int t = (int)(thd * 100000.0f + 0.5f);
  printf("THD = %d.%03d%%\r\n", t / 1000, t % 1000);
}

/* 真实测量：读 ADC 缓冲 �?? 去直�?? �?? FFT �?? 找基�?? �?? �?? THD（结果存全局变量�?? */
/* 核心：adc_buf -> 去直流 ->（可选加汉宁窗）-> FFT -> 找基波 -> 算 THD
 * use_window=1 时加汉宁窗，压低非整数 bin 造成的频谱泄漏。
 * 谐波幅值用"主瓣能量求和"：在峰值附近 +/-2 个 bin 内累加能量，
 * 这样基频/谐波落在格子中间时也不会少算。结果存 g_thd/g_fund_freq/g_harm */
/* 设置 ADC 采样率：Fs = 84MHz / ((PSC+1)(ARR+1))，反算 TIM3 分频。
 * 定时器时钟 = 168MHz(APB1分频≠1时×2)... 实际 APB1=PCLK1*2 = 42MHz*2 = 84MHz。
 * 84MHz 只能整数分频，实际 Fs 取最近可达值；返回实际采样率（频率换算必须用它）。*/
uint32_t set_sample_rate(uint32_t fs_target)
{
  const uint32_t TIM_CLK = 84000000u;         /* APB1 定时器时钟 */
  if (fs_target < 1300u)       fs_target = 1300u;     /* ARR 上限 65536 → 最低 Fs */
  if (fs_target > TIM_CLK / 2u) fs_target = TIM_CLK / 2u;

  uint32_t k = (TIM_CLK + fs_target / 2u) / fs_target; /* 四舍五入求分频数 */
  if (k < 2u) k = 2u;
  if (k > 65536u) k = 65536u;

  htim3.Instance->PSC = 0u;                   /* 分频全放 ARR，分辨率最高 */
  htim3.Instance->ARR = k - 1u;
  htim3.Init.Prescaler = 0u;
  htim3.Init.Period = k - 1u;

  g_sample_rate = TIM_CLK / k;                /* 实际采样率 */
  return g_sample_rate;
}

/* 整帧快照：ADC 只由 TIM3 TRGO 触发，暂停 TIM3 就等于暂停采样。
 * ⚠️ 环形 DMA 里存的是"最近 N 点"的循环版本：物理上的“新旧接缝”
 *    （最新点与最老点之间差了整整一圈）落在数组中间的随机位置。
 *    汉宁窗两端为 0，若接缝在数组首尾会被压掉；若在中间，这个相位跳变
 *    会被窗完整加权 → 主瓣展宽、±2bin 抬升、频率被拉偏（非整数周期/帧时）。
 * 所以：停表 → 等正在进行的转换完成 → 读 NDTR 得到下一写入位置 k（=最老点），
 *       从 k 开始环形拷出，把接缝放到数组首尾（窗零点处）。 */
static void adc_snapshot(void)
{
  __HAL_TIM_DISABLE(&htim3);

  /* 等最后一次已触发的转换完成（ADC 转换约 0.7us），之后 DMA 不再写入 */
  for (volatile int i = 0; i < 600; i++) { __NOP(); }

  uint32_t nd = hdma_adc1.Instance->NDTR;
  uint32_t k = (ADC_BUF_SIZE - nd) & (ADC_BUF_SIZE - 1);   /* 最老样点的下标 */

  for (int n = 0; n < ADC_BUF_SIZE; n++)
  {
    uint32_t idx = (uint32_t)n + k;
    if (idx >= ADC_BUF_SIZE) idx -= ADC_BUF_SIZE;
    adc_snap[n] = adc_buf[idx];
  }
  __HAL_TIM_ENABLE(&htim3);
}

/* 时域过零测频（诊断用）：不依赖 FFT/bin/窗口，直接数快照里向上过零的间隔。
 * 与 FFT 结果对照，可以区分“采样率/硬件问题”还是“频谱分析问题”。 */
static float estimate_freq_zerocross(void)
{
  float mean = 0.0f;
  for (int n = 0; n < ADC_BUF_SIZE; n++) mean += adc_snap[n];
  mean /= (float)ADC_BUF_SIZE;

  int count = 0;
  float first_x = 0.0f, last_x = 0.0f;
  for (int n = 1; n < ADC_BUF_SIZE; n++)
  {
    float a = (float)adc_snap[n - 1] - mean;
    float b = (float)adc_snap[n] - mean;
    if (a < 0.0f && b >= 0.0f)
    {
      float x = (float)(n - 1) + a / (a - b);   /* 线性插值过零点位置 */
      if (count == 0) first_x = x;
      last_x = x;
      count++;
    }
  }
  if (count < 2) return 0.0f;
  return (float)(count - 1) * (float)g_sample_rate / (last_x - first_x);
}

/* 只做“算频谱”这一步：adc_snap -> 去直流 ->（可选加汉宁窗）-> FFT -> fft_mag[]
 * 单独抽出来是为了多帧平均复用：每帧只算频谱、累加功率。 */
static void compute_spectrum(int use_window)
{
  const float TWO_PI = 6.28318530717959f;

  adc_snapshot();

  /* 去直流，并（可选）加汉宁窗 */
  float mean = 0.0f;
  for (int n = 0; n < FFT_SIZE; n++) mean += adc_snap[n];
  mean /= (float)FFT_SIZE;

  for (int n = 0; n < FFT_SIZE; n++)
  {
    float w = 1.0f;
    if (use_window)
      w = 0.5f * (1.0f - cosf(TWO_PI * (float)n / (float)FFT_SIZE));
    fft_re[n] = ((float)adc_snap[n] - mean) * w;
    fft_im[n] = 0.0f;
  }

  fft(fft_re, fft_im, FFT_SIZE);

  for (int k = 0; k < FFT_SIZE / 2; k++)
    fft_mag[k] = sqrtf(fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k]);
}

/* 只做“分析频谱”这一步：从 fft_mag[] 找基波、算归一化谐波幅值和 THD。
 * 结果存 g_thd/g_fund_freq/g_harm。多帧平均把功率平均好、开方回 fft_mag[] 后也调用它。 */
static void analyze_spectrum(int use_window)
{
  /* 找基波：低频段最大峰 */
  int f0 = 2;
  for (int k = 3; k < FFT_SIZE / 2; k++)
    if (fft_mag[k] > fft_mag[f0]) f0 = k;
  g_f0_bin = f0;

  /* 3) 修正基频频率：基频不落在整数 bin 上也能报准
   *    加汉宁窗时用汉宁窗专属的比值法（精确）：δ=(2R-1)/(R+1)，R=相邻格/峰值格
   *    不加窗时退回抛物线插值 */
  float f0_fine = (float)f0;
  if (f0 > 0 && f0 < FFT_SIZE / 2 - 1)
  {
    if (use_window)
    {
      float peak = fft_mag[f0];
      if (peak > 0.0f)
      {
        if (fft_mag[f0 + 1] >= fft_mag[f0 - 1])
        {
          float R = fft_mag[f0 + 1] / peak;
          f0_fine = (float)f0 + (2.0f * R - 1.0f) / (R + 1.0f);
        }
        else
        {
          float R = fft_mag[f0 - 1] / peak;
          f0_fine = (float)f0 - (2.0f * R - 1.0f) / (R + 1.0f);
        }
      }
    }
    else
    {
      float a = fft_mag[f0 - 1], b = fft_mag[f0], c = fft_mag[f0 + 1];
      float denom = a - 2.0f * b + c;
      if (denom != 0.0f) f0_fine = (float)f0 + 0.5f * (a - c) / denom;
    }
  }
  g_fund_freq = f0_fine * ((float)g_sample_rate / FFT_SIZE);
  g_harm[0] = 1.0f;

  /* 4) 主瓣能量求和（+/-2 bin）取各分量能量，再算归一化幅值与 THD */
  float p1 = 0.0f;
  for (int k = f0 - 2; k <= f0 + 2; k++)
    if (k > 0 && k < FFT_SIZE / 2) p1 += fft_mag[k] * fft_mag[k];

  float harm_power = 0.0f;
  for (int h = 2; h <= 5; h++)
  {
    int approx = (int)((float)f0 * h + 0.5f);
    int lo = approx - 2, hi = approx + 2;
    if (lo < 1) lo = 1;
    if (hi > FFT_SIZE / 2 - 1) hi = FFT_SIZE / 2 - 1;
    int pk = lo;
    for (int k = lo; k <= hi; k++) if (fft_mag[k] > fft_mag[pk]) pk = k;
    float ph = 0.0f;
    for (int k = pk - 2; k <= pk + 2; k++)
      if (k > 0 && k < FFT_SIZE / 2) ph += fft_mag[k] * fft_mag[k];
    g_harm[h - 1] = (p1 > 1.0f) ? sqrtf(ph / p1) : 0.0f;
    harm_power += ph;
  }

  g_thd = (p1 > 1.0f) ? sqrtf(harm_power / p1) : 0.0f;
}

/* 单帧测量：算频谱 + 分析（= 原 compute_thd 的行为，结果完全一致） */
float compute_thd(int use_window)
{
  compute_spectrum(use_window);
  analyze_spectrum(use_window);
  return g_thd;
}

/* 圆形 DMA 一直在后台采：等 >1 帧时间，保证 adc_buf 里全是"当前采样率"下的新数据 */
static void adc_wait_frame(void)
{
  uint32_t ms = (uint32_t)((FFT_SIZE * 1000u) / g_sample_rate) + 5u;
  HAL_Delay(ms);
}

/* 【多帧功率谱平均 Welch】采 frames 帧，累加每帧的功率谱 |X|²，平均后开方回 fft_mag[]，
 * 再走与单帧完全相同的 analyze_spectrum()。
 *   信号（基波+谐波）帧间相干 → 功率是常数，平均后原样保留；
 *   噪声帧间随机 → 平均后地板降 1/√frames（frames=16 → 降 4 倍）。
 * 平均是无偏的：大信号(600mV)读数不变，收益主要在小信号档(30mV)。
 * ⚠️ 必须平均“功率谱”：DMA 圆形缓冲帧间不同相，时域平均会把信号本身抵消掉。 */
float compute_thd_avg(int use_window, int frames)
{
  static float acc[FFT_SIZE / 2];      /* 功率累加器（2KB，放静态区，别占栈） */
  if (frames < 1) frames = 1;

  for (int k = 0; k < FFT_SIZE / 2; k++) acc[k] = 0.0f;
  for (int f = 0; f < frames; f++)
  {
    adc_wait_frame();                  /* 每帧都等一帧“新”数据 */
    compute_spectrum(use_window);
    for (int k = 0; k < FFT_SIZE / 2; k++)
      acc[k] += fft_mag[k] * fft_mag[k];
  }
  for (int k = 0; k < FFT_SIZE / 2; k++)
    fft_mag[k] = sqrtf(acc[k] / (float)frames);   /* 平均功率开方 → 等效“平均后的频谱” */

  analyze_spectrum(use_window);
  return g_thd;
}

/* 【Phase B 发挥核心】两遍自适应采样率 + 加窗，覆盖 500Hz~50kHz：
 *   pass1  用固定 Fs1=150kHz 粗测基频 f0（150k 奈奎斯特 75k > 50kHz，基波不混叠，
 *          且 500Hz 落在 bin3 以上，可被识别）——只为"知道 f0 大概多少"。
 *   选周期数 M：让 pass2 的 Fs2 = f0*N/M 落进 [~60k, 1.4M]，M 取整数（相干采样），
 *          并保证 5 次谐波不混叠（Fs2 ≥ 12·f0 → M ≤ N/12）且每周期 ≥12 点。
 *   pass2  用 Fs2 精测，基波/谐波正好落在整数 bin 上，泄漏最小；实际 TIM3 只能
 *          整数分频、且 f0 估计有误差，相干不完美 → 仍加汉宁窗 + 主瓣求和兜底。
 * 结果：g_thd / g_fund_freq / g_harm（与 compute_thd 同一套全局变量）。*/
void measure_thd_auto(void)
{
  /* ---- pass1：粗测 f0（多帧平均，小信号下 f0 也不会被噪声带偏）---- */
  set_sample_rate(150000u);
  compute_thd_avg(1, AVG_FRAMES);
  float f0 = g_fund_freq;
#if USE_DEBUG_PRINT
  {
    int fb = g_f0_bin;
    if (fb < 2) fb = 2;
    if (fb > FFT_SIZE / 2 - 3) fb = FFT_SIZE / 2 - 3;
    printf("[pass1] Fs=%lu f0=%dHz bin=%d zc=%dHz  m=%lu %lu %lu %lu %lu\r\n",
           (unsigned long)g_sample_rate,
           (int)(f0 + 0.5f), g_f0_bin,
           (int)(estimate_freq_zerocross() + 0.5f),
           (unsigned long)fft_mag[fb - 2], (unsigned long)fft_mag[fb - 1],
           (unsigned long)fft_mag[fb], (unsigned long)fft_mag[fb + 1],
           (unsigned long)fft_mag[fb + 2]);
  }
#endif
  if (f0 < 100.0f) f0 = 100.0f;          /* 保护：没测到基波时退回偏低值，M 会被夹到上限 */

  /* ---- 选整数周期数 M ---- */
  int M_min = (int)ceilf(f0 * (float)FFT_SIZE / 1400000.0f);  /* Fs2 ≤ 1.4MHz */
  if (M_min < 8) M_min = 8;              /* M≥8：基波与 2 次谐波至少隔 8 个 bin，主瓣不糊在一起 */
  int M_max = FFT_SIZE / 12;             /* =85：Fs2 ≥ 12·f0，5 次谐波(5f0)不混叠，每周期≥12点 */
  if (M_max < M_min) M_max = M_min;
  int M = (int)(f0 * (float)FFT_SIZE / 450000.0f + 0.5f);     /* 目标 Fs2≈450kHz（分辨率和余量都舒服） */
  if (M < M_min) M = M_min;
  if (M > M_max) M = M_max;

  uint32_t fs2 = (uint32_t)(f0 * (float)FFT_SIZE / (float)M + 0.5f);

  /* ---- pass2：精测（同样多帧功率谱平均，压低小信号噪声地板）---- */
  set_sample_rate(fs2);                  /* 写 TIM3；实际可达 Fs 存 g_sample_rate */
#if USE_DEBUG_PRINT
  printf("[pass2] M=%d fs_target=%lu fs=%lu\r\n",
         M, (unsigned long)fs2, (unsigned long)g_sample_rate);
#endif
  compute_thd_avg(1, AVG_FRAMES);        /* g_thd/g_fund_freq/g_harm 就绪 */
}

/* 【调试】临时读一次芯片内部 VREFINT 通道，判断 ADC 内核 + 参考电压(VDDA) 是否正常。
 * 完全不用接任何外部线；健康时约 1.21V -> 约 1500 个码字。
 * 用完把 ADC 恢复成 PA1 通道 + 上升沿外部触发。 */
uint32_t adc_read_vrefint_raw(void)
{
  ADC_ChannelConfTypeDef cfg = {0};
  uint32_t val = 0xFFFFFFFFu;

  HAL_ADC_Stop(&hadc1);

  /* 打开内部基准通路，等它上电稳定，再临时改成软件触发 */
  ADC->CCR |= ADC_CCR_TSVREFE;
  HAL_Delay(20u);
  hadc1.Instance->CR2 &= ~ADC_CR2_EXTEN;

  cfg.Channel = ADC_CHANNEL_VREFINT;
  cfg.Rank = 1;
  cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) == HAL_OK)
  {
    for (int i = 0; i < 3; i++)   /* 连读 3 次，丢掉前几次预热值 */
    {
      if (HAL_ADC_Start(&hadc1) == HAL_OK)
      {
        if (HAL_ADC_PollForConversion(&hadc1, 20u) == HAL_OK)
          val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
      }
    }
  }

  /* 恢复：PA1 通道 + 上升沿外部触发 */
  cfg.Channel = ADC_CHANNEL_1;
  cfg.Rank = 1;
  cfg.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &cfg);
  hadc1.Instance->CR2 |= ADC_CR2_EXTEN_0;

  return val;
}

/* 链路自测：软件�??"基波 + 2~5次谐�?"�? adc_buf，走 measure_thd()�?
 * 验证 FFT->THD->显示 整条链（不经过真�? ADC 硬件）�??
 * 期望：THD �? 6.245%，基�? �? 976.6Hz（bin=10�? */
void run_adc_chain_self_test(void)
{
  const float A1 = 400.0f;
  const float f0b = 10.24f;   /* 1000Hz：故意落在非整数 bin，模拟真机 1kHz 的频谱泄漏 */
  const float h2 = 0.05f, h3 = 0.03f, h4 = 0.02f, h5 = 0.01f;
  for (int n = 0; n < ADC_BUF_SIZE; n++)
  {
    float t = 2.0f * 3.14159265358979f * n / ADC_BUF_SIZE;
    float v = 2048.0f
            + A1 * sinf(f0b * t)
            + h2 * A1 * sinf(2.0f * f0b * t)
            + h3 * A1 * sinf(3.0f * f0b * t)
            + h4 * A1 * sinf(4.0f * f0b * t)
            + h5 * A1 * sinf(5.0f * f0b * t);
    if (v < 0.0f) v = 0.0f;
    if (v > 4095.0f) v = 4095.0f;
    adc_buf[n] = (uint16_t)(v + 0.5f);
  }
}

/* 【Phase B 自测】纯软件验证"两遍自适应采样率"逻辑（不接硬件）：
 * 对一组真值频率，按当前 g_sample_rate 合成"基波+2~5次谐波"填 adc_buf，
 * 跑完整 pass1→选 M→pass2 流程，打印测得的 f0 与 THD（理论 THD=6.245%）。
 * 用它验证：M 选择、TIM3 整数分频后的相干残差、加窗兜底是否达标。
 * 注意：必须先 set_sample_rate() 再 synth()，否则合成用的是旧采样率。 */
static void synth_signal(float f_true, float amp)
{
  const float h2 = 0.05f, h3 = 0.03f, h4 = 0.02f, h5 = 0.01f;
  for (int n = 0; n < ADC_BUF_SIZE; n++)
  {
    float ph = 6.28318530717959f * f_true * (float)n / (float)g_sample_rate;
    float v = 2048.0f + amp * (sinf(ph)
             + h2 * sinf(2.0f * ph) + h3 * sinf(3.0f * ph)
             + h4 * sinf(4.0f * ph) + h5 * sinf(5.0f * ph));
    if (v < 0.0f) v = 0.0f;
    if (v > 4095.0f) v = 4095.0f;
    adc_buf[n] = (uint16_t)(v + 0.5f);
  }
}

void run_auto_selftest(void)
{
  const float freqs[] = { 500.0f, 1000.0f, 5000.0f, 10000.0f, 50000.0f };
  const int NF = (int)(sizeof(freqs) / sizeof(freqs[0]));
  printf("=== Phase B 自适应采样率自测（理论 THD=6.245%%）===\r\n");
  for (int i = 0; i < NF; i++)
  {
    float f = freqs[i];
    /* pass1 */
    set_sample_rate(150000u);
    synth_signal(f, 400.0f);
    compute_thd(1);
    float f0_est = g_fund_freq;
    /* 选 M（与 measure_thd_auto 同一套规则） */
    int M_min = (int)ceilf(f0_est * (float)FFT_SIZE / 1400000.0f);
    if (M_min < 8) M_min = 8;
    int M_max = FFT_SIZE / 12;
    if (M_max < M_min) M_max = M_min;
    int M = (int)(f0_est * (float)FFT_SIZE / 450000.0f + 0.5f);
    if (M < M_min) M = M_min;
    if (M > M_max) M = M_max;
    uint32_t fs2 = (uint32_t)(f0_est * (float)FFT_SIZE / (float)M + 0.5f);
    /* pass2 */
    set_sample_rate(fs2);
    synth_signal(f, 400.0f);
    float thd = compute_thd(1) * 100.0f;
    printf("f=%5dHz: pass1 f0=%5d M=%2d Fs2=%6lu -> f0=%5dHz THD=%d.%03d%%\r\n",
           (int)f, (int)f0_est, M, (unsigned long)g_sample_rate,
           (int)g_fund_freq, (int)thd, (int)((thd - (float)(int)thd) * 1000.0f + 0.5f));
  }
  set_sample_rate(SAMPLE_RATE);   /* 复位默认采样率 */
}

/* ==================== 波形还原：显示 1 个周期 ==================== */

/* 画归一化谐波幅值：3 位小数、去掉前导 0（0.050 -> .050），省显示宽度 */
static void oled_draw_harm(uint8_t x, uint8_t page, float v)
{
  char buf[16];
  oled_ftoa(v, 3, buf);
  if (v < 1.0f && buf[0] == '0')
    oled_draw_string(x, page, &buf[1]);
  else
    oled_draw_string(x, page, buf);
}

/* 在线性插值位置 idx 处取 adc_snap 的值（已去直流 mean） */
static float adc_sample_at(float idx, float mean)
{
  int i0 = (int)idx;
  float frac = idx - (float)i0;
  if (i0 < 0) { i0 = 0; frac = 0.0f; }
  if (i0 >= ADC_BUF_SIZE - 1) { i0 = ADC_BUF_SIZE - 2; frac = 1.0f; }
  float a = (float)adc_snap[i0] - mean;
  float b = (float)adc_snap[i0 + 1] - mean;
  return a + (b - a) * frac;
}

/* 在 OLED 底部画 1 个周期的波形（128 列宽 x 32 像素高） */
void draw_waveform(void)
{
  if (g_fund_freq < 1.0f) return;

  /* 求均值做去直流（用快照，和频谱是同一帧） */
  float mean = 0.0f;
  for (int n = 0; n < ADC_BUF_SIZE; n++) mean += adc_snap[n];
  mean /= (float)ADC_BUF_SIZE;

  /* 触发点：第一个"从负变正"穿过中线的采样点，作为周期起点（类似示波器触发） */
  int start = 0;
  for (int n = 1; n < ADC_BUF_SIZE; n++)
  {
    if (((float)adc_snap[n] - mean) >= 0.0f && ((float)adc_snap[n - 1] - mean) < 0.0f)
    { start = n; break; }
  }

  /* 一个周期对应多少采样点 */
  float spp = (float)g_sample_rate / g_fund_freq;

  /* 算出 128 列的值，并统计峰峰用于自适应纵向缩放 */
  static float col[WAVE_X];
  float vmin = 1e9f, vmax = -1e9f;
  for (int x = 0; x < WAVE_X; x++)
  {
    float idx = (float)start + (float)x * spp / (float)WAVE_X;
    float v = adc_sample_at(idx, mean);
    col[x] = v;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }
  float vpp = vmax - vmin;
  if (vpp < 1.0f) vpp = 1.0f;

  /* 逐列打点：把 vmin..vmax 映射到波形区底部..顶部 */
  const int y_bot = WAVE_Y0 + WAVE_H - 1;
  for (int x = 0; x < WAVE_X; x++)
  {
    int y = y_bot - (int)((col[x] - vmin) / vpp * (float)(WAVE_H - 1) + 0.5f);
    if (y < WAVE_Y0) y = WAVE_Y0;
    if (y > y_bot) y = y_bot;
    oled_buf[y >> 3][x] |= (uint8_t)(1u << (y & 7));
  }
}

/* Send THD result to phone over Bluetooth (USART2 / JDY-31) */
void bt_send_result(void)
{
  char line[128];
  int thd_p2 = (int)(g_thd * 10000.0f + 0.5f);   /* THD% x 100 */
  int h2 = (int)(g_harm[1] * 1000.0f + 0.5f);    /* x 1000 */
  int h3 = (int)(g_harm[2] * 1000.0f + 0.5f);
  int h4 = (int)(g_harm[3] * 1000.0f + 0.5f);
  int h5 = (int)(g_harm[4] * 1000.0f + 0.5f);
  int len = snprintf(line, sizeof(line),
                     "THD=%d.%02d%% f0=%dHz\r\n"
                     "H1=1.000 H2=%d.%03d H3=%d.%03d H4=%d.%03d H5=%d.%03d\r\n",
                     thd_p2 / 100, thd_p2 % 100, (int)g_fund_freq,
                     h2 / 1000, h2 % 1000, h3 / 1000, h3 % 1000,
                     h4 / 1000, h4 % 1000, h5 / 1000, h5 % 1000);
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
}

/* bt_mcu App 数据包（固定 7 个 float）：
 *   A5 | float0..float6（每个 4 字节，小端） | 校验和 | 5A
 *   校验和 = 全部数据字节之和 & 0xFF。
 * 字段顺序：0=波形  1=THD(%)  2=f0(Hz)  3=H2  4=H3  5=H4  6=H5
 * App 端把“接收数据包”配成 7*float（共 31 字节），
 * 波形控件链接 FLOAT0，文本框分别链接 FLOAT1~FLOAT6。 */
static void bt_send_measure_packet(float wave)
{
  float vals[7];
  vals[0] = wave;
  vals[1] = g_thd * 100.0f;
  vals[2] = g_fund_freq;
  vals[3] = g_harm[1];
  vals[4] = g_harm[2];
  vals[5] = g_harm[3];
  vals[6] = g_harm[4];

  uint8_t pkt[3 + 7 * 4];
  uint8_t sum = 0;
  int p = 0;
  pkt[p++] = 0xA5;
  for (int i = 0; i < 7; i++)
  {
    union { float f; uint32_t u; } cv;
    cv.f = vals[i];
    for (int b = 0; b < 4; b++)
    {
      uint8_t byte = (uint8_t)((cv.u >> (8 * b)) & 0xFFu);
      pkt[p++] = byte;
      sum += byte;
    }
  }
  pkt[p++] = sum;
  pkt[p++] = 0x5A;
  HAL_UART_Transmit(&huart2, pkt, (uint16_t)p, HAL_MAX_DELAY);
}

/* 蓝牙发送一周期波形。
 * 取点方式与 OLED 波形一致：从中线由负变正的触发点开始，等间隔取整整一个周期。
 * BT_WAVE_FORMAT=1：每点一个 App 数据包（波形 + 6 个测量结果），
 *   波形值归一化到 -950~+950（App 默认纵轴 -1000~+1000，正好铺满）；
 * BT_WAVE_FORMAT=0：旧 ASCII 帧 W,v0,...,v63\r\n。 */
void bt_send_wave(void)
{
  if (g_fund_freq < 1.0f) return;

  float mean = 0.0f;
  for (int n = 0; n < ADC_BUF_SIZE; n++) mean += adc_snap[n];
  mean /= (float)ADC_BUF_SIZE;

  int start = 0;
  for (int n = 1; n < ADC_BUF_SIZE; n++)
  {
    if (((float)adc_snap[n] - mean) >= 0.0f && ((float)adc_snap[n - 1] - mean) < 0.0f)
    { start = n; break; }
  }

  float spp = (float)g_sample_rate / g_fund_freq;   /* 每周期采样点数 */

#if BT_WAVE_FORMAT
  static float col[BT_WAVE_POINTS];
  float vmin = 1e9f, vmax = -1e9f;
  for (int x = 0; x < BT_WAVE_POINTS; x++)
  {
    float idx = (float)start + (float)x * spp / (float)BT_WAVE_POINTS;
    col[x] = adc_sample_at(idx, mean);
    if (col[x] < vmin) vmin = col[x];
    if (col[x] > vmax) vmax = col[x];
  }
  float span = vmax - vmin;
  if (span < 1.0f) span = 1.0f;
  for (int x = 0; x < BT_WAVE_POINTS; x++)
    bt_send_measure_packet(-950.0f + 1900.0f * (col[x] - vmin) / span);
#else
  static char line[420];
  int pos = snprintf(line, sizeof(line), "W");
  for (int x = 0; x < 64; x++)
  {
    if (pos >= (int)sizeof(line) - 10) break;
    float idx = (float)start + (float)x * spp / 64.0f;
    int v = (int)(adc_sample_at(idx, mean) + 0.5f);
    pos += snprintf(line + pos, sizeof(line) - (size_t)pos, ",%d", v);
  }
  pos += snprintf(line + pos, sizeof(line) - (size_t)pos, "\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)pos, HAL_MAX_DELAY);
#endif
}

/* 【Phase E】一次完整测量：采集 → THD → 串口 → OLED → 蓝牙。返回耗时(ms)。*/
uint32_t do_one_measurement(void)
{
  uint32_t t0 = HAL_GetTick();

#if USE_ADC_SELF_TEST
  run_adc_chain_self_test();
  compute_thd(0);            /* 不加窗（对比用），结果被下一行覆盖 */
  compute_thd(1);            /* 加汉宁窗（这个结果留全局，供显示/蓝牙） */
#else
  if (g_advanced_mode)
  {
    measure_thd_auto();      /* 发挥部分：500Hz~50kHz，两遍自适应采样率精测 */
  }
  else
  {
    /* 基础部分：1kHz 固定基频。每次测量前重设相干采样率——
     * 从发挥模式切回来时，采样率还停在自适应值上，必须恢复。 */
    set_sample_rate((1000u * FFT_SIZE) / 10u);   /* 目标 102.4kHz，实际 102439Hz */
    compute_thd_avg(USE_HANN_WINDOW, AVG_FRAMES);
  }
#if USE_DEBUG_PRINT
  /* 调试：打印 adc_buf 原始读数统计，验证真实 ADC 采集通路（接线/削顶判断用） */
  uint16_t raw_min = 4095U, raw_max = 0U;
  uint32_t raw_sum = 0U;
  for (int n = 0; n < ADC_BUF_SIZE; n++)
  {
    uint16_t v = adc_snap[n];
    if (v < raw_min) raw_min = v;
    if (v > raw_max) raw_max = v;
    raw_sum += v;
  }
  printf("ADC raw: min=%u max=%u mean=%lu | adc_state=%d dma_state=%d dma_err=%lu NDTR=%lu\r\n",
         raw_min, raw_max, (unsigned long)(raw_sum / ADC_BUF_SIZE),
         (int)hadc1.State, (int)hdma_adc1.State, (unsigned long)hdma_adc1.ErrorCode,
         (unsigned long)hdma_adc1.Instance->NDTR);
  /* 诊断：看基波到底落在哪个 bin、邻近 bin 的幅度（判断泄漏/峰是否抖） */
  int fb = g_f0_bin;
  if (fb < 2) fb = 2;
  if (fb > FFT_SIZE / 2 - 3) fb = FFT_SIZE / 2 - 3;
  printf("dbg: Fs=%lu f0bin=%d zc=%dHz  m[%d-2,..,+2]=%lu %lu %lu %lu %lu\r\n",
         (unsigned long)g_sample_rate, g_f0_bin,
         (int)(estimate_freq_zerocross() + 0.5f), g_f0_bin,
         (unsigned long)fft_mag[fb - 2], (unsigned long)fft_mag[fb - 1],
         (unsigned long)fft_mag[fb], (unsigned long)fft_mag[fb + 1],
         (unsigned long)fft_mag[fb + 2]);
#endif
#endif

  int thd_scaled = (int)(g_thd * 100000.0f + 0.5f);   /* 百分比 ×1000 */
  int h2 = (int)(g_harm[1] * 1000.0f + 0.5f);         /* 归一化幅值 ×1000（printf 不支持 %f） */
  int h3 = (int)(g_harm[2] * 1000.0f + 0.5f);
  int h4 = (int)(g_harm[3] * 1000.0f + 0.5f);
  int h5 = (int)(g_harm[4] * 1000.0f + 0.5f);
  printf("THD = %d.%03d%%  f0 = %d Hz  H2=%d.%03d H3=%d.%03d H4=%d.%03d H5=%d.%03d\r\n",
         thd_scaled / 1000, thd_scaled % 1000, (int)g_fund_freq,
         h2 / 1000, h2 % 1000, h3 / 1000, h3 % 1000,
         h4 / 1000, h4 % 1000, h5 / 1000, h5 % 1000);

  /* OLED 显示：上 4 行文字 + 底部 1 个周期波形 */
  char line[21];
  oled_clear();

  oled_draw_string(0, 0, "THD=");
  oled_ftoa(g_thd * 100.0f, 2, line);
  oled_draw_string(32, 0, line);
  oled_draw_string(80, 0, "%");
  oled_draw_string(100, 0, g_advanced_mode ? "ADV" : "BAS");   /* 当前模式 */

  oled_draw_string(0, 1, "f0=");
  oled_ftoa(g_fund_freq, 0, line);
  oled_draw_string(24, 1, line);
  oled_draw_string(56, 1, "Hz");
  oled_draw_string(72, 1, "H1=");
  oled_draw_harm(90, 1, 1.0f);    /* 基波归一化幅值：按定义为 1.000 */

  oled_draw_string(0, 2, "H2=");
  oled_draw_harm(24, 2, g_harm[1]);
  oled_draw_string(64, 2, "H3=");
  oled_draw_harm(88, 2, g_harm[2]);

  oled_draw_string(0, 3, "H4=");
  oled_draw_harm(24, 3, g_harm[3]);
  oled_draw_string(64, 3, "H5=");
  oled_draw_harm(88, 3, g_harm[4]);

  draw_waveform();           /* 底部：1 个周期波形 */
  oled_update();
  bt_send_result();
#if BT_SEND_WAVE
  bt_send_wave();            /* 发挥部分：手机端显示一周期波形 */
#endif

  uint32_t elapsed = HAL_GetTick() - t0;
  printf("[one-key] elapsed %lu ms (target <3000)\r\n", (unsigned long)elapsed);
  return elapsed;
}

/* 【Phase E】等触发：
 *   短按 PA0（<0.8s）或串口1收到任意字符 → 返回 1，调用方测一次；
 *   长按 PA0（≥0.8s）           → 切换 基础/发挥 模式，返回 0，不测量。 */
int wait_for_trigger(void)
{
  for (;;)
  {
    /* 串口触发：没有按键时，用串口助手发任意字符即可触发一次测量 */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
      volatile uint8_t c = (uint8_t)(huart1.Instance->DR & 0xFFu);  /* 读 DR 清标志 */
      (void)c;
      HAL_Delay(50u);
      return 1;
    }
    /* 按键触发：按下=BTN_PRESSED，去抖后开始计时 */
    if (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == BTN_PRESSED)
    {
      HAL_Delay(20u);
      if (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == BTN_PRESSED)
      {
        int long_press = 0;
        uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == BTN_PRESSED)
        {
          if (HAL_GetTick() - t0 >= LONG_PRESS_MS) { long_press = 1; break; }
          HAL_Delay(10u);
        }
        while (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == BTN_PRESSED) { HAL_Delay(10u); }  /* 等松开 */

        if (long_press)
        {
          g_advanced_mode = !g_advanced_mode;
          printf("[mode] -> %s\r\n", g_advanced_mode ? "advanced" : "basic");
          /* 字库是"或"写入，先清掉这 24 列，避免 ADV/BAS 叠在一起 */
          for (uint8_t x = 100; x < 124; x++) oled_buf[0][x] = 0;
          oled_draw_string(100, 0, g_advanced_mode ? "ADV" : "BAS");
          oled_update();
          return 0;
        }
        return 1;   /* 短按：测一次 */
      }
    }
    HAL_Delay(10u);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("=== THD FW v4 (mode=%s) ===\r\n",
         USE_ADVANCED_MODE ? "advanced" : "basic");
#if USE_DEBUG_PRINT
  /* 【Phase A 自检】验证采样率反算：实际值应≈目标值 */
  printf("Fs test: 100k->%lu  500k->%lu  1.4M->%lu Hz\r\n",
         (unsigned long)set_sample_rate(100000u),
         (unsigned long)set_sample_rate(500000u),
         (unsigned long)set_sample_rate(1400000u));
#endif
#if USE_ADVANCED_MODE
  set_sample_rate(SAMPLE_RATE);   /* 发挥部分：默认 100kHz，实际由自适应测量改写 */
#else
  /* 基础部分：1kHz 固定 → 用【相干采样率】把 1kHz 精确放到整数 bin 上。
   * 目标 Fs = 1000 * N / M（M=10 个周期）≈ 102.4kHz，1kHz 正好落在 bin 10，
   * 2~5 次谐波正好落在 bin 20/30/40/50 → 泄漏≈0，f0 不再抖、THD 稳定。
   * （84MHz 只能整数分频，实际 Fs=84e6/820≈102439Hz，1kHz≈bin 9.992，足够相干。） */
  printf("basic Fs=%lu Hz (coherent for 1kHz)\r\n",
         (unsigned long)set_sample_rate((1000u * FFT_SIZE) / 10u));
#endif

  oled_init();
  HAL_UART_Transmit(&huart2, (uint8_t *)"THD Meter ready\r\n",
                    (uint16_t)(sizeof("THD Meter ready\r\n") - 1), HAL_MAX_DELAY);

#if USE_ADC_SELF_TEST
  run_auto_selftest();            /* Phase B 自测：跑一次，验证自适应采样率 */
#else
#if USE_VREFINT_SELFTEST
  /* 调试：读一次芯片内部 VREFINT，不接任何外部线，单独验证 ADC 内核 + 参考电压。
   * 健康值约 1500（=1.21V/3.3V*4095）。
   * ⚠️ adc_read_vrefint_raw() 内部会 HAL_ADC_Stop()，实测会干扰后面的 DMA 采集
   *    （导致 adc_buf 恒 0），平时务必保持 USE_VREFINT_SELFTEST=0。 */
  printf("VREFINT raw=%lu (健康约 1500)\r\n",
         (unsigned long)adc_read_vrefint_raw());
#endif
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_SIZE);
  HAL_TIM_Base_Start(&htim3);
#endif

#if USE_CONTINUOUS
  printf("连续测量模式\r\n");
#else
  printf("ONE-KEY: short press PA0 (or serial char) = measure; long press 0.8s = toggle mode\r\n");
  do_one_measurement();     /* 上电先测一次，屏幕立即有显示 */
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if USE_CONTINUOUS
    do_one_measurement();     /* 连续测量：台上调试方便 */
    HAL_Delay(1000U);
#else
    if (wait_for_trigger())
      do_one_measurement();   /* 短按/串口触发测量；长按只切模式，不测 */
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;   /* 3周期 @21MHz → 最高约 1.4Msps */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Phase E: 一键启动按键（PA0），输入 + 内部下拉（按下=高电平） */
  GPIO_InitTypeDef btn = {0};
  btn.Pin = BTN_PIN;
  btn.Mode = GPIO_MODE_INPUT;
  btn.Pull = GPIO_PULLDOWN;
  btn.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN_PORT, &btn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
