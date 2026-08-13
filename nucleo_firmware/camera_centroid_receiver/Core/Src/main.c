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
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  AXIS_X = 0,
  AXIS_Y = 1
} fta_axis_t;

/* CLOSED_LOOP: single-axis (dac_y -> cx) P+I position control -- the
 * architecture was decided 2026-08-04 (CLAUDE.md v2) but left
 * unimplemented until the optics were locked down and dac_y->cx
 * confirmed as by far the cleanest DAC->pixel pairing (2026-08-12, same
 * file). Derivative action and a second controlled axis are both
 * deliberately deferred -- see run_closed_loop_step's docstring below. */
typedef enum
{
  MODE_OPEN_LOOP   = 0,
  MODE_CLOSED_LOOP = 1
} fta_mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Wire format sent by nucleo_i2c_sender.py (rpi_camera_system repo):
 *   [0] reg pointer   (always 0x00, no real register file on this side --
 *                       kept only for convention parity with the OV9281
 *                       sensor's own register-pointer protocol on the Pi)
 *   [1] seq           (u8, wraps 0-255)
 *   [2] status        (u8, bit0 = beam confidently detected this cycle)
 *   [3:4] x           (s16, little-endian, real pixel value * POSITION_SCALE)
 *   [5:6] y           (s16, little-endian, real pixel value * POSITION_SCALE)
 *   [7] checksum      (u8, additive sum of bytes [1:6], mod 256)
 * Total 8 bytes, single write transaction, no repeated start / read-back. */
#define BEAM_PKT_LEN 8U

/* Must match POSITION_SCALE in NucleoLink.send_position
 * (nucleo_i2c_sender.py, rpi_camera_system repo): the Pi scales real
 * (sub-pixel, float) x/y by this before packing into the wire's s16
 * fields, to preserve one decimal digit of centroid precision without
 * growing the packet or putting a float on the wire. Values received here
 * are still in these scaled units -- divide by POSITION_SCALE to recover
 * real pixels. */
#define POSITION_SCALE 10

/* DAC setpoint clamp, matches the "FTA Controller" firmware's own default
 * safety clamp (rpi_camera_system CLAUDE.md, FTA architecture decision) --
 * kept identical so fta_calibration.py / fta_step_response_test.py behave
 * the same against this firmware's open_loop set_x/set_y as they did
 * against "FTA Controller". */
#define DAC_MIN_COUNT 95
#define DAC_MAX_COUNT 4000

/* Max VCP command line length (laptop -> Nucleo), including the
 * terminating NUL this buffer adds. Generous for the longest command
 * currently defined (set_x/set_y with a 4-digit value). */
#define VCP_LINE_BUF_LEN 96U

/* Closed-loop staleness watchdog threshold -- generous vs. the observed
 * ~0-25ms telemetry age even at the slower full-frame capture mode (see
 * rpi_camera_system CLAUDE.md, 2026-08-12 sine-check section), so this
 * only trips on genuine stream loss (Pi crashed, cable unplugged, etc.),
 * not normal jitter. Checked once per heartbeat tick (1Hz) rather than
 * only when a new packet arrives, so a fully-dead stream (no more
 * g_new_packet_ready events at all) still gets caught within ~1s instead
 * of silently freezing run_closed_loop_step with no active warning. */
#define STALE_TELEMETRY_MS 200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Raw bytes for the reception currently armed on the I2C peripheral --
 * only touched by the I2C1 ISR (via the HAL callbacks below) until a
 * transfer completes, so it's safe without extra locking here. */
static uint8_t i2c_rx_buf[BEAM_PKT_LEN];

/* Latest successfully-checksummed packet. Written only inside
 * HAL_I2C_SlaveRxCpltCallback (ISR context); read from main-loop code
 * later on. Not a multi-byte atomic snapshot -- if that matters once a
 * real consumer reads this mid-update, revisit with a double-buffer or
 * a copy-with-IRQ-disabled pattern. Fine for a first bring-up/smoke test.
 * x/y are stored exactly as received -- i.e. still scaled by
 * POSITION_SCALE, not real pixel units. Any future consumer (not just the
 * debug print below) must divide by POSITION_SCALE before using them. */
static volatile struct
{
  uint8_t  seq;
  uint8_t  status;
  int16_t  x;
  int16_t  y;
} g_latest_beam;

static volatile uint32_t g_packet_count = 0;          /* valid packets received */
static volatile uint32_t g_checksum_error_count = 0;   /* corrupt packets dropped */

/* HAL_GetTick() at the last valid packet -- used by get_status/heartbeat to
 * report telemetry age. Written only inside process_beam_packet (ISR
 * context), same lifetime/locking rationale as g_latest_beam above. */
static volatile uint32_t g_latest_beam_tick = 0;

/* Set (in ISR context) whenever a new valid packet lands in g_latest_beam;
 * the main loop polls and clears this rather than the UART print happening
 * directly in the I2C callback -- keeps the ISR short and avoids calling a
 * blocking HAL_UART_Transmit from interrupt context. */
static volatile uint8_t g_new_packet_ready = 0;

/* --- Phase-1 (no PID yet) FTA control state ---------------------------- */

/* DAC1 handle -- PA4 (DAC1_OUT1, x-axis), PA5 (DAC1_OUT2, y-axis). Both
 * pins are free of the old SB16/SB18 solder-bridge coupling to I2C1 now
 * that those bridges were removed (see rpi_camera_system CLAUDE.md,
 * amp-board I2C fault thread) -- no conflict with I2C1 SCL/SDA on D5/D4. */
static DAC_HandleTypeDef hdac1;

static volatile fta_mode_t g_mode = MODE_OPEN_LOOP;
static volatile uint8_t    g_amp_enabled   = 0;
static volatile uint8_t    g_estop_latched = 0;
static volatile int32_t    g_last_dac_x = DAC_MIN_COUNT;
static volatile int32_t    g_last_dac_y = DAC_MIN_COUNT;

/* --- Closed-loop (dac_y -> cx) control state -- only ever touched from
 * main-loop context (command processing and run_closed_loop_step, both
 * called from the main while(1) loop, never from ISR context), so unlike
 * g_latest_beam/g_last_dac_x/y above these don't need `volatile` for
 * cross-context visibility. Kp/Ki are taken over the VCP as milli-units
 * integers (strtol, no float parsing) and converted to float once here --
 * matches decode_scaled's existing avoidance of pulling in newlib's
 * float-formatting support for the small dedicated purpose of *display*,
 * while still doing the actual control arithmetic in float (cheap, this
 * MCU has a hardware FPU, see -mfpu=fpv4-sp-d16 in the build). */
static int32_t g_target_x_scaled = 0;   /* pixel setpoint for cx, POSITION_SCALE-scaled */
static uint8_t g_target_x_set    = 0;   /* set_mode closed_loop refuses to engage until this is 1 */
static int32_t g_kp_milli = 0;          /* Kp = g_kp_milli/1000, DAC counts per pixel of error */
static int32_t g_ki_milli = 0;          /* Ki = g_ki_milli/1000, DAC counts per (pixel*second) */
static float   g_kp = 0.0f;
static float   g_ki = 0.0f;
static float   g_integral_px_s = 0.0f;  /* running integral of error_px*dt_s, anti-windup clamped */
static int32_t g_closed_loop_base_dac_y = DAC_MIN_COUNT;  /* bumpless-transfer bias, see cmd_set_mode */
static uint32_t g_last_control_tick = 0;

/* Single-byte interrupt-driven VCP (USART2) receive, re-armed on every
 * completion/error -- same one-shot-then-rearm pattern as the I2C
 * reception below, just at the byte level instead of the packet level
 * since VCP commands are variable-length ASCII lines, not a fixed frame. */
static uint8_t   vcp_rx_byte;
static char      vcp_line_buf[VCP_LINE_BUF_LEN];
static volatile uint16_t vcp_line_len   = 0;
static volatile uint8_t  vcp_line_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static void process_beam_packet(const uint8_t *buf);
static void decode_scaled(int32_t scaled, const char **sign, int *whole, int *frac);

static void MX_DAC1_Init(void);
static void apply_dac(fta_axis_t axis, int32_t value);
static void amp_enable(void);
static void amp_disable(void);
static void estop(void);

static void send_line(const char *s);
static void process_command_line(char *line);
static void cmd_set_mode(const char *arg);
static void cmd_set_axis(fta_axis_t axis, const char *arg);
static void cmd_amp_enable(void);
static void cmd_amp_disable(void);
static void cmd_clear_estop(void);
static void cmd_get_status(void);
static void cmd_set_target_x(const char *arg);
static void cmd_set_kp(const char *arg);
static void cmd_set_ki(const char *arg);
static void run_closed_loop_step(int16_t tel_x_scaled, uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* MX_DAC1_Init() is hand-added (see its definition below) rather than
   * CubeMX-generated -- DAC1/PA4/PA5/PA12 and the USART2 RX interrupt were
   * never added to this project's .ioc (same "not board-file-configured"
   * tradeoff already made for the LED GPIO in MX_GPIO_Init above). Placing
   * everything for this inside USER CODE markers means a future CubeMX
   * regeneration won't delete any of it, but note it *will* stomp on
   * HAL_DAC_MODULE_ENABLED in stm32l4xx_hal_conf.h (that file isn't
   * USER-CODE-protected) -- re-enable it by hand if the project is ever
   * regenerated from the .ioc. */
  MX_DAC1_Init();

  /* Arm reception of the first 8-byte packet. This is a one-shot "receive
   * exactly BEAM_PKT_LEN bytes once addressed" request, not a persistent
   * listen mode -- it must be re-armed after every completion or error,
   * which both callbacks below do. */
  HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_buf, BEAM_PKT_LEN);

  /* Same one-shot/re-arm pattern for the VCP command link, at the single-
   * byte granularity described near vcp_rx_byte's declaration above. */
  HAL_UART_Receive_IT(&huart2, &vcp_rx_byte, 1);

  /* Safe boot default: amp stays disabled (g_amp_enabled=0, GPIOA12 low
   * from MX_GPIO_Init) until a VCP amp_enable command arrives. */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Free-running heartbeat, completely independent of whether any I2C
     * packet has ever arrived -- proves the firmware is alive and the
     * USART2/VCP link itself works, so that question can be answered
     * before touching anything I2C-related. */
    {
      static uint32_t last_heartbeat_tick = 0;
      uint32_t now = HAL_GetTick();

      if ((now - last_heartbeat_tick) >= 1000U)
      {
        /* Closed-loop staleness watchdog -- see STALE_TELEMETRY_MS's
         * comment for why this lives here (1Hz, independent of whether
         * new packets are even still arriving) rather than only being
         * checked inside run_closed_loop_step. Trips a full estop, not
         * just a hold -- driving further open-loop DAC commands on stale
         * position data is the one failure mode here with no safe
         * automatic default, so stop and make a human look at it. */
        if (g_mode == MODE_CLOSED_LOOP && g_packet_count > 0U
            && (now - g_latest_beam_tick) > STALE_TELEMETRY_MS)
        {
          estop();
          send_line("WARN closed-loop estop: telemetry stale\r\n");
        }

        char hb_line[96];
        int  hb_len = snprintf(hb_line, sizeof(hb_line),
                                "heartbeat uptime=%lus mode=%s amp=%u estop=%u pkts=%lu errs=%lu\r\n",
                                (unsigned long)(now / 1000U),
                                (g_mode == MODE_OPEN_LOOP) ? "open_loop" : "closed_loop",
                                (unsigned)g_amp_enabled, (unsigned)g_estop_latched,
                                (unsigned long)g_packet_count,
                                (unsigned long)g_checksum_error_count);
        if (hb_len > 0)
        {
          HAL_UART_Transmit(&huart2, (uint8_t *)hb_line, (uint16_t)hb_len, 100);
        }
        last_heartbeat_tick = now;
      }
    }

    /* Drain one completed VCP command line per main-loop pass. Dispatch
     * happens here, not in the UART ISR, for the same reason the beam
     * packet print is deferred to the main loop below: keep the ISR short,
     * no blocking HAL_UART_Transmit (used by the command handlers' replies)
     * from interrupt context. */
    if (vcp_line_ready)
    {
      process_command_line(vcp_line_buf);
      vcp_line_len   = 0;
      vcp_line_ready = 0;
    }

    if (g_new_packet_ready)
    {
      uint8_t  seq;
      uint8_t  status;
      int16_t  x;
      int16_t  y;
      uint32_t pkt_count;
      uint32_t err_count;
      char     line[80];
      int      len;

      /* Snapshot under a brief IRQ-disable so a new packet landing
       * mid-copy can't tear these fields -- cheap here (a few loads),
       * not worth a double-buffer for a ~20Hz smoke test. */
      __disable_irq();
      seq       = g_latest_beam.seq;
      status    = g_latest_beam.status;
      x         = g_latest_beam.x;
      y         = g_latest_beam.y;
      pkt_count = g_packet_count;
      err_count = g_checksum_error_count;
      g_new_packet_ready = 0;
      __enable_irq();

      /* Closed-loop control step -- only on a CONFIDENT detection (status
       * bit0), matching the same "don't trust this position" convention
       * the host-side scripts already use (e.g. fta_calibration_vcp.py's
       * capture_centroid). The Pi-side streamer still sends its last-known
       * position with the confidence bit clear rather than going silent
       * (see camera_view_tool.py's NucleoLink.send_position note), so
       * status must be checked here rather than assuming every packet is
       * usable. Skipping a step just holds the DAC at its last commanded
       * value -- the staleness watchdog above (1Hz) is what actually
       * catches a fully-dead stream. */
      if (g_mode == MODE_CLOSED_LOOP && (status & 1U))
      {
        run_closed_loop_step(x, HAL_GetTick());
      }

      {
        const char *x_sign, *y_sign;
        int x_whole, x_frac, y_whole, y_frac;

        decode_scaled(x, &x_sign, &x_whole, &x_frac);
        decode_scaled(y, &y_sign, &y_whole, &y_frac);

        len = snprintf(line, sizeof(line),
                        "seq=%3u status=%u x=%s%d.%01d y=%s%d.%01d pkts=%lu errs=%lu\r\n",
                        (unsigned)seq, (unsigned)status,
                        x_sign, x_whole, x_frac, y_sign, y_whole, y_frac,
                        (unsigned long)pkt_count, (unsigned long)err_count);
      }
      if (len > 0)
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 132;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* LD3 (green user LED) on PB3, per the NUCLEO-L432KC user manual (UM2179)
   * -- not board-file-configured here (this .ioc uses a bare MCU selection,
   * board=custom), and not independently verified against this specific
   * physical board. Used as a per-received-packet heartbeat for the initial
   * smoke test; double check against the board silkscreen/schematic if it
   * doesn't blink as expected. */
  GPIO_InitTypeDef led_gpio = {0};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  led_gpio.Pin   = GPIO_PIN_3;
  led_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  led_gpio.Pull  = GPIO_NOPULL;
  led_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &led_gpio);

  /* PA12 -- amplifier enable gate (active high), same GPIOA12 convention
   * documented for "FTA Controller" in rpi_camera_system CLAUDE.md. Default
   * LOW (disabled) at boot, before anything else on this pin runs -- amp
   * only comes up on an explicit VCP amp_enable command. __HAL_RCC_GPIOA_CLK_ENABLE()
   * already ran above for PA2/PA15 (USART2), no separate clock enable needed. */
  GPIO_InitTypeDef amp_gpio = {0};
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  amp_gpio.Pin   = GPIO_PIN_12;
  amp_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  amp_gpio.Pull  = GPIO_NOPULL;
  amp_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &amp_gpio);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Splits a POSITION_SCALE-scaled value back into a sign string plus
 * separate whole/fractional parts for printf-free fixed-point display
 * (this project's newlib-nano doesn't need float formatting pulled in
 * just for a one-decimal-digit debug print). Handles negative values
 * correctly even when the whole part is 0 (e.g. -4 -> "-0.4") --
 * plain integer division alone loses the sign in that case since
 * -4 / 10 == 0 in C, which would otherwise silently print "0.4".
 * Takes int32_t (not int16_t) so it can also decode g_target_x_scaled,
 * which isn't wire-format-constrained to 16 bits like the telemetry
 * fields are -- existing int16_t callers still work unchanged, promoted
 * implicitly at the call site. */
static void decode_scaled(int32_t scaled, const char **sign, int *whole, int *frac)
{
  int32_t v = scaled;

  *sign = (v < 0) ? "-" : "";
  if (v < 0)
  {
    v = -v;
  }
  *whole = v / POSITION_SCALE;
  *frac  = v % POSITION_SCALE;
}

/* buf is exactly BEAM_PKT_LEN bytes: [reg_ptr, seq, status, x_lo, x_hi,
 * y_lo, y_hi, checksum] -- see the packet-format comment near BEAM_PKT_LEN.
 * x/y are still POSITION_SCALE-scaled here, exactly as received; not
 * converted to real pixel units until printed (see decode_scaled above).
 * Called from ISR context (HAL_I2C_SlaveRxCpltCallback). */
static void process_beam_packet(const uint8_t *buf)
{
  uint8_t computed = (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6]);

  if (computed != buf[7])
  {
    /* Corrupt packet (link noise) -- drop it rather than trust a
     * potentially garbled x/y. Per-byte I2C ACK already proved each byte
     * was clocked in, not that the packet as a whole is uncorrupted. */
    g_checksum_error_count++;
    return;
  }

  g_latest_beam.seq    = buf[1];
  g_latest_beam.status = buf[2];
  g_latest_beam.x = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
  g_latest_beam.y = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
  g_latest_beam_tick = HAL_GetTick();
  g_packet_count++;
  g_new_packet_ready = 1;

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    process_beam_packet(i2c_rx_buf);
    /* Re-arm for the next transaction -- HAL_I2C_Slave_Receive_IT is a
     * one-shot request, it does not automatically repeat. */
    HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_buf, BEAM_PKT_LEN);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    /* Any I2C error (e.g. a NACK from a malformed transaction) aborts the
     * pending reception -- re-arm so the slave doesn't sit dead waiting
     * for a transfer that will never complete. */
    HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_buf, BEAM_PKT_LEN);
  }
}

/* --- DAC1 (hand-added, not CubeMX-generated -- see the USER CODE BEGIN 2
 * comment in main() for why) --------------------------------------------- */

/**
  * @brief DAC1 Initialization Function -- PA4/PA5 = DAC1_OUT1/OUT2.
  * @retval None
  */
static void MX_DAC1_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Software-set value via HAL_DAC_SetValue, no hardware trigger -- setpoints
   * only ever change on an explicit set_x/set_y (or, later, a PID step), not
   * on a timer. Output buffer enabled (HAL default) since DAC1_OUT is driving
   * an external amp input, not measured directly at the pin. */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start both channels at boot, holding the safety-clamp floor
   * (DAC_MIN_COUNT, matching g_last_dac_x/y's initializer) until a real
   * setpoint arrives -- a DAC channel that was never Start()ed reads as 0V,
   * not "off", so leaving it unstarted isn't a safer default here. */
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)DAC_MIN_COUNT);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)DAC_MIN_COUNT);
}

/* The only function that ever writes the DAC registers -- called by
 * cmd_set_axis (open_loop) today, and will be the same choke point a future
 * PID's run_control_step() calls in closed_loop. Clamps to
 * [DAC_MIN_COUNT, DAC_MAX_COUNT] unconditionally, matching "FTA
 * Controller"'s own default safety clamp. */
static void apply_dac(fta_axis_t axis, int32_t value)
{
  uint32_t channel;
  int32_t  clamped = value;

  if (clamped < DAC_MIN_COUNT) { clamped = DAC_MIN_COUNT; }
  if (clamped > DAC_MAX_COUNT) { clamped = DAC_MAX_COUNT; }

  channel = (axis == AXIS_X) ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  HAL_DAC_SetValue(&hdac1, channel, DAC_ALIGN_12B_R, (uint32_t)clamped);

  if (axis == AXIS_X)
  {
    g_last_dac_x = clamped;
  }
  else
  {
    g_last_dac_y = clamped;
  }
}

/* Single-axis (dac_y -> cx) closed-loop control step -- P+I only,
 * derivative deliberately deferred (no filtering/kick-avoidance work has
 * been done for it yet). Called once per fresh, confidently-detected
 * telemetry packet while g_mode == MODE_CLOSED_LOOP (see the call site in
 * main()'s while(1) loop) -- never during open_loop, and never for stale/
 * unconfident packets. tel_x_scaled is g_latest_beam.x, still
 * POSITION_SCALE-scaled, same as everywhere else in this file.
 *
 * The sign here (positive error -> positive dac_y correction) and the
 * choice of dac_y/cx as the controlled pair both come directly from the
 * locked-optics calibration finding that dac_y's effect on cx is
 * +0.126 px/count, the single largest coefficient in that calibration's
 * gain matrix (rpi_camera_system CLAUDE.md, 2026-08-12). If the optics
 * are ever recollimated again, both of those may need to change together
 * with a fresh calibration -- this function does not re-derive them.
 *
 * Anti-windup: the integral accumulator is clamped so its contribution
 * (g_ki * g_integral_px_s) can never exceed the full DAC output range --
 * Ki-agnostic (the bound is computed from the current g_ki each step), so
 * it stays correct across a live set_ki change rather than needing a
 * fixed magic number tuned for one particular gain. */
static void run_closed_loop_step(int16_t tel_x_scaled, uint32_t now)
{
  float   error_px;
  float   dt_s;
  float   p_term;
  float   i_term;
  int32_t output;

  error_px = (float)(g_target_x_scaled - (int32_t)tel_x_scaled) / (float)POSITION_SCALE;

  dt_s = (float)(now - g_last_control_tick) / 1000.0f;
  if (dt_s < 0.0f)
  {
    /* HAL_GetTick() wraps every ~49.7 days -- treat a negative delta as a
     * skipped step (dt effectively 0, no integral contribution this
     * round) rather than feed a huge bogus dt into the integral. */
    dt_s = 0.0f;
  }

  if (g_ki_milli != 0)
  {
    float max_integral = ((float)(DAC_MAX_COUNT - DAC_MIN_COUNT)) / fabsf(g_ki);
    g_integral_px_s += error_px * dt_s;
    if (g_integral_px_s > max_integral)  { g_integral_px_s = max_integral; }
    if (g_integral_px_s < -max_integral) { g_integral_px_s = -max_integral; }
    i_term = g_ki * g_integral_px_s;
  }
  else
  {
    i_term = 0.0f;
  }

  p_term = g_kp * error_px;
  output = g_closed_loop_base_dac_y + (int32_t)(p_term + i_term);

  apply_dac(AXIS_Y, output);  /* clamps internally to [DAC_MIN_COUNT, DAC_MAX_COUNT] */
  g_last_control_tick = now;
}

/* --- Amp / safety -------------------------------------------------------- */

static void amp_enable(void)
{
  if (g_estop_latched)
  {
    /* Latched fault blocks re-enable until an explicit clear_estop --
     * manual disable (including via estop()) always wins over enable. */
    return;
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  g_amp_enabled = 1;
}

static void amp_disable(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  g_amp_enabled = 0;
}

/* ISR-safe (only a GPIO write + two volatile stores, no blocking calls) --
 * called directly from the bare '!' byte handler in HAL_UART_RxCpltCallback,
 * carried forward from "FTA Controller"'s emergency-stop convention. Holds
 * the DAC (doesn't zero it -- the last commanded value stays latched in the
 * DAC output register, only the amp gate drops) and latches a fault that
 * clear_estop must explicitly clear before amp_enable will do anything
 * again. */
static void estop(void)
{
  amp_disable();
  g_estop_latched = 1;
}

/* --- VCP command link ----------------------------------------------------- */

static void send_line(const char *s)
{
  HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* line is NUL-terminated, no trailing \r/\n (stripped by the ISR before
 * vcp_line_ready is set). Splits on the first space into a command token
 * and a single optional argument -- every command defined so far takes at
 * most one. */
static void process_command_line(char *line)
{
  char *cmd = line;
  char *arg = strchr(line, ' ');

  if (arg != NULL)
  {
    *arg = '\0';
    arg++;
  }

  if (cmd[0] == '\0')
  {
    return; /* blank line, nothing to do */
  }
  else if (strcmp(cmd, "set_mode") == 0)
  {
    cmd_set_mode(arg);
  }
  else if (strcmp(cmd, "set_x") == 0)
  {
    cmd_set_axis(AXIS_X, arg);
  }
  else if (strcmp(cmd, "set_y") == 0)
  {
    cmd_set_axis(AXIS_Y, arg);
  }
  else if (strcmp(cmd, "amp_enable") == 0)
  {
    cmd_amp_enable();
  }
  else if (strcmp(cmd, "amp_disable") == 0)
  {
    cmd_amp_disable();
  }
  else if (strcmp(cmd, "clear_estop") == 0)
  {
    cmd_clear_estop();
  }
  else if (strcmp(cmd, "get_status") == 0)
  {
    cmd_get_status();
  }
  else if (strcmp(cmd, "set_target_x") == 0)
  {
    cmd_set_target_x(arg);
  }
  else if (strcmp(cmd, "set_kp") == 0)
  {
    cmd_set_kp(arg);
  }
  else if (strcmp(cmd, "set_ki") == 0)
  {
    cmd_set_ki(arg);
  }
  else
  {
    char resp[64];
    int  len = snprintf(resp, sizeof(resp), "ERR unknown command: %s\r\n", cmd);
    if (len > 0)
    {
      send_line(resp);
    }
  }
}

static void cmd_set_mode(const char *arg)
{
  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR set_mode requires an argument\r\n");
  }
  else if (strcmp(arg, "open_loop") == 0)
  {
    g_mode = MODE_OPEN_LOOP;
    send_line("OK mode=open_loop\r\n");
  }
  else if (strcmp(arg, "closed_loop") == 0)
  {
    if (!g_target_x_set)
    {
      /* Refuse to engage with whatever g_target_x_scaled's zero-init
       * default happens to be -- that's an arbitrary, almost certainly
       * wrong pixel target, not a safe "do nothing" value. Forces an
       * intentional set_target_x first. */
      send_line("ERR set_target_x first\r\n");
      return;
    }
    /* Bumpless transfer: bias the output off wherever dac_y already is
     * and start the integral at 0, rather than jumping straight to a raw
     * Kp*error value computed from an implicit zero base -- see
     * run_closed_loop_step's docstring for the control law itself.
     * g_last_control_tick is reset too so the first real step after this
     * gets a small, sane dt instead of however long it's been since
     * boot. */
    g_closed_loop_base_dac_y = g_last_dac_y;
    g_integral_px_s = 0.0f;
    g_last_control_tick = HAL_GetTick();
    g_mode = MODE_CLOSED_LOOP;
    send_line("OK mode=closed_loop\r\n");
  }
  else
  {
    send_line("ERR unknown mode\r\n");
  }
}

static void cmd_set_axis(fta_axis_t axis, const char *arg)
{
  long  val;
  char *endptr;
  char  resp[48];
  int   len;

  if (axis == AXIS_Y && g_mode == MODE_CLOSED_LOOP)
  {
    /* dac_y is under closed-loop control in this mode (run_closed_loop_step
     * writes it every telemetry packet) -- a manual set_y here would just
     * get overwritten on the next control step, or fight it in between.
     * Reject explicitly rather than silently accepting a command that
     * wouldn't do what it looks like it does. */
    send_line("ERR set_y is under closed-loop control -- set_mode open_loop first\r\n");
    return;
  }

  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR set_x/set_y requires an argument\r\n");
    return;
  }

  val = strtol(arg, &endptr, 10);
  if (endptr == arg)
  {
    send_line("ERR invalid integer\r\n");
    return;
  }

  apply_dac(axis, (int32_t)val); /* clamps internally to [DAC_MIN_COUNT, DAC_MAX_COUNT] */

  len = snprintf(resp, sizeof(resp), "OK %s=%ld\r\n",
                  (axis == AXIS_X) ? "x" : "y",
                  (long)((axis == AXIS_X) ? g_last_dac_x : g_last_dac_y));
  if (len > 0)
  {
    send_line(resp);
  }
}

/* Sets the closed-loop pixel setpoint for cx (plain integer pixels, NOT
 * POSITION_SCALE-scaled -- friendlier to type over the VCP than requiring
 * the operator to pre-multiply by 10; converted to scaled units here to
 * compare directly against g_latest_beam.x). Does not itself touch
 * g_mode -- has no effect on an already-running closed loop's output
 * until the next control step picks up the new target naturally (no
 * special-case bump needed, target changes are supposed to move the
 * setpoint). */
static void cmd_set_target_x(const char *arg)
{
  long  val;
  char *endptr;
  char  resp[48];
  int   len;

  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR set_target_x requires an argument (pixels)\r\n");
    return;
  }

  val = strtol(arg, &endptr, 10);
  if (endptr == arg)
  {
    send_line("ERR invalid integer\r\n");
    return;
  }

  g_target_x_scaled = (int32_t)val * POSITION_SCALE;
  g_target_x_set = 1;

  len = snprintf(resp, sizeof(resp), "OK target_x=%ld\r\n", val);
  if (len > 0)
  {
    send_line(resp);
  }
}

/* Kp/Ki are taken as milli-units integers (e.g. "set_kp 2500" -> Kp=2.5)
 * rather than a float string -- strtol only, no strtof/newlib float-scanf
 * dependency, same rationale as decode_scaled's existing avoidance of
 * float-printf (see that function's docstring and the Includes comment
 * near math.h above). Kp is DAC counts per pixel of error; Ki is DAC
 * counts per (pixel*second) of accumulated error -- see
 * run_closed_loop_step. */
static void cmd_set_kp(const char *arg)
{
  long  val;
  char *endptr;
  char  resp[48];
  int   len;

  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR set_kp requires an argument (milli-units, e.g. 2500 = Kp 2.5)\r\n");
    return;
  }

  val = strtol(arg, &endptr, 10);
  if (endptr == arg)
  {
    send_line("ERR invalid integer\r\n");
    return;
  }

  g_kp_milli = (int32_t)val;
  g_kp = (float)g_kp_milli / 1000.0f;

  len = snprintf(resp, sizeof(resp), "OK kp_milli=%ld\r\n", val);
  if (len > 0)
  {
    send_line(resp);
  }
}

static void cmd_set_ki(const char *arg)
{
  long  val;
  char *endptr;
  char  resp[48];
  int   len;

  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR set_ki requires an argument (milli-units, e.g. 500 = Ki 0.5)\r\n");
    return;
  }

  val = strtol(arg, &endptr, 10);
  if (endptr == arg)
  {
    send_line("ERR invalid integer\r\n");
    return;
  }

  g_ki_milli = (int32_t)val;
  g_ki = (float)g_ki_milli / 1000.0f;
  /* Changing Ki mid-flight invalidates whatever the old integral
   * accumulated under the previous gain -- reset rather than let a stale
   * accumulator produce a sudden i_term jump under the new one. */
  g_integral_px_s = 0.0f;

  len = snprintf(resp, sizeof(resp), "OK ki_milli=%ld\r\n", val);
  if (len > 0)
  {
    send_line(resp);
  }
}

static void cmd_amp_enable(void)
{
  amp_enable();
  if (g_amp_enabled)
  {
    send_line("OK amp_enabled\r\n");
  }
  else
  {
    send_line("ERR amp latched by estop, clear_estop first\r\n");
  }
}

static void cmd_amp_disable(void)
{
  amp_disable();
  send_line("OK amp_disabled\r\n");
}

static void cmd_clear_estop(void)
{
  g_estop_latched = 0;
  send_line("OK estop cleared\r\n");
}

static void cmd_get_status(void)
{
  uint8_t  seq, status, amp_en, estop_latched;
  int16_t  tel_x_scaled, tel_y_scaled;
  uint32_t pkt_count, err_count, last_tel_tick, now;
  int32_t  dac_x, dac_y;
  char     line[220];
  int      len;

  __disable_irq();
  seq           = g_latest_beam.seq;
  status        = g_latest_beam.status;
  tel_x_scaled  = g_latest_beam.x;
  tel_y_scaled  = g_latest_beam.y;
  last_tel_tick = g_latest_beam_tick;
  pkt_count     = g_packet_count;
  err_count     = g_checksum_error_count;
  amp_en        = g_amp_enabled;
  estop_latched = g_estop_latched;
  dac_x         = g_last_dac_x;
  dac_y         = g_last_dac_y;
  __enable_irq();

  now = HAL_GetTick();

  {
    const char *tx_sign, *ty_sign, *tgt_sign;
    int         tx_whole, tx_frac, ty_whole, ty_frac, tgt_whole, tgt_frac;
    uint32_t    tel_age_ms = (pkt_count > 0U) ? (now - last_tel_tick) : 0U;

    decode_scaled(tel_x_scaled, &tx_sign, &tx_whole, &tx_frac);
    decode_scaled(tel_y_scaled, &ty_sign, &ty_whole, &ty_frac);
    decode_scaled(g_target_x_scaled, &tgt_sign, &tgt_whole, &tgt_frac);

    len = snprintf(line, sizeof(line),
                    "STATUS mode=%s amp=%u estop=%u dac_x=%ld dac_y=%ld "
                    "tel_x=%s%d.%01d tel_y=%s%d.%01d tel_seq=%u tel_status=%u "
                    "tel_age_ms=%lu pkts=%lu errs=%lu uptime=%lus "
                    "target_x_set=%u target_x=%s%d.%01d kp_milli=%ld ki_milli=%ld\r\n",
                    (g_mode == MODE_OPEN_LOOP) ? "open_loop" : "closed_loop",
                    (unsigned)amp_en, (unsigned)estop_latched,
                    (long)dac_x, (long)dac_y,
                    tx_sign, tx_whole, tx_frac, ty_sign, ty_whole, ty_frac,
                    (unsigned)seq, (unsigned)status,
                    (unsigned long)tel_age_ms,
                    (unsigned long)pkt_count, (unsigned long)err_count,
                    (unsigned long)(now / 1000U),
                    (unsigned)g_target_x_set, tgt_sign, tgt_whole, tgt_frac,
                    (long)g_kp_milli, (long)g_ki_milli);
  }
  if (len > 0)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t c = vcp_rx_byte;

    if (c == '!')
    {
      /* Bare emergency-stop byte -- ISR-level, bypasses the line parser
       * entirely, carried forward from "FTA Controller"'s convention. */
      estop();
    }
    else if (c == '\r' || c == '\n')
    {
      if ((vcp_line_len > 0U) && !vcp_line_ready)
      {
        vcp_line_buf[vcp_line_len] = '\0';
        vcp_line_ready = 1;
      }
      /* A bare \n with nothing buffered (blank line, or the \n half of a
       * \r\n pair whose \r already triggered ready) is silently ignored --
       * vcp_line_len is only reset once the main loop drains the line. */
    }
    else if (!vcp_line_ready && (vcp_line_len < (VCP_LINE_BUF_LEN - 1U)))
    {
      vcp_line_buf[vcp_line_len++] = (char)c;
    }
    /* else: line too long, or the previous line hasn't been drained by the
     * main loop yet -- drop the byte rather than overrun/overwrite a
     * pending line. VCP commands are low-rate (occasional laptop input),
     * not a hot path, so this is not expected to matter in practice. */

    HAL_UART_Receive_IT(&huart2, &vcp_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* Same re-arm-after-any-error rationale as HAL_I2C_ErrorCallback
     * above. */
    HAL_UART_Receive_IT(&huart2, &vcp_rx_byte, 1);
  }
}

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
#ifdef USE_FULL_ASSERT
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
