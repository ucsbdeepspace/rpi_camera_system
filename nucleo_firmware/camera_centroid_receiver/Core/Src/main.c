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

#include "pid_wrapper.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  AXIS_X = 0,
  AXIS_Y = 1
} fta_axis_t;

/* CLOSED_LOOP: single-axis (dac_y -> cx) position control -- the
 * architecture was decided 2026-08-04 (CLAUDE.md v2) but left
 * unimplemented until the optics were locked down and dac_y->cx
 * confirmed as by far the cleanest DAC->pixel pairing (2026-08-12, same
 * file). Originally P+I only (hand-rolled); as of 2026-08-18 the P/I/D
 * math itself lives in PIDController.hpp (used verbatim, per Phil's
 * e-mail) via pid_wrapper.h/.cpp -- see run_closed_loop_step's docstring
 * below. A second controlled axis is still deliberately deferred. */
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
 * cross-context visibility. Kp/Ki/Kd are taken over the VCP as milli-
 * units integers (strtol, no float parsing) -- matches decode_scaled's
 * existing avoidance of pulling in newlib's float-formatting support for
 * the small dedicated purpose of *display*. The actual P/I/D math itself
 * (as of 2026-08-18) lives in PIDController.hpp via pid_wrapper.h/.cpp,
 * not here -- g_kp_milli/g_ki_milli/g_kd_milli below exist only so this
 * file can echo the last-commanded gains back in cmd_get_status. */
static int32_t g_target_x_scaled = 0;   /* pixel setpoint for cx, POSITION_SCALE-scaled */
static uint8_t g_target_x_set    = 0;   /* set_mode closed_loop refuses to engage until this is 1 */
static int32_t g_kp_milli = 0;          /* Kp = g_kp_milli/1000, DAC counts per pixel of error */
static int32_t g_ki_milli = 0;          /* Ki = g_ki_milli/1000, DAC counts per (pixel*second) */
static int32_t g_kd_milli = 0;          /* Kd = g_kd_milli/1000, DAC counts per (pixel/second) */
static int32_t g_fc_millihz = 20000;    /* derivative filter cutoff = g_fc_millihz/1000 Hz --
                                          * default matches PIDController.hpp's own example value,
                                          * found 2026-08-18 to be far too high relative to this
                                          * rig's ~15.3Hz resonance (see pid_wrapper.cpp) --
                                          * live-settable via set_fc so this can be retried without
                                          * a reflash per attempt. */
static int32_t g_closed_loop_base_dac_y = DAC_MIN_COUNT;  /* bumpless-transfer bias, see cmd_set_mode */

/* On-board sine setpoint generator -- 2026-08-13 "emergency" addition,
 * added after this session spent a very long time chasing why the VCP
 * link can't reliably stream target_x updates fast enough to trace a
 * real 10-20Hz sine from the host (root cause never fully resolved --
 * see rpi_camera_system CLAUDE.md). Rather than keep debugging the link,
 * sidestep it: have the firmware itself compute
 * target_x(t) = center + amplitude*sin(2*pi*freq*(t-t0)) once per
 * control step, using its own HAL_GetTick() as the time base, so the
 * only VCP traffic needed is ONE start_sine command instead of ~150+
 * target_x updates per second. The host already knows the exact
 * function it asked for, so it can fit the measured response against
 * the theoretical sin(wt) directly -- no need for the firmware to also
 * report the commanded value back over the (bandwidth-limited) link.
 * freq is stored in milli-Hz (matching the Kp/Ki milli-units-integer
 * convention elsewhere in this file) so non-integer Hz values (e.g.
 * 0.5Hz = 500) don't need float parsing over VCP. */
static uint8_t  g_sine_active          = 0;
static int32_t  g_sine_center_scaled   = 0;  /* POSITION_SCALE-scaled, same units as g_target_x_scaled */
static int32_t  g_sine_amplitude_scaled = 0; /* POSITION_SCALE-scaled */
static int32_t  g_sine_freq_millihz    = 0;
static uint32_t g_sine_start_tick      = 0;

/* Single-byte interrupt-driven VCP (USART2) receive, re-armed on every
 * completion/error -- same one-shot-then-rearm pattern as the I2C
 * reception below, just at the byte level instead of the packet level
 * since VCP commands are variable-length ASCII lines, not a fixed frame. */
static uint8_t   vcp_rx_byte;

/* Diagnostic counters -- 2026-08-13, added to get a direct, real answer
 * for why single-burst VCP commands corrupt in a deterministic way
 * (e.g. "get_status" always becomes "getsa") that hasn't changed across
 * several different fixes (RX queue, non-blocking TX, faster clock,
 * faster baud). Exposed in the heartbeat line, not get_status, since
 * get_status itself needs RX to work to even request it -- the
 * heartbeat is free-running and doesn't depend on any command landing.
 * huart2.ErrorCode is checked in HAL_UART_ErrorCallback to distinguish
 * a genuine hardware overrun (ORE -- a new byte arrived before the
 * previous one was read out, i.e. reception genuinely couldn't keep up)
 * from framing/noise/parity errors (which would point somewhere else
 * entirely, e.g. a real electrical/wiring issue rather than a timing
 * one). */
static volatile uint32_t g_uart_ore_count = 0;
static volatile uint32_t g_uart_fe_count  = 0;
static volatile uint32_t g_uart_ne_count  = 0;
static volatile uint32_t g_uart_pe_count  = 0;

/* Multi-line RX queue -- 2026-08-13. The original design held exactly one
 * pending line (a single buffer + ready flag) and silently dropped any
 * byte that arrived before the main loop drained it -- fine under the
 * "VCP commands are low-rate, occasional laptop input" assumption this
 * project started with, but that assumption broke once host-side test
 * scripts started streaming commands at real rates (e.g. an attempted
 * closed-loop sine-tracking setpoint stream) against a main loop that's
 * also busy handling ~150-200Hz I2C telemetry -- see
 * rpi_camera_system CLAUDE.md, "First closed-loop PID bench test" and
 * the closed-loop sine-test entry, same date. A queue means a command
 * that arrives while the main loop is still finishing the previous one
 * gets buffered, not dropped, as long as the queue doesn't fill. */
#define VCP_RX_QUEUE_DEPTH 8U
static char              vcp_rx_queue[VCP_RX_QUEUE_DEPTH][VCP_LINE_BUF_LEN];
static volatile uint8_t  vcp_rx_head  = 0;  /* next slot the ISR fills */
static volatile uint8_t  vcp_rx_tail  = 0;  /* next slot the main loop drains */
static volatile uint8_t  vcp_rx_count = 0;
static uint16_t          vcp_cur_len  = 0;  /* length assembled so far into vcp_rx_queue[vcp_rx_head] */

/* TX queue -- 2026-08-13, paired with the RX queue above. Every outbound
 * line (heartbeat, per-packet telemetry relay, command replies) used to
 * go through a BLOCKING HAL_UART_Transmit(..., 100) call -- with the
 * relay print firing on every I2C packet (~150-200Hz), the main loop
 * spent a large fraction of its time inside that blocking call, which
 * is exactly the window during which incoming VCP command bytes could
 * arrive and, combined with the single-pending-line RX design (now also
 * fixed above), get corrupted or dropped. enqueue_tx() copies a message
 * into this ring buffer (fast, no HAL call) and kicks off
 * HAL_UART_Transmit_IT if TX is currently idle; HAL_UART_TxCpltCallback
 * below advances the queue and starts the next message, all without the
 * main loop ever blocking on UART transmission again. 224 bytes covers
 * the largest message (get_status's ~220-byte reply) with a little
 * margin. */
#define TX_QUEUE_DEPTH   8U
#define TX_MSG_MAX_LEN   256U
typedef struct { char data[TX_MSG_MAX_LEN]; uint16_t len; } tx_msg_t;
static tx_msg_t          tx_queue[TX_QUEUE_DEPTH];
static volatile uint8_t  tx_head  = 0;  /* next slot enqueue_tx fills */
static volatile uint8_t  tx_tail  = 0;  /* slot currently being (or about to be) transmitted */
static volatile uint8_t  tx_count = 0;
static volatile uint8_t  tx_busy  = 0;

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

static void enqueue_tx(const char *s, uint16_t len);
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
static void cmd_set_kd(const char *arg);
static void cmd_set_fc(const char *arg);
static void cmd_start_sine(const char *arg);
static void cmd_stop_sine(void);
static void update_sine_target(uint32_t now);
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

  /* Construct the PIDController instance (see pid_wrapper.h) with all-
   * zero gains -- matches the previous hand-rolled controller's boot
   * default exactly (Kp=Ki=Kd=0 until a set_kp/set_ki/set_kd command
   * arrives, so a stray early set_mode closed_loop just holds dac_y at
   * its bumpless-transfer base, not garbage). Output limits are a
   * symmetric correction range spanning the full DAC span -- the
   * correction is added to g_closed_loop_base_dac_y and then hard-
   * clamped to [DAC_MIN_COUNT, DAC_MAX_COUNT] by apply_dac() regardless,
   * so this only needs to be wide enough not to clip a legitimate
   * correction before that final clamp does. */
  pid_wrapper_init(0.0, 0.0, 0.0, 1.0 / 210.0, (double)g_fc_millihz / 1000.0,
                    -(double)(DAC_MAX_COUNT - DAC_MIN_COUNT), (double)(DAC_MAX_COUNT - DAC_MIN_COUNT));

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

        char hb_line[160];
        int  hb_len = snprintf(hb_line, sizeof(hb_line),
                                "heartbeat uptime=%lus mode=%s amp=%u estop=%u pkts=%lu errs=%lu "
                                "uart_ore=%lu uart_fe=%lu uart_ne=%lu uart_pe=%lu\r\n",
                                (unsigned long)(now / 1000U),
                                (g_mode == MODE_OPEN_LOOP) ? "open_loop" : "closed_loop",
                                (unsigned)g_amp_enabled, (unsigned)g_estop_latched,
                                (unsigned long)g_packet_count,
                                (unsigned long)g_checksum_error_count,
                                (unsigned long)g_uart_ore_count, (unsigned long)g_uart_fe_count,
                                (unsigned long)g_uart_ne_count, (unsigned long)g_uart_pe_count);
        if (hb_len > 0)
        {
          enqueue_tx(hb_line, (uint16_t)hb_len);
        }
        last_heartbeat_tick = now;
      }
    }

    /* Drain one completed VCP command line per main-loop pass -- the
     * ISR-side queue (VCP_RX_QUEUE_DEPTH deep, see vcp_rx_queue's
     * declaration) means there may be several more waiting; each
     * main-loop pass drains exactly one, same as before, so a burst of
     * queued commands drains over consecutive passes rather than a
     * single pass blocking on all of them. Dispatch happens here, not in
     * the UART ISR, for the same reason the beam packet print is
     * deferred to the main loop below: keep the ISR short. Replies are
     * enqueued (enqueue_tx), not blocking-transmitted, so a slow/backed-
     * up TX can no longer stall command draining either. */
    if (vcp_rx_count > 0U)
    {
      process_command_line(vcp_rx_queue[vcp_rx_tail]);
      vcp_rx_tail = (uint8_t)((vcp_rx_tail + 1U) % VCP_RX_QUEUE_DEPTH);
      /* Guards against the RX ISR's vcp_rx_count++ -- genuinely a
       * USART2-only race (I2C1 never touches this), so scope the mask
       * to USART2_IRQn rather than a global __disable_irq() (see the
       * 2026-08-13 fix a few lines below for why that distinction turned
       * out to matter a lot). This window is a single decrement, tiny
       * either way, but there's no reason to leave it global. */
      HAL_NVIC_DisableIRQ(USART2_IRQn);
      vcp_rx_count--;
      HAL_NVIC_EnableIRQ(USART2_IRQn);
    }

    if (g_new_packet_ready)
    {
      uint8_t  seq;
      uint8_t  status;
      int16_t  x;
      int16_t  y;
      uint32_t pkt_count;
      uint32_t err_count;
      char     line[120];
      int      len;

      /* Snapshot under a brief IRQ-disable so a new packet landing
       * mid-copy can't tear these fields -- cheap here (a few loads),
       * not worth a double-buffer for a ~20Hz smoke test.
       *
       * REAL BUG fixed here 2026-08-13: these fields are only ever
       * written by the I2C1 ISR (process_beam_packet) -- USART2 never
       * touches them -- so this only ever needed to guard against I2C1,
       * not a global __disable_irq(). But __disable_irq() masks
       * EVERYTHING via PRIMASK, including USART2, and NVIC preemption
       * priority (USART2 configured higher than I2C1 specifically so it
       * can preempt) can't help against a blanket PRIMASK mask -- it
       * only affects priority-based preemption between IRQs that are
       * both individually enabled. This snapshot runs once per
       * telemetry packet (~150-200Hz) and was silently blocking UART RX
       * reception during every single one, which a direct hardware
       * check confirmed: HAL_UART_ErrorCallback's ORE (overrun) counter
       * climbed by ~1 per burst-written VCP command sent while this was
       * in place, exactly matching the long-standing "get_status" ->
       * "getsa" corruption chased all session. Narrowed to only disable
       * the two IRQs that can actually write these fields, leaving
       * USART2 free to preempt (and actually service RX bytes) the
       * entire time. */
      HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
      HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
      seq       = g_latest_beam.seq;
      status    = g_latest_beam.status;
      x         = g_latest_beam.x;
      y         = g_latest_beam.y;
      pkt_count = g_packet_count;
      err_count = g_checksum_error_count;
      g_new_packet_ready = 0;
      HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
      HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

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
        uint32_t ctrl_now = HAL_GetTick();
        if (g_sine_active)
        {
          update_sine_target(ctrl_now);
        }
        run_closed_loop_step(x, ctrl_now);
      }

      {
        const char *x_sign, *y_sign, *tgt_sign;
        int x_whole, x_frac, y_whole, y_frac, tgt_whole, tgt_frac;

        decode_scaled(x, &x_sign, &x_whole, &x_frac);
        decode_scaled(y, &y_sign, &y_whole, &y_frac);
        /* g_target_x_scaled reflects whatever run_closed_loop_step just
         * used this cycle (including update_sine_target's write above,
         * if sine mode is active) -- reporting it alongside x/y lets a
         * host-side fit compare measured cx directly against the
         * setpoint that was ACTUALLY in effect for this sample, instead
         * of reconstructing an assumed setpoint from a host-side start
         * time + elapsed wall-clock, which is exactly what produced the
         * impossible negative-lag ("cx leads its own command") artifact
         * in fta_closed_loop_onboard_sine_test.py (2026-08-13) -- that
         * script trusted a t=0 captured after a paced multi-hundred-ms
         * command round trip, systematically late relative to the
         * firmware's real start moment. Reporting the setpoint per-
         * sample removes the need to trust any host-side clock at all. */
        decode_scaled(g_target_x_scaled, &tgt_sign, &tgt_whole, &tgt_frac);

        /* g_last_dac_y is the actual commanded actuator output this
         * cycle (whatever apply_dac() last wrote to DAC1 channel 2) --
         * plain DAC counts, not POSITION_SCALE-scaled, since it's a
         * hardware setpoint, not a pixel measurement. Reporting it lets
         * a host-side plot show the real actuator command alongside the
         * resulting cx/tgt trace, instead of having to infer it from the
         * control law offline. */
        len = snprintf(line, sizeof(line),
                        "seq=%3u status=%u x=%s%d.%01d y=%s%d.%01d tgt=%s%d.%01d dac_y=%ld pkts=%lu errs=%lu\r\n",
                        (unsigned)seq, (unsigned)status,
                        x_sign, x_whole, x_frac, y_sign, y_whole, y_frac,
                        tgt_sign, tgt_whole, tgt_frac,
                        (long)g_last_dac_y,
                        (unsigned long)pkt_count, (unsigned long)err_count);
      }
      if (len > 0)
      {
        enqueue_tx(line, (uint16_t)len);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  hi2c1.Init.Timing = 0x00503D58;
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
  /* Raised from 115200 to 460800 -- 2026-08-13, now that the system clock
   * is 16MHz (HSI, see SystemClock_Config above) instead of the original
   * 4MHz. At 4MHz, 460800 was mathematically unreachable (max ~250,000
   * at 16x oversampling) and the firmware silently hung in Error_Handler
   * before ever booting -- confirmed directly, not assumed, on a first
   * attempt. At 16MHz there's ample margin (16MHz/(16x460800)=2.17, a
   * valid divisor with room to spare) -- matches the old "FTA Controller"
   * firmware's rate again, restoring real command headroom against the
   * ~150-200Hz telemetry relay traffic sharing this same wire (see
   * rpi_camera_system CLAUDE.md for the full bandwidth-budget math: at
   * 115200 baud the relay print alone left room for only ~266 command
   * bytes/s no matter how good the firmware's buffering was). Not
   * tracked in the .ioc (same situation as the DAC1/USART2-interrupt
   * settings noted elsewhere in this file) -- reapply by hand if this
   * project is ever regenerated again. */
  huart2.Init.BaudRate = 460800;
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

/* Single-axis (dac_y -> cx) closed-loop control step. Called once per
 * fresh, confidently-detected telemetry packet while
 * g_mode == MODE_CLOSED_LOOP (see the call site in main()'s while(1)
 * loop) -- never during open_loop, and never for stale/unconfident
 * packets. tel_x_scaled is g_latest_beam.x, still POSITION_SCALE-scaled,
 * same as everywhere else in this file.
 *
 * The sign here (positive error -> positive dac_y correction) and the
 * choice of dac_y/cx as the controlled pair both come directly from the
 * locked-optics calibration finding that dac_y's effect on cx is
 * +0.126 px/count, the single largest coefficient in that calibration's
 * gain matrix (rpi_camera_system CLAUDE.md, 2026-08-12). If the optics
 * are ever recollimated again, both of those may need to change together
 * with a fresh calibration -- this function does not re-derive them.
 *
 * As of 2026-08-18, the P/I/D math + anti-windup that used to live
 * directly in this function is PIDController.hpp (via pid_wrapper.h),
 * used verbatim per Phil's e-mail -- see that header for the class
 * itself and pid_wrapper.h for the calling convention/unit choices.
 * pid_wrapper_calculate() returns a CORRECTION relative to
 * g_closed_loop_base_dac_y, not an absolute DAC value; apply_dac()
 * still does the final hardware clamp exactly as before. */
static void run_closed_loop_step(int16_t tel_x_scaled, uint32_t now)
{
  double  target_px;
  double  measured_px;
  double  correction;
  int32_t output;

  (void)now;  /* PIDController.hpp's calculate() has no dt argument --
               * ts_ is fixed at construction (see pid_wrapper.h's
               * docstring on why), so the real per-packet tick is no
               * longer needed here. */

  target_px   = (double)g_target_x_scaled / (double)POSITION_SCALE;
  measured_px = (double)tel_x_scaled / (double)POSITION_SCALE;

  correction = pid_wrapper_calculate(target_px, measured_px);
  output = g_closed_loop_base_dac_y + (int32_t)correction;

  apply_dac(AXIS_Y, output);  /* clamps internally to [DAC_MIN_COUNT, DAC_MAX_COUNT] */
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

/* Copies s into the TX ring buffer and starts transmitting immediately if
 * nothing else is currently in flight -- never blocks. Truncates (rather
 * than drops) an oversized message, and drops the whole message if the
 * queue itself is full (only expected under sustained flooding well past
 * anything this firmware's own message rate produces -- heartbeat 1Hz +
 * relay ~150-200Hz + occasional command replies, comfortably inside an
 * 8-deep queue drained every main-loop pass). Only ever called from main-
 * loop context (never from ISR context -- estop(), the only ISR-level
 * caller of anything in this file, doesn't send).
 *
 * REAL BUG fixed here 2026-08-13, found only after the user pushed back
 * hard on "whole-line sends used to be reliable, why do we suddenly need
 * per-character pacing" -- and they were right to. The first version of
 * this function did the memcpy (up to ~40 bytes, for the per-packet relay
 * line) INSIDE the __disable_irq()/__enable_irq() critical section. That
 * section now runs on every telemetry packet (~150-200Hz) -- a copy that
 * size can easily take longer than one UART byte period at 460800 baud
 * (~21.7us), so this fix, meant to stop the relay print from blocking the
 * main loop, was instead creating a NEW, more frequent RX-blocking window
 * than the original blocking-HAL_UART_Transmit code ever did (that code
 * never disabled interrupts at all, just occupied the main loop). This is
 * the most likely real explanation for burst-write commands staying
 * unreliable (even fully corruption-free single-threaded, zero-contention
 * synchronous tests still failed) despite every other fix this session
 * (RX queue, faster baud, faster clock). Fixed by only holding the lock
 * for the cheap integer bookkeeping (reserve a slot, do the memcpy
 * UNLOCKED, then publish it) -- see the two-phase reserve/publish split
 * below; the slot isn't visible to the consumer ISR (HAL_UART_TxCpltCallback)
 * until tx_count is incremented in the second locked section, which only
 * happens after the memcpy completes, so there's no window where the ISR
 * could transmit not-yet-written data. */
static void enqueue_tx(const char *s, uint16_t len)
{
  uint8_t my_slot;
  uint8_t need_kick = 0;

  if (len >= TX_MSG_MAX_LEN)
  {
    len = TX_MSG_MAX_LEN - 1U;
  }

  /* Guards against HAL_UART_TxCpltCallback (a genuine USART2-IRQn race --
   * scoped to just that IRQ, not a global __disable_irq(), so I2C1 is
   * never needlessly blocked here; RX-complete shares the same IRQn as
   * TX-complete on this MCU so it's still briefly affected, unavoidably,
   * but only for these few integer ops, not a memcpy -- see this
   * function's main docstring above for the 2026-08-13 history). */
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  if (tx_count >= TX_QUEUE_DEPTH)
  {
    /* Queue full, message dropped -- see docstring above. */
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return;
  }
  my_slot = tx_head;
  tx_head = (uint8_t)((tx_head + 1U) % TX_QUEUE_DEPTH);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* Unlocked on purpose -- see docstring. Safe: this slot index was
   * claimed above (tx_head already moved past it) so nothing else will
   * try to write it, and it isn't visible to the consumer ISR until
   * tx_count is incremented below, which happens only after this copy
   * finishes. */
  memcpy(tx_queue[my_slot].data, s, len);
  tx_queue[my_slot].len = len;

  HAL_NVIC_DisableIRQ(USART2_IRQn);
  tx_count++;
  if (!tx_busy)
  {
    tx_busy = 1;
    need_kick = 1;
  }
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  if (need_kick)
  {
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx_queue[tx_tail].data, tx_queue[tx_tail].len);
  }
}

static void send_line(const char *s)
{
  enqueue_tx(s, (uint16_t)strlen(s));
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
  else if (strcmp(cmd, "set_kd") == 0)
  {
    cmd_set_kd(arg);
  }
  else if (strcmp(cmd, "set_fc") == 0)
  {
    cmd_set_fc(arg);
  }
  else if (strcmp(cmd, "start_sine") == 0)
  {
    cmd_start_sine(arg);
  }
  else if (strcmp(cmd, "stop_sine") == 0)
  {
    cmd_stop_sine();
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
    /* Bumpless transfer: bias the output off wherever dac_y already is,
     * rather than jumping straight to a raw correction computed from an
     * implicit zero base -- see run_closed_loop_step's docstring for the
     * control law itself. pid_wrapper_reset() clears the PIDController's
     * integral/derivative history for the same reason this file's
     * previous hand-rolled version zeroed its own integral here. */
    g_closed_loop_base_dac_y = g_last_dac_y;
    pid_wrapper_reset();
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

/* start_sine FREQ_MILLIHZ AMPLITUDE_PX CENTER_PX -- three
 * space-separated integers, e.g. "start_sine 10000 25 250" for a 10Hz,
 * +-25px sine around cx=250. Parses all three from `arg` by hand (unlike
 * every other command here, which takes at most one argument) since
 * process_command_line only splits the line on its FIRST space. Sets
 * g_target_x_set=1 too, same as cmd_set_target_x, so `set_mode
 * closed_loop` doesn't need a separate priming call first if this is
 * used to start things from scratch -- though the intended flow is
 * still prime -> engage closed_loop -> start_sine, for a bumpless
 * transfer into the sine (see cmd_set_mode's own docstring). */
static void cmd_start_sine(const char *arg)
{
  /* amplitude_x10 is in POSITION_SCALE units (tenths of a pixel), NOT
   * whole pixels -- whole-pixel-only amplitude (the original design)
   * was too coarse for small-amplitude tests (e.g. a ~10um peak-to-peak
   * target is ~1.7px amplitude at MICRONS_PER_PIXEL=3.0, which could
   * only round to 1 or 2px -- a 6 or 12um result, not close to 10). This
   * matches the precision g_target_x_scaled/tel_x_scaled already carry
   * elsewhere in this firmware, just exposed on the wire directly rather
   * than re-deriving it by another *POSITION_SCALE multiply here. */
  long  freq_millihz, amplitude_x10, center_px;
  char *p = (char *)arg;
  char *endptr;
  char  resp[80];
  int   len;

  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR start_sine requires FREQ_MILLIHZ AMPLITUDE_X10 CENTER_PX\r\n");
    return;
  }

  freq_millihz = strtol(p, &endptr, 10);
  if (endptr == p)
  {
    send_line("ERR invalid freq_millihz\r\n");
    return;
  }
  p = endptr;
  while (*p == ' ') { p++; }

  amplitude_x10 = strtol(p, &endptr, 10);
  if (endptr == p)
  {
    send_line("ERR invalid amplitude_x10\r\n");
    return;
  }
  p = endptr;
  while (*p == ' ') { p++; }

  center_px = strtol(p, &endptr, 10);
  if (endptr == p)
  {
    send_line("ERR invalid center_px\r\n");
    return;
  }

  if (freq_millihz <= 0)
  {
    send_line("ERR freq_millihz must be positive\r\n");
    return;
  }

  g_sine_freq_millihz     = (int32_t)freq_millihz;
  g_sine_amplitude_scaled = (int32_t)amplitude_x10;
  g_sine_center_scaled    = (int32_t)center_px * POSITION_SCALE;
  g_sine_start_tick       = HAL_GetTick();
  g_sine_active           = 1;
  g_target_x_set          = 1;

  {
    const char *amp_sign;
    int amp_whole, amp_frac;
    decode_scaled(g_sine_amplitude_scaled, &amp_sign, &amp_whole, &amp_frac);
    len = snprintf(resp, sizeof(resp),
                    "OK sine_started freq_millihz=%ld amplitude=%s%d.%01d center=%ld start_tick=%lu\r\n",
                    freq_millihz, amp_sign, amp_whole, amp_frac, center_px, (unsigned long)g_sine_start_tick);
  }
  if (len > 0)
  {
    send_line(resp);
  }
}

static void cmd_stop_sine(void)
{
  g_sine_active = 0;
  send_line("OK sine_stopped\r\n");
}

/* Called once per control step (see the call site in main()'s while(1)
 * loop, right before run_closed_loop_step) while g_sine_active -- computes
 * target_x(t) = center + amplitude*sin(2*pi*freq*(t-t0)) using the
 * firmware's own HAL_GetTick() as the time base and writes it directly
 * into g_target_x_scaled, so run_closed_loop_step itself needs no changes
 * at all -- it just sees a target that happens to be moving. sinf() (not
 * sin()) since everything else in this control path is single-precision
 * float already (see run_closed_loop_step's docstring on why: this MCU
 * has a hardware FPU for single precision only). */
static void update_sine_target(uint32_t now)
{
  float elapsed_s = (float)(now - g_sine_start_tick) / 1000.0f;
  float freq_hz = (float)g_sine_freq_millihz / 1000.0f;
  float phase = 2.0f * 3.14159265358979323846f * freq_hz * elapsed_s;  /* M_PI isn't guaranteed defined by newlib's math.h without extra feature macros -- literal instead */
  float amplitude_scaled = (float)g_sine_amplitude_scaled;
  float center_scaled = (float)g_sine_center_scaled;

  g_target_x_scaled = (int32_t)(center_scaled + amplitude_scaled * sinf(phase));
}

/* Kp/Ki/Kd are taken as milli-units integers (e.g. "set_kp 2500" ->
 * Kp=2.5) rather than a float string -- strtol only, no strtof/newlib
 * float-scanf dependency, same rationale as decode_scaled's existing
 * avoidance of float-printf (see that function's docstring and the
 * Includes comment near math.h above). Kp is DAC counts per pixel of
 * error; Ki is DAC counts per (pixel*second) of accumulated error; Kd is
 * DAC counts per (pixel/second) of error rate-of-change -- see
 * PIDController.hpp (via pid_wrapper.h) for the actual control law. */
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

  /* Reconstructs the PIDController instance with the new gain set (see
   * pid_wrapper.h -- the class has no gain setters by design, so a
   * live gain change means rebuilding it). This also resets the
   * integral/derivative history, same as this file's previous hand-
   * rolled version did explicitly on a Ki change -- now implicit in
   * every gain change, Kp included, which is at least as safe. */
  pid_wrapper_set_gains((double)g_kp_milli / 1000.0, (double)g_ki_milli / 1000.0, (double)g_kd_milli / 1000.0);

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
  pid_wrapper_set_gains((double)g_kp_milli / 1000.0, (double)g_ki_milli / 1000.0, (double)g_kd_milli / 1000.0);

  len = snprintf(resp, sizeof(resp), "OK ki_milli=%ld\r\n", val);
  if (len > 0)
  {
    send_line(resp);
  }
}

static void cmd_set_kd(const char *arg)
{
  long  val;
  char *endptr;
  char  resp[48];
  int   len;

  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR set_kd requires an argument (milli-units, e.g. 100 = Kd 0.1)\r\n");
    return;
  }

  val = strtol(arg, &endptr, 10);
  if (endptr == arg)
  {
    send_line("ERR invalid integer\r\n");
    return;
  }

  g_kd_milli = (int32_t)val;
  pid_wrapper_set_gains((double)g_kp_milli / 1000.0, (double)g_ki_milli / 1000.0, (double)g_kd_milli / 1000.0);

  len = snprintf(resp, sizeof(resp), "OK kd_milli=%ld\r\n", val);
  if (len > 0)
  {
    send_line(resp);
  }
}

/* Derivative low-pass filter cutoff, milli-Hz units (e.g. "set_fc 5000"
 * -> fc=5.0Hz), same integer-only convention as set_kp/set_ki/set_kd.
 * Added 2026-08-18 after the default 20Hz cutoff (PIDController.hpp's
 * own example value) turned out to barely attenuate this rig's ~15.3Hz
 * resonance at all (see the ring-down test in CLAUDE.md) -- live-
 * settable so a much lower cutoff can be tried without a reflash per
 * attempt. */
static void cmd_set_fc(const char *arg)
{
  long  val;
  char *endptr;
  char  resp[48];
  int   len;

  if (arg == NULL || arg[0] == '\0')
  {
    send_line("ERR set_fc requires an argument (milli-Hz, e.g. 5000 = 5.0Hz)\r\n");
    return;
  }

  val = strtol(arg, &endptr, 10);
  if (endptr == arg)
  {
    send_line("ERR invalid integer\r\n");
    return;
  }

  if (val <= 0)
  {
    send_line("ERR fc_millihz must be positive\r\n");
    return;
  }

  g_fc_millihz = (int32_t)val;
  pid_wrapper_set_fc((double)g_fc_millihz / 1000.0);

  len = snprintf(resp, sizeof(resp), "OK fc_millihz=%ld\r\n", val);
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
  char     line[280];
  int      len;

  /* Same 2026-08-13 fix as the telemetry snapshot in main()'s while(1)
   * loop -- only g_latest_beam/g_latest_beam_tick/g_packet_count/
   * g_checksum_error_count are ever written by an ISR (I2C1's), so only
   * I2C1 needs masking here. amp_en/estop_latched/dac_x/dac_y are only
   * ever written by main-loop command handlers, never an ISR -- reading
   * them under this lock is just convenience/consistency, not a real
   * race. A global __disable_irq() here would block USART2 (and thus
   * risk corrupting the very reply this function is about to send)
   * during every get_status call, not just every telemetry packet. */
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
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
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

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
                    "target_x_set=%u target_x=%s%d.%01d kp_milli=%ld ki_milli=%ld kd_milli=%ld "
                    "fc_millihz=%ld sine=%u sine_freq_millihz=%ld\r\n",
                    (g_mode == MODE_OPEN_LOOP) ? "open_loop" : "closed_loop",
                    (unsigned)amp_en, (unsigned)estop_latched,
                    (long)dac_x, (long)dac_y,
                    tx_sign, tx_whole, tx_frac, ty_sign, ty_whole, ty_frac,
                    (unsigned)seq, (unsigned)status,
                    (unsigned long)tel_age_ms,
                    (unsigned long)pkt_count, (unsigned long)err_count,
                    (unsigned long)(now / 1000U),
                    (unsigned)g_target_x_set, tgt_sign, tgt_whole, tgt_frac,
                    (long)g_kp_milli, (long)g_ki_milli, (long)g_kd_milli,
                    (long)g_fc_millihz,
                    (unsigned)g_sine_active, (long)g_sine_freq_millihz);
  }
  if (len > 0)
  {
    enqueue_tx(line, (uint16_t)len);
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
      if ((vcp_cur_len > 0U) && (vcp_rx_count < VCP_RX_QUEUE_DEPTH))
      {
        vcp_rx_queue[vcp_rx_head][vcp_cur_len] = '\0';
        vcp_rx_head = (uint8_t)((vcp_rx_head + 1U) % VCP_RX_QUEUE_DEPTH);
        vcp_rx_count++;
        vcp_cur_len = 0U;
      }
      else if (vcp_cur_len > 0U)
      {
        /* Queue is full (8 unprocessed lines already waiting) -- drop this
         * completed line rather than overrun the ring buffer. Only
         * expected under sustained flooding well past anything this
         * firmware's command set is used for; reset assembly so the next
         * line starts clean instead of concatenating onto a dropped one. */
        vcp_cur_len = 0U;
      }
      /* A bare \n with nothing buffered (blank line, or the \n half of a
       * \r\n pair whose \r already queued the line) is silently ignored. */
    }
    else if (vcp_cur_len < (VCP_LINE_BUF_LEN - 1U))
    {
      vcp_rx_queue[vcp_rx_head][vcp_cur_len++] = (char)c;
    }
    /* else: single line too long (>VCP_LINE_BUF_LEN) -- drop the byte,
     * same as the original design; not related to the queue-depth fix. */

    HAL_UART_Receive_IT(&huart2, &vcp_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint32_t err = huart->ErrorCode;
    if (err & HAL_UART_ERROR_ORE) { g_uart_ore_count++; }
    if (err & HAL_UART_ERROR_FE)  { g_uart_fe_count++;  }
    if (err & HAL_UART_ERROR_NE)  { g_uart_ne_count++;  }
    if (err & HAL_UART_ERROR_PE)  { g_uart_pe_count++;  }
    /* Same re-arm-after-any-error rationale as HAL_I2C_ErrorCallback
     * above. */
    HAL_UART_Receive_IT(&huart2, &vcp_rx_byte, 1);
  }
}

/* Fires once the current interrupt-driven transmit (started by
 * enqueue_tx) completes. Advances the TX queue and starts the next
 * message if one is waiting, otherwise clears tx_busy so the next
 * enqueue_tx call kicks off a fresh transmit. This callback is what
 * makes every send in this file non-blocking -- see enqueue_tx's
 * docstring for why that matters (it's what used to let a relay-print
 * transmit stall the main loop long enough to corrupt/drop incoming VCP
 * command bytes). */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    tx_tail = (uint8_t)((tx_tail + 1U) % TX_QUEUE_DEPTH);
    tx_count--;
    if (tx_count > 0U)
    {
      HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx_queue[tx_tail].data, tx_queue[tx_tail].len);
    }
    else
    {
      tx_busy = 0;
    }
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
