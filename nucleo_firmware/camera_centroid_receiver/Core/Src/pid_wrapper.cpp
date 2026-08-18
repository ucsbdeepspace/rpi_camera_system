#include "PIDController.hpp"
#include "pid_wrapper.h"

/* Raw storage + placement-new, deliberately NOT a function-local static
 * object (the usual embedded-C++ idiom for "construct once, lazily").
 * A function-local static with a non-trivial constructor pulls in the
 * compiler's thread-safe "magic statics" guard-variable machinery
 * (__cxa_guard_acquire/__cxa_guard_release), which lives in
 * libstdc++/libsupc++ -- this project's link line (see Debug/makefile)
 * only pulls in -lc -lm, matching every other file in this otherwise
 * pure-C firmware. A plain namespace-scope global with a non-trivial
 * constructor would also work (STM32's startup_stm32l432xx.s already
 * calls __libc_init_array before main(), which is what runs C++ global
 * constructors), but ties construction to implicit startup ordering
 * relative to HAL_Init()/DAC init instead of an explicit call this file
 * controls. Placement-new into a byte buffer, constructed only when
 * pid_wrapper_init()/_set_gains() explicitly ask for it, avoids both
 * concerns and adds no link-time dependency beyond what placement new
 * itself needs (inline, defined in <new>, no out-of-line symbol). */
alignas(PIDController) static unsigned char g_pid_storage[sizeof(PIDController)];
static PIDController *g_pid = nullptr;

static double g_ts_s   = 0.0047619;  /* ~1/210s, see pid_wrapper.h */
static double g_fc_hz  = 20.0;       /* derivative low-pass cutoff -- Phil's own example value
                                       * originally, found 2026-08-18 to be far too high relative
                                       * to this rig's own ~15.3Hz lightly-damped resonance
                                       * (confirmed via a direct free-decay ring-down test, see
                                       * CLAUDE.md) -- a 20Hz cutoff barely attenuates it at all,
                                       * which is the most likely reason even tiny Kd values
                                       * destabilized the loop. Now live-settable via set_fc so
                                       * this can be retried at a genuinely conservative cutoff
                                       * without a reflash per attempt. */
static double g_out_min = 0.0;
static double g_out_max = 0.0;
static double g_kp = 0.0, g_ki = 0.0, g_kd = 0.0;  /* last-commanded gains, so pid_wrapper_set_fc
                                                      * can reconstruct without needing them re-sent */

static void reconstruct(void)
{
    g_pid->~PIDController();
    g_pid = new (g_pid_storage) PIDController(g_kp, g_ki, g_kd, g_ts_s, g_fc_hz);
    g_pid->setOutputLimits(g_out_min, g_out_max);
}

extern "C" void pid_wrapper_init(double kp, double ki, double kd, double ts_s, double fc_hz,
                                   double out_min, double out_max)
{
    g_ts_s   = ts_s;
    g_fc_hz  = fc_hz;
    g_out_min = out_min;
    g_out_max = out_max;
    g_kp = kp; g_ki = ki; g_kd = kd;
    g_pid = new (g_pid_storage) PIDController(kp, ki, kd, g_ts_s, g_fc_hz);
    g_pid->setOutputLimits(g_out_min, g_out_max);
}

extern "C" void pid_wrapper_set_gains(double kp, double ki, double kd)
{
    /* Reconstruct rather than mutate -- PIDController has no gain
     * setters (by design, per the class as given verbatim), and
     * reconstructing also clears prev_error_/integral_/prev_d_meas_,
     * matching this firmware's existing "changing a gain invalidates
     * the old integral" behavior (previously done by hand in
     * cmd_set_kp/cmd_set_ki). */
    g_kp = kp; g_ki = ki; g_kd = kd;
    reconstruct();
}

extern "C" void pid_wrapper_set_fc(double fc_hz)
{
    g_fc_hz = fc_hz;
    reconstruct();
}

extern "C" double pid_wrapper_calculate(double setpoint_px, double measurement_px)
{
    return g_pid->calculate(setpoint_px, measurement_px);
}

extern "C" void pid_wrapper_reset(void)
{
    g_pid->reset();
}
