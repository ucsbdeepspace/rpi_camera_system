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

/* Second axis (dac_x <- cy error), added 2026-08-19 for completeness --
 * identical Kp/Ki/Kd/ts_s/fc_hz/output-limits to the primary axis (g_pid,
 * dac_y <- cx), reconstructed together in lockstep by the same
 * reconstruct() below. Kept genuinely separate instances (not a shared
 * one called twice) since each axis needs its own independent integral/
 * derivative history -- cx and cy are two different error signals. */
alignas(PIDController) static unsigned char g_pid2_storage[sizeof(PIDController)];
static PIDController *g_pid2 = nullptr;

static float g_ts_s   = 0.0021858f;  /* ~1/457.5s -- this default is overwritten by
                                       * pid_wrapper_init()'s real argument at startup; kept
                                       * roughly current (2026-08-19: ~440-475Hz measured
                                       * telemetry rate, was ~207-235Hz pre-ROI-change) only as
                                       * a defensive fallback, see pid_wrapper.h. */
static float g_fc_hz  = 20.0f;       /* derivative low-pass cutoff -- Phil's own example value
                                       * originally, found 2026-08-18 to be far too high relative
                                       * to this rig's own ~15.3Hz lightly-damped resonance
                                       * (confirmed via a direct free-decay ring-down test, see
                                       * CLAUDE.md) -- a 20Hz cutoff barely attenuates it at all,
                                       * which is the most likely reason even tiny Kd values
                                       * destabilized the loop. Now live-settable via set_fc so
                                       * this can be retried at a genuinely conservative cutoff
                                       * without a reflash per attempt. */
static float g_out_min = 0.0f;
static float g_out_max = 0.0f;
static float g_kp = 0.0f, g_ki = 0.0f, g_kd = 0.0f;  /* last-commanded gains, so pid_wrapper_set_fc
                                                      * can reconstruct without needing them re-sent */
/* Second axis's own independent gains -- added 2026-09-03, see
 * pid_wrapper_set_gains2's header docstring. Seeded equal to the first
 * axis's gains at pid_wrapper_init() (preserves prior "identical by
 * default" behavior until explicitly overridden). */
static float g_kp2 = 0.0f, g_ki2 = 0.0f, g_kd2 = 0.0f;

static void reconstruct(void)
{
    g_pid->~PIDController();
    g_pid = new (g_pid_storage) PIDController(g_kp, g_ki, g_kd, g_ts_s, g_fc_hz);
    g_pid->setOutputLimits(g_out_min, g_out_max);

    g_pid2->~PIDController();
    g_pid2 = new (g_pid2_storage) PIDController(g_kp2, g_ki2, g_kd2, g_ts_s, g_fc_hz);
    g_pid2->setOutputLimits(g_out_min, g_out_max);
}

extern "C" void pid_wrapper_init(float kp, float ki, float kd, float ts_s, float fc_hz,
                                   float out_min, float out_max)
{
    g_ts_s   = ts_s;
    g_fc_hz  = fc_hz;
    g_out_min = out_min;
    g_out_max = out_max;
    g_kp = kp; g_ki = ki; g_kd = kd;
    g_kp2 = kp; g_ki2 = ki; g_kd2 = kd;
    g_pid = new (g_pid_storage) PIDController(kp, ki, kd, g_ts_s, g_fc_hz);
    g_pid->setOutputLimits(g_out_min, g_out_max);
    g_pid2 = new (g_pid2_storage) PIDController(kp, ki, kd, g_ts_s, g_fc_hz);
    g_pid2->setOutputLimits(g_out_min, g_out_max);
}

extern "C" void pid_wrapper_set_gains(float kp, float ki, float kd)
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

extern "C" void pid_wrapper_set_gains2(float kp, float ki, float kd)
{
    /* Second axis, independent of the first -- see pid_wrapper.h. */
    g_kp2 = kp; g_ki2 = ki; g_kd2 = kd;
    reconstruct();
}

extern "C" void pid_wrapper_set_fc(float fc_hz)
{
    g_fc_hz = fc_hz;
    reconstruct();
}

extern "C" void pid_wrapper_set_ts(float ts_s)
{
    /* Live-settable sample time -- added 2026-08-19 specifically so the
     * "does throttling the control rate back down recover the old
     * pre-ROI-change stability" question can be tested without a reflash
     * per attempt, AND so the throttle interval and ts_s can be set
     * together by one call site (main.c's cmd_set_ctrl_rate) instead of
     * two separately-maintained values that can silently drift apart --
     * exactly the mismatch that happened earlier this same day with the
     * since-removed DIAG_CONTROL_INTERVAL_MS throttle. Reconstructs
     * (clears integral), matching set_gains/_set_fc's existing
     * convention for any change that alters the class's own dynamics. */
    g_ts_s = ts_s;
    reconstruct();
}

extern "C" void pid_wrapper_set_out_limits(float out_min, float out_max)
{
    /* setOutputLimits() alone (no reconstruct) is enough here -- unlike
     * gains/fc, PIDController's constructor doesn't derive anything from
     * min/max at construction time, so mutating the live instance is
     * safe and, unlike pid_wrapper_set_gains/_set_fc, deliberately does
     * NOT clear the integral -- this is meant to be tweaked mid-run
     * while comparing behavior, not treated as a fresh start. */
    g_out_min = out_min;
    g_out_max = out_max;
    g_pid->setOutputLimits(g_out_min, g_out_max);
    g_pid2->setOutputLimits(g_out_min, g_out_max);
}

extern "C" float pid_wrapper_calculate(float setpoint_px, float measurement_px, float dt_s)
{
    /* dt_s is the caller's real measured elapsed time since its own last
     * control step (see main.c's run_closed_loop_step) -- passed straight
     * through to PIDController::calculate()'s now dt-aware integral/
     * derivative math (2026-08-19 fix, see PIDController.hpp's own
     * docstring). Pass <= 0 for "use ts_ as before" (first call after
     * (re)construction, or a caller that doesn't track real time). */
    return g_pid->calculate(setpoint_px, measurement_px, dt_s);
}

extern "C" float pid_wrapper_calculate2(float setpoint_px, float measurement_px, float dt_s)
{
    /* Second axis, see g_pid2's docstring above. */
    return g_pid2->calculate(setpoint_px, measurement_px, dt_s);
}

extern "C" void pid_wrapper_reset(void)
{
    g_pid->reset();
    g_pid2->reset();
}
