#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

// dt-aware variant, 2026-08-19 (rpi_camera_system) -- see that project's
// CLAUDE.md for the full story. The original version of this class (still
// what's byte-for-byte above this comment in spirit -- same P/I/D terms,
// same back-calculation anti-windup, same EMA-filtered derivative, only
// the timing math changed) assumed a fixed ts_ baked in at construction,
// with no way to tell calculate() how much real time had actually passed.
// That's fine for a PID driven by a hardware timer at a constant rate; it
// silently mis-weights the integral (and derivative) whenever the real
// call interval varies, which a telemetry-arrival-driven control loop
// (not a fixed timer) does routinely. calculate() now takes an optional
// per-call dt (real elapsed seconds since the previous call); ts_ is kept
// only as the construction-time default/fallback for a caller that
// doesn't track real time (dt <= 0). The derivative filter's smoothing
// coefficient is now recomputed per call from the real dt too (fc_ is
// stored instead of a fixed alpha_), for the same reason.
class PIDController {
public:
    // Constructor initializes tuning parameters and sample time
    PIDController(double kp, double ki, double kd, double ts, double fc = 0.0)
        : kp_(kp), ki_(ki), kd_(kd), ts_(ts), fc_(fc),
          prev_error_(0.0), integral_(0.0), prev_d_meas_(0.0),
          min_output_(-1.0), max_output_(1.0) {}

    // Set physical actuator saturation limit boundaries
    void setOutputLimits(double min, double max) {
        min_output_ = min;
        max_output_ = max;
    }

    // Reset the internal state history (Call this when changing targets abruptly)
    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
        prev_d_meas_ = 0.0;
    }

    // Process loop: Computes next output command given setpoint and current
    // measurement. dt is the real elapsed time (seconds) since the previous
    // call; if <= 0 (e.g. the first call after construction/reset, or a
    // caller that doesn't track real time), falls back to the constructor's
    // ts_ as before.
    double calculate(double setpoint, double measurement, double dt = -1.0) {
        if (dt <= 0.0) {
            dt = ts_;
        }
        double error = setpoint - measurement;

        // 1. Proportional Term
        double p_term = kp_ * error;

        // 2. Integral Term with Clamping (Anti-Windup protection)
        integral_ += error * dt;
        double i_term = ki_ * integral_;

        // 3. Derivative Term on Measurement (Prevents derivative kick)
        // D = (Error - PrevError) / dt, filtered using Exponential Moving Average
        double raw_d_meas = (error - prev_error_) / dt;
        double alpha;
        if (fc_ > 0.0) {
            double rc = 1.0 / (2.0 * 3.141592653589793 * fc_);
            alpha = dt / (rc + dt);
        } else {
            alpha = 1.0; // Filter disabled
        }
        double filtered_d_meas = alpha * raw_d_meas + (1.0 - alpha) * prev_d_meas_;
        double d_term = kd_ * filtered_d_meas;

        // Total raw output sum
        double output = p_term + i_term + d_term;

        // 4. Actuator Saturation Guard & Back-Calculation Integral Anti-Windup
        if (output > max_output_) {
            output = max_output_;
            integral_ -= error * dt; // Undo integral accumulation if saturated
        } else if (output < min_output_) {
            output = min_output_;
            integral_ -= error * dt; // Undo integral accumulation if saturated
        }

        // Save states for next cycle
        prev_error_ = error;
        prev_d_meas_ = filtered_d_meas;

        return output;
    }

private:
    double kp_, ki_, kd_, ts_, fc_;
    double prev_error_, integral_, prev_d_meas_;
    double min_output_, max_output_;
};

#endif // PID_CONTROLLER_HPP
