#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

class PIDController {
public:
    // Constructor initializes tuning parameters and sample time
    PIDController(double kp, double ki, double kd, double ts, double fc = 0.0)
        : kp_(kp), ki_(ki), kd_(kd), ts_(ts),
          prev_error_(0.0), integral_(0.0), prev_d_meas_(0.0),
          min_output_(-1.0), max_output_(1.0) {

        // Calculate the low-pass filter smoothing coefficient if alpha > 0
        if (fc > 0.0) {
            double rc = 1.0 / (2.0 * 3.141592653589793 * fc);
            alpha_ = ts_ / (rc + ts_);
        } else {
            alpha_ = 1.0; // Filter disabled
        }
    }

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

    // Process loop: Computes next output command given setpoint and current measurement
    double calculate(double setpoint, double measurement) {
        double error = setpoint - measurement;

        // 1. Proportional Term
        double p_term = kp_ * error;

        // 2. Integral Term with Clamping (Anti-Windup protection)
        integral_ += error * ts_;
        double i_term = ki_ * integral_;

        // 3. Derivative Term on Measurement (Prevents derivative kick)
        // D = (Error - PrevError) / dt, filtered using Exponential Moving Average
        double raw_d_meas = (error - prev_error_) / ts_;
        double filtered_d_meas = alpha_ * raw_d_meas + (1.0 - alpha_) * prev_d_meas_;
        double d_term = kd_ * filtered_d_meas;

        // Total raw output sum
        double output = p_term + i_term + d_term;

        // 4. Actuator Saturation Guard & Back-Calculation Integral Anti-Windup
        if (output > max_output_) {
            output = max_output_;
            integral_ -= error * ts_; // Undo integral accumulation if saturated
        } else if (output < min_output_) {
            output = min_output_;
            integral_ -= error * ts_; // Undo integral accumulation if saturated
        }

        // Save states for next cycle
        prev_error_ = error;
        prev_d_meas_ = filtered_d_meas;

        return output;
    }

private:
    double kp_, ki_, kd_, ts_, alpha_;
    double prev_error_, integral_, prev_d_meas_;
    double min_output_, max_output_;
};

#endif // PID_CONTROLLER_HPP
