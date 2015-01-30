#ifndef PID_INCLUDED_H
#define PID_INCLUDED_H

#include "htp_auto/PIDState.h"
#include "ros/publisher.h"

/**
 * Proportional/ Integral/ Derivative controller extended for ROS publishing.
 */
class PID
{
public: // Methods

    PID
        (
            double kp,                        // Proportional Gain
            double ki,                        // Integral Gain
            double kd,                        // Derivative Gain
            double saturation_high,           // Upper Output Saturation Limit
            double saturation_low,            // Lower Output Saturation Limit
            double integral_saturation_high,  // Upper Error Integral Saturation Limit
            double integral_saturation_low,   // Lower Error Integral Saturation Limit
            ros::Publisher * publisher,       // Can be null.  Not copied.
            int32_t pub_rate_prescaler        // If negative disables publishing.  Higher = slower publish rate.
        );

    double calculate
        (
            double error,      // Difference between desired and actual measurement.
            double delta_time  // Change in time since last measurement. (seconds)
        );

    double calculate
        (
            double error,            // Difference between desired and actual measurement.
            double derivative_error, // Change in error over specified change in time.
            double delta_time        // Change in time since last measurement. (seconds)
        );

    // Setters
    void reset_integral_error(void) { error_sum_ = 0; }
    void set_kp(double kp) { kp_ = kp; }
    void set_ki(double ki) { ki_ = ki; }
    void set_kd(double kd) { kd_ = kd; }
    void set_saturation_high(double saturation_high) { saturation_high_ = saturation_high; }
    void set_saturation_low(double saturation_low) { saturation_low_ = saturation_low; }
    void set_integral_saturation_high(double integral_saturation_high) { integral_saturation_high_ = integral_saturation_high; }
    void set_integral_saturation_low(double integral_saturation_low) { integral_saturation_low_ = integral_saturation_low; }
    void set_pub_rate_prescaler(int32_t new_value) { pub_rate_prescaler_ = new_value; }

private: // Methods

    bool needToPublish(void);

    void publish(double error, double derivative_error, double delta_time, double result);

private: // Variables

    double kp_;                       // Proportional Gain
    double ki_;                       // Integral Gain
    double kd_;                       // Derivative Gain
    double saturation_high_;          // Saturation for high output limits
    double saturation_low_;           // Saturation for low output limits
    double integral_saturation_high_; // Anti wind-up saturation
    double integral_saturation_low_;  // Anti wind-up saturation
    double error_sum_;                // Integral error
    double previous_error_;           // Error that was last used for calculation.

    // ROS specific fields for publishing calculation state/results.
    ros::Publisher * publisher_;
    int32_t pub_rate_prescaler_;
    uint32_t run_counter_;
    htp_auto::PIDState publish_data_;

}; //PID

#endif // PID_INCLUDED_H
