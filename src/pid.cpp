/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>
*
*  \summary    Implementation of a generic Proportional, Integral, Derivative controller
*              with saturation on both integral term and output.
*/

#include "pid.h"

/**
 * Helper function to return input value capped between min and max.
 */
template<class T>
T limit(T value, T min, T max)
{
    if (value > max)
    {
        value = max;
    }
    else if (value < min)
    {
        value = min;
    }

    return value;
}

/**
 * Constructor
 */
PID::PID
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
    ) :
    kp_(kp),
    ki_(ki),
    kd_(kd),
    saturation_high_(saturation_high),
    saturation_low_(saturation_low),
    integral_saturation_high_(integral_saturation_high),
    integral_saturation_low_(integral_saturation_low),
    error_sum_(0),
    previous_error_(0),
    publisher_(publisher),
    pub_rate_prescaler_(pub_rate_prescaler),
    run_counter_(0)
{}

/**
 * Returns the controller output for a given error.  Derivative error is calculated
 * as a simple dx/dt.  If a derivative error is already calculated then use
 * overloaded method.
 */
double PID::calculate
    (
        double error,      // Difference between desired and actual measurement.
        double delta_time  // Change in time since last measurement. (seconds)
    )
{
    double derivative_error = 0.0;

    if (delta_time != 0.0) // Avoid division by zero.
    {
        derivative_error = (error - previous_error_) / delta_time;
    }

    return calculate(error, derivative_error, delta_time);

}

/**
 * Returns output of PID controller calculation.
 */
double PID::calculate
    (
        double error,            // Difference between desired and actual measurement.
        double derivative_error, // Change in error over specified change in time.
        double delta_time        // Change in time since last measurement. (seconds)
    )
{
    error_sum_ += error * delta_time;

    // Ensure integral term is between saturation limits.
    error_sum_ = limit(error_sum_, integral_saturation_low_, integral_saturation_high_);

    // Calculate output from given inputs.
    double result = (kp_ * error) + (ki_ * error_sum_) + (kd_ * derivative_error);

    // Ensure output is between saturation limits.
    result = limit(result, saturation_low_, saturation_high_);

    // Store error so can calculate derivative error as dx/dt if needed.
    previous_error_ = error;

    ++run_counter_;
    if (needToPublish())
    {
        publish(error, derivative_error, delta_time, result);
    }

    return result;

}

/**
 * Returns true if need to publish new PID data.
 */
bool PID::needToPublish(void)
{
    if (publisher_ == NULL) { return false; }

    // Negative prescaler means disabled.
    if (pub_rate_prescaler_ < 0) { return false; }

    // Zero prescaler means publish every run.
    if (pub_rate_prescaler_ == 0) { return true; }

    // Return true only if rolled around.
    return (run_counter_% pub_rate_prescaler_) == 0;
}

/**
 * Publishes non-constant calculation data.  Need to make sure have publisher before calling.
 */
void PID::publish(double error, double derivative_error, double delta_time, double result)
{
    publish_data_.header.stamp = ros::Time::now();
    publish_data_.error = error;
    publish_data_.error_sum = error_sum_;
    publish_data_.derivative_error = derivative_error;
    publish_data_.dt = delta_time;
    publish_data_.output = result;

    publisher_->publish(publish_data_);

}
