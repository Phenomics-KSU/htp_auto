// Credit: Thomas D. https://github.com/tdenewiler/pid_server
// Changes: - Added zero check on DT when calcualting output.

#include "ros/ros.h"
#include "htp_auto/PID.h"

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

bool computePIDOut(htp_auto::PID::Request& req, htp_auto::PID::Response& res)
{
    double error = req.target_val - req.current_val;
    double integral_term = req.previous_integrator_val + error * req.dt;

    // Apply integrator limits.
    integral_term = limit(integral_term, req.integral_term_min, req.integral_term_max);

    if (req.dt != 0.0)
    {
        // Compute the output and cap at maximum.
        res.output = req.kp * error + req.ki * integral_term + req.kd * (error - req.previous_error) / req.dt;

        res.output = limit(res.output, -req.output_max, req.output_max);
    }
    else 
    {
        res.output = 0;
    }
    
    res.current_integrator_val = integral_term;
    res.current_error = error;

    return true;
}

int main(int argc, char **argv)
{
    // Start ROS.
    ros::init(argc, argv, "pidserver");
    ros::NodeHandle n;

    // Start PID service.
    ros::ServiceServer service = n.advertiseService("pid", computePIDOut);
    ros::spin();

    return 0;
}
