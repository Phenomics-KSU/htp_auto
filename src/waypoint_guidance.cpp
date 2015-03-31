/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>  
*
*  \summary    Implements an action server to move a differential drive robot to a 2D coordinate.
*              When a goal is active the guidance logic runs in sync with new robot pose data.
*              Publishes angular and forward linear velocity to commanded velocity topic.
*/

// Standard Headers
#include <iomanip>
#include <math.h>

// ROS Library Headers
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

// ROS Topic Headers
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <htp_auto/WaypointGuidanceAction.h>
#include <htp_auto/GuidanceParamsConfig.h>
#include <nav_msgs/Odometry.h>

// HTP Package Headers
#include "pid.h"

/**
 * Tracks a target in two dimensions.  Allows user to set a desired (x,y) position and provides 'update()' method
 * to calculate forward linear and angular velocities for a differential drive robot.
 *
 * Internally uses a simple sequential state machine with following states:
 *
 *  waiting state - No valid target to move towards.  Either already reached it or none provided yet.
 *
 *  face target state - Initially points robot towards target without moving forward.
 *
 *  travel state - Drives at specified velocity while also controlling heading to point at target.
 *                 If target is reached and don't want to stop robot then the target is complete.
 *                 Otherwise if want to stop then moves to last state.
 *
 *  closing in on target - Only entered if want to stop and have already reached acceptance radius.
 *                         Decreases velocity until the robot is close to the actual target
 *                         then comes to a complete stop.
 *
 * <<<<<<<<<<<  ALL UNITS ARE SI UNLESS OTHERWISE NOTED  >>>>>>>>>>>>>>>
 *
 */
class WaypointGuidance2D
{
public: // types

    typedef struct
    {
        double x; // local x position
        double y; // local y position
        bool stop; // if true will stop when reaching target
    } target_t;

private: // types

    // See class description for description of each state.
    enum state_t
    {
        waiting_for_target,
        facing_target,
        traveling_to_target,
        closing_in_on_target,
    };

public: // methods

    // Constructor
    WaypointGuidance2D(PID & heading_pid, PID & lateral_pid) :
        heading_pid_(heading_pid),
        lateral_pid_(lateral_pid),
        heading_pid_last_time_(0),
        lateral_pid_last_time_(0),
        travel_velocity_(.5),
        state_(waiting_for_target),
        min_acceptance_radius_(0.3), // Do not make this zero.
        minimum_stopping_speed_(.05), // Do not make this zero.
        target_(),
        last_target_valid_(false),
        last_target_(),
        distance_remaining_(0),
        bearing_(0),
        locked_desired_heading_(0),
        smallest_distance_remaining_(0),
        in_line_following_mode_(false),
        control_projection_distance_(0)
    {
        setAcceptanceRadius(min_acceptance_radius_);
    }
    
    // Reset any stateful information to default values that can't be done through setTarget.
    void reset(void)
    {
        last_target_valid_ = false;
    }

    // Set new target.  Will move on from last target even if not reached.
    void setTarget(target_t const & target)
    {
        target_ = target;
        
        reset_heading_pid_integral();
        reset_lateral_pid_integral();

        // Default state for new target.  Could be overwritten on first update.
        updateState(facing_target);
    }
    
    // New radius will be capped if too small.
    void setAcceptanceRadius(double new_radius)
    {
        if (new_radius < min_acceptance_radius_) { new_radius = min_acceptance_radius_; }
        acceptance_radius_ = new_radius;
    }
    
    // Crowfly distance from last updated position and target waypoint.
    double getDistanceRemaining(void) const { return distance_remaining_; }
    
    // Should be called when new position is measured.
    // Center Position is (x,y) and is center of rotation of vehicle.
    // Positive heading is CCW measured off x axis.
    // Velocities are output parameters.  Returns true (only once) if target is reached.
    bool update(double const * position, double heading, double & linear_velocity, double & angular_velocity)
    {
        // Default output velocities in case they're not updated.
        linear_velocity = 0;
        angular_velocity = 0;
        
        if (state_ == waiting_for_target)
        {
            return false; // Can't reach target since we don't have one.
        }

        if (!last_target_valid_)
        {
            // Create a 'fake' last target using current position.
            // Set 'stopped' to true so we will face first target below.
            last_target_.x = position[0];
            last_target_.y = position[1];
            last_target_.stop = true;
            last_target_valid_ = true;
        }

        if (!last_target_.stop)
        {
            // Don't want to stop so avoid suddenly stopping to face next target.
            updateState(traveling_to_target);
        }
     
        bool reached_target = false;

        // Compute direct path to reach target from current position so we can find distance remaining and bearing.
        double direct_path[2];
        direct_path[0] = target_.x - position[0];
        direct_path[1] = target_.y - position[1];

        distance_remaining_ = sqrt(direct_path[0]*direct_path[0] + direct_path[1]*direct_path[1]);
        
        bearing_ = atan2(direct_path[1], direct_path[0]);

        double control_position[2];
        control_position[0] = position[0];
        control_position[1] = position[1];

        if (in_line_following_mode_)
        {
            // Project position out in front of robot to make dynamics easier to control.
            control_position[0] += cos(heading) * control_projection_distance_;
            control_position[1] += sin(heading) * control_projection_distance_;
        }

        switch (state_)
        {
            case facing_target:
                faceTarget(heading, angular_velocity);
                break;
            case traveling_to_target:
                reached_target = travel(control_position, heading, linear_velocity, angular_velocity);
                break;
            case closing_in_on_target:
                reached_target = travelInsideRadius(control_position, heading, linear_velocity, angular_velocity);
                break;
        }
        
        if (reached_target)
        {
            updateState(waiting_for_target);
            
            // These should already be zero, but set them to make sure.
            if (target_.stop)
            {
                linear_velocity = 0;
                angular_velocity = 0;
            }
        }

        // Save for next target.
        last_target_ = target_;
    
        return reached_target;
    }
    
    // Should be called from reconfiguration server.
    void dynamicConfigure(htp_auto::GuidanceParamsConfig & config, uint32_t level)
    {
        travel_velocity_ = config.travel_vel;
        in_line_following_mode_ = config.line_following_mode;
        control_projection_distance_ = config.projection_distance;
        heading_pid_.set_kp(config.heading_kp);
        heading_pid_.set_ki(config.heading_ki);
        heading_pid_.set_kd(config.heading_kd);
        heading_pid_.set_saturation_high(+config.heading_sat);
        heading_pid_.set_saturation_low(-config.heading_sat);
        heading_pid_.set_integral_saturation_high(+config.heading_int_sat);
        heading_pid_.set_integral_saturation_low(-config.heading_int_sat);
        heading_pid_.set_pub_rate_prescaler(config.heading_pub_prescaler);
        lateral_pid_.set_kp(config.lateral_kp);
        lateral_pid_.set_ki(config.lateral_ki);
        lateral_pid_.set_kd(config.lateral_kd);
        lateral_pid_.set_saturation_high(+config.lateral_sat);
        lateral_pid_.set_saturation_low(-config.lateral_sat);
        lateral_pid_.set_integral_saturation_high(+config.lateral_int_sat);
        lateral_pid_.set_integral_saturation_low(-config.lateral_int_sat);
        lateral_pid_.set_pub_rate_prescaler(config.lateral_pub_prescaler);
    }

private: // methods
    
    void updateState(state_t new_state)
    {
        // State entrance actions.
        switch (new_state)
        {
            case waiting_for_target:
                break;
            case facing_target:
                break;
            case traveling_to_target:
                break;
            case closing_in_on_target:
                locked_desired_heading_ = bearing_;
                smallest_distance_remaining_ = distance_remaining_;
                break;
        }
    
        state_ = new_state;
    }

    void faceTarget(double heading, double & angular_velocity)
    {
        double heading_error = bearing_ - heading;
        
        angular_velocity = updateHeadingPID(bearing_, heading);
        
        // Maximum threshold (+/-) for heading error in order to move forward.
        double heading_threshold = 4 * M_PI / 180.0;
        
        if (fabs(heading_error) <= heading_threshold)
        {
            updateState(traveling_to_target);        
        }
    }
    
    // Returns true if reached target.
    bool travel(double const * current_position, double heading, double & linear_velocity, double & angular_velocity)
    {
        bool reached_target = false;
    
        if (!in_line_following_mode_)
        {
            // Simply try to face the next waypoint
            angular_velocity = updateHeadingPID(bearing_, heading);
        }
        else // Want to stay on line between waypoints.
        {
            double lateral_error = calculateLateralError(current_position);

            angular_velocity = updateLateralPID(lateral_error);
        }

        linear_velocity = travel_velocity_;

        if (distance_remaining_ < acceptance_radius_)
        {
            if (target_.stop)
            {
                // Need to slow down and get closer to target.
                updateState(closing_in_on_target);
            }
            else
            {
                // Reached acceptance radius and don't want to stop so call it close enough.
                reached_target = true;
            }
        }            

        return reached_target;        
    }
    
    // Should only be called if want to stop at target.  Returns true if reached target.
    bool travelInsideRadius(double const * current_position, double heading, double & linear_velocity, double & angular_velocity)
    {
        if (distance_remaining_ > smallest_distance_remaining_)
        {
            // Getting farther away from target so consider it reached.
            linear_velocity = 0;
            angular_velocity = 0;
            return true; // reached target.
        }

        smallest_distance_remaining_ = distance_remaining_;
        
        // Decrease linear velocity as position error gets smaller to make a smooth stop.  
        linear_velocity = travel_velocity_ * (distance_remaining_ / acceptance_radius_);
        
        if (fabs(linear_velocity) < minimum_stopping_speed_)
        {
            // Slowed down enough to consider we're at center of waypoint.
            linear_velocity = 0;
            angular_velocity = 0;
            return true; // reached target.
        }
        
        if (!in_line_following_mode_)
        {
            // Run heading controller with same desired heading as when we reached the acceptance radius.
            angular_velocity = updateHeadingPID(locked_desired_heading_, heading);
        }
        else // Want to stay on line between waypoints.
        {
            double lateral_error = calculateLateralError(current_position);

            angular_velocity = updateLateralPID(lateral_error);
        }
        
        return false; // because we still need to get closer to target.
    }
    
    // Returns lateral error associated with current position.
    double calculateLateralError(double const * current_position)
    {
        // Calculate vector from last waypoint to current waypoint.
        double path[2];
        path[0] = target_.x - last_target_.x;
        path[1] = target_.y - last_target_.y;

        double path_magnitude = sqrt(path[0]*path[0]+path[1]*path[1]);

        // Calculate position vector relative to the last target.
        double position[2];
        position[0] = current_position[0] - last_target_.x;
        position[1] = current_position[1] - last_target_.y;

        // Project position vector onto path vector to find how long along current path we've travelled.
        double path_travelled_magnitude = (position[0]*path[0] + position[1]*path[1]) / path_magnitude;

        double path_travelled[2];
        path_travelled[0] = path[0] * path_travelled_magnitude / path_magnitude;
        path_travelled[1] = path[1] * path_travelled_magnitude / path_magnitude;

        // The lateral error magnitude is the length of the vector between current position and
        // the projected point along the path.
        double lateral_error_magnitude = sqrt((position[0]-path_travelled[0]) * (position[0]-path_travelled[0])
                                            + (position[1]-path_travelled[1]) * (position[1]-path_travelled[1]));

        // Use cross product between path and position vector to find correct sign of lateral error.
        double path_cross_position_z = path[0]*position[1] - path[1]*position[0];
        float lateral_error_sign = path_cross_position_z < 0 ? -1.0 : 1.0;

        double lateral_error = lateral_error_sign * lateral_error_magnitude;

        return lateral_error;
    }

    // Runs PID calculation.  Returns angular velocity command.
    double updateHeadingPID(double desired_heading, double actual_heading)
    {
        ros::Time current_time = ros::Time::now();
        
        // Subtract times to get a duration and then convert to floating point representation.
        double dt = (current_time - heading_pid_last_time_).toSec();
        
        if ((dt <= 0.0) || (dt > 1.0))
        {
            // Duration is unreasonable so don't run PID loop this call.
            heading_pid_last_time_ = current_time;   
            return 0;
        }

        double heading_error = desired_heading - actual_heading;
        
        // Make sure heading is between -pi and +pi to avoid 'wrap around' problem
        // when trying to hold a heading of -/+ pi radians.
        while (heading_error > +M_PI) { heading_error -= 2.0 * M_PI; }
        while (heading_error < -M_PI) { heading_error += 2.0 * M_PI; }

        double angular_velocity = heading_pid_.calculate(heading_error, dt);
        
        heading_pid_last_time_ = current_time;

        return angular_velocity;
    }
    
    // Runs PID calculation.  Returns angular velocity command.
    // Assumes desired lateral error is zero.
    double updateLateralPID(double measured_lateral_error)
    {
        ros::Time current_time = ros::Time::now();

        // Subtract times to get a duration and then convert to floating point representation.
        double dt = (current_time - lateral_pid_last_time_).toSec();

        if ((dt <= 0.0) || (dt > 1.0))
        {
            // Duration is unreasonable so don't run PID loop this call.
            lateral_pid_last_time_ = current_time;
            return 0;
        }

        double angular_velocity = lateral_pid_.calculate(0.0 - measured_lateral_error, dt);

        lateral_pid_last_time_ = current_time;

        return angular_velocity;
    }

    void reset_heading_pid_integral(void) { heading_pid_.reset_integral_error(); }
    void reset_lateral_pid_integral(void) { lateral_pid_.reset_integral_error(); }

private: // fields

    // Used to calculate angular velocity.
    PID & heading_pid_;
    
    // Used to track line between waypoints when in 'line track' mode.
    PID & lateral_pid_;

    // Last time that PID calculation was ran.
    ros::Time heading_pid_last_time_;
    ros::Time lateral_pid_last_time_;

    // Commanded velocity when traveling to target.
    double travel_velocity_;
    
    // Current state of guidance.
    state_t state_;
    
    // Where the robot is trying to get to.
    target_t target_;
    
    // Last target the robot tried to reach.
    bool last_target_valid_;
    target_t last_target_;

    // Set smallest allowed radius to ensure heading controller doesn't get sketchy as we get really close to target.
    // **Do not make this zero.
    double min_acceptance_radius_;
    
    // How slow robot has to travel when approaching target center to be considered at waypoint.
    // **Recommended to not make this zero.
    double minimum_stopping_speed_;
    
    // How close robot must get to target before slowing down or considering it reached.
    double acceptance_radius_;
    
    // Crow-fly distance to reach target from last updated position.
    double distance_remaining_;
    
    // Heading angle to next waypoint.  Positive CCW.
    double bearing_;

    // Desired heading when acceptance radius is first reached.  Keeps heading controller from overreacting.
    double locked_desired_heading_;
    
    // The closest the robot has gotten to the target when trying to approach it inside the acceptance radius.
    double smallest_distance_remaining_;

    // True if currently trying to stay on line between waypoints instead of just facing next waypoint.
    bool in_line_following_mode_;

    // How far (meters) to control out in front of Center of Rotation position when line following.
    double control_projection_distance_;

};


/**
 * This class implements the action server that uses the base waypoint guidance logic.
 *
 * It subscribes to the robot's pose topic and synchronously publishes the linear/angular velocity commands.
 *
 *  * <<<<<<<<<<<  ALL UNITS ARE SI UNLESS OTHERWISE NOTED  >>>>>>>>>>>>>>>
 */
class WaypointGuidanceAction
{
public: // methods

    // Constructor.
    WaypointGuidanceAction(ros::NodeHandle & nh, std::string action_name, WaypointGuidance2D & guidance) :
        nh_(nh),
        server_(nh, action_name, boost::bind(&WaypointGuidanceAction::executeGoalCallback, this, _1), false),
        action_name_(action_name),
        guidance_(guidance),
        reached_target_(false)
    {
        velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    
        pose_subscriber_ = nh_.subscribe("gps", 20,  &WaypointGuidanceAction::gpsMessageReceived, this);

        goal_periodic_timer_ = nh_.createTimer(ros::Duration(.5), &WaypointGuidanceAction::goalPeriodicTimerCallback, this);
        
        // Pretty sure timers don't start automatically, but I couldn't find it in the documentation.
        goal_periodic_timer_.stop();
        
        server_.start();
    }

    void gpsMessageReceived(const nav_msgs::Odometry & message)
    {
        if (server_.isActive() && !reached_target_)
        {
            // We have an accepted goal that's still being processed.
            // Run waypoint guidance to get velocity commands, then publish them.
            geometry_msgs::Pose const & pose = message.pose.pose;
            double linear_velocity = 0;
            double angular_velocity = 0;
            double position[2] = { pose.position.x, pose.position.y };
            double heading = tf::getYaw(pose.orientation);

            // Make sure heading is between -pi and +pi to keep consistent with desired heading.
            while (heading > +M_PI) { heading -= 2.0 * M_PI; }
            while (heading < -M_PI) { heading += 2.0 * M_PI; }
           
            reached_target_ = guidance_.update(position, heading, linear_velocity, angular_velocity);
            
            geometry_msgs::Twist velocity_message;
            velocity_message.linear.x = linear_velocity;
            velocity_message.angular.z = angular_velocity;

            velocity_publisher_.publish(velocity_message);

            // Temporary. Just for debugging.
            //ROS_INFO_THROTTLE(2, "Guide: (%.3lf, %.3lf, %.1lf)", position[0], position[1], heading * 180 / M_PI);
        }
    }

    void executeGoalCallback(const htp_auto::WaypointGuidanceGoalConstPtr & goal)
    {
        // TODO validate and do something with goal frame
        WaypointGuidance2D::target_t new_target;
        new_target.x = goal->target_x;
        new_target.y = goal->target_y;
        new_target.stop = goal->stop_at_target;
        guidance_.setTarget(new_target);
        guidance_.setAcceptanceRadius(goal->acceptance_radius);

        // This call generates an error of "Cannot accept next goal when new goal is not available".
        // Seems that the goal is automatically accepted using the execute callback method.
        //server_.acceptNewGoal();
            
        // Reset flag so we don't instantly complete goal.
        reached_target_ = false;
    
        ROS_INFO_STREAM(std::setprecision(2) << std::fixed << action_name_ << ": Going to (" << goal->target_x << ", " << goal->target_y << ")");

        // Start timer to check for goal cancellation and send feedback to client.
        goal_periodic_timer_.start();
        
        while (server_.isActive() && !reached_target_)
        {          
            // Check if need to run any callbacks.
            ros::spinOnce();
        }

        goal_periodic_timer_.stop();
        
        if (!reached_target_)
        {
            // Never reached goal so make sure vehicle is stopped.  (for example if client cancelled goal)
            geometry_msgs::Twist zero_velocity_message;
            zero_velocity_message.linear.x = 0;
            zero_velocity_message.angular.z = 0;
            velocity_publisher_.publish(zero_velocity_message);

            // Reset any stateful information from guidance so can restart cleanly.
            guidance_.reset();
        }
        else // reached target.
        {
            // Notify client that the goal succeeded.
            htp_auto::WaypointGuidanceResult result;
            result.distance_remaining = guidance_.getDistanceRemaining();
            server_.setSucceeded(result);
        }
    }
    
    void goalPeriodicTimerCallback(const ros::TimerEvent & event)
    {
        if (server_.isPreemptRequested() || !ros::ok())
        {
            // Client request cancellation or node shut down.
            ROS_INFO("%s: Preempted", action_name_.c_str());
            server_.setPreempted(); // update action state
            return;
        }
        
        feedback_.distance_remaining = guidance_.getDistanceRemaining();
        server_.publishFeedback(feedback_);
    }
    
private: // fields

    // Used for subscriptions/publications/server/timer/etc.
    ros::NodeHandle & nh_;

    // Action server that accepts goals and provides feedback.
    actionlib::SimpleActionServer<htp_auto::WaypointGuidanceAction> server_; 
    
    // Feedback data to send periodically to client.
    htp_auto::WaypointGuidanceFeedback feedback_;
    
    // Logic to convert pose to angular velocities.
    WaypointGuidance2D & guidance_;

    // Timer to run slower rate events when goal is being processed.
    ros::Timer goal_periodic_timer_;
    
    // Name of action server.
    std::string action_name_;
    
    // To get robot's position and heading.
    ros::Subscriber pose_subscriber_;
    
    // To update velocity commands.
    ros::Publisher velocity_publisher_;
    
    // True if reached goal (getting to target)
    bool reached_target_;

};

int main(int argc, char **argv)
{
    // Setup ROS node.
    ros::init(argc, argv, "waypoint_guidance");
    
    // Establish this program as a node. This is what actually connects to master.
    ros::NodeHandle nh;

    // Define publisher to output PID state data for tuning.
    ros::Publisher heading_pid_pub = nh.advertise<htp_auto::PIDState>("heading_pid_state", 30);
    ros::Publisher lateral_pid_pub = nh.advertise<htp_auto::PIDState>("lateral_pid_state", 30);

    // Default params to zero because they will get set in reconfigure callback.
    PID heading_pid(0, 0, 0, 0, 0, 0, 0, &heading_pid_pub, -1);
    PID lateral_pid(0, 0, 0, 0, 0, 0, 0, &lateral_pid_pub, -1);

    WaypointGuidance2D waypoint_guidance(heading_pid, lateral_pid);

    // Setup reconfigure server to allow for parameter updates.
    dynamic_reconfigure::Server<htp_auto::GuidanceParamsConfig> config_server;
    dynamic_reconfigure::Server<htp_auto::GuidanceParamsConfig>::CallbackType config_callback;

    // Setting this callback will initially call it will all the default values.
    config_callback = boost::bind(&WaypointGuidance2D::dynamicConfigure, &waypoint_guidance, _1, _2);
    config_server.setCallback(config_callback);

    // Create action server to provide ROS interface with guidance logic.
    WaypointGuidanceAction guidance_action_server(nh, ros::this_node::getName(), waypoint_guidance);
    
    // Wait for callbacks.
    ros::spin();
    
    return 0;

}
