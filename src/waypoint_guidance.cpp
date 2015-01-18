/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>  
*
*  \summary    Implements an action server to move a differential drive robot to a 2D coordinate.
*              When a goal is active the guidance logic runs in sync with new robot pose data.
*              Publishes angular and forward linear velocity to commanded velocity topic.
*  
*  \depend     Requires a running PID server for heading control.
*/

// Standard Headers
#include <iomanip>
#include <math.h>

// ROS Library Headers
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_broadcaster.h>

// ROS Topic Headers
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <htp_auto/WaypointGuidanceAction.h>
#include <htp_auto/PID.h>

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
 * The only dependency on ROS is relying on a running PID server to convert heading -> angular velocity.
 * The reason it uses a server is it exposes just about everything about the loop through a service message.
 * This message can be monitored and the data plotted using rqt_plot.
 *
 * <<<<<<<<<<<  ALL UNITS ARE SI UNLESS OTHERWISE NOTED  >>>>>>>>>>>>>>>
 *
 */
class WaypointGuidance2D
{
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
    WaypointGuidance2D(ros::ServiceClient & heading_pid_client, double travel_velocity) :
        heading_pid_client_(heading_pid_client),
        heading_pid_last_time_(0),
        travel_velocity_(travel_velocity),
        state_(waiting_for_target),
        min_acceptance_radius_(0.3), // Do not make this zero.
        minimum_stopping_speed_(.05), // Do not make this zero.
        stop_at_target_(false),
        distance_remaining_(0),
        desired_heading_(0),
        locked_desired_heading_(0),
        smallest_distance_remaining_(0)
    {
        target_[0] = 0;
        target_[1] = 0;
  
        acceptance_radius_ = min_acceptance_radius_;
        
        // TODO: setup dynamic reconfigure to change PID values at runtime
        setPIDValues();
    }
    
    // New target should be (x,y)
    void setTarget(double const * new_target, bool stop_at_target)
    {
        if (!new_target) { return; }
        target_[0] = new_target[0];
        target_[1] = new_target[1];
        
        stop_at_target_ = stop_at_target;
        
        reset_heading_pid_integral();
        
        updateState(facing_target);
    }
    
    // New radius will be capped if too small.
    void setAcceptanceRadius(double new_radius)
    {
        if (new_radius < min_acceptance_radius_) { new_radius = min_acceptance_radius_; }
        acceptance_radius_ = new_radius;
    }
    
    double getDistanceRemaining(void) const { return distance_remaining_; }
    
    // Should be called when new position is measured. Position is (x,y) and positive heading is CCW.
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
     
        bool reached_target = false;

        // Compute path to reach target from current position.
        double path[2];
        path[0] = target_[0] - position[0];
        path[1] = target_[1] - position[1];

        distance_remaining_ = sqrt(path[0]*path[0] + path[1]*path[1]);
        
        // Need to switch sign since positive heading is counter-clockwise.
        desired_heading_ = -1.0 * atan2(path[0], path[1]);

        switch (state_)
        {
            case facing_target:
                faceTarget(heading, angular_velocity);
                break;
            case traveling_to_target:
                reached_target = travel(heading, linear_velocity, angular_velocity);
                break;
            case closing_in_on_target:
                reached_target = travelInsideRadius(heading, linear_velocity, angular_velocity);
                break;
        }
        
        if (reached_target)
        {
            updateState(waiting_for_target);
            
            // These should already be zero, but set them to make sure.
            if (stop_at_target_)
            {
                linear_velocity = 0;
                angular_velocity = 0;
            }
        }
    
        return reached_target;
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
                locked_desired_heading_ = desired_heading_;
                smallest_distance_remaining_ = distance_remaining_;
                break;
        }
    
        state_ = new_state;
    }

    void faceTarget(double heading, double & angular_velocity)
    {
        double heading_error = desired_heading_ - heading;
        
        angular_velocity = updateHeadingPID(desired_heading_, heading);
        
        // Maximum threshold (+/-) for heading error in order to move forward.
        double heading_threshold = 4 * M_PI / 180.0;
        
        if (fabs(heading_error) <= heading_threshold)
        {
            updateState(traveling_to_target);        
        }
    }
    
    // Returns true if reached target.
    bool travel(double heading, double & linear_velocity, double & angular_velocity)
    {
        bool reached_target = false;
    
        angular_velocity = updateHeadingPID(desired_heading_, heading);
        
        linear_velocity = travel_velocity_;

        if (distance_remaining_ < acceptance_radius_)
        {
            if (stop_at_target_)
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
    bool travelInsideRadius(double heading, double & linear_velocity, double & angular_velocity)
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
        
        // Run heading controller with same desired heading as when we reached the acceptance radius.
        angular_velocity = updateHeadingPID(locked_desired_heading_, heading);
        
        return false; // because we still need to get closer to target.
    }
    
    // Calls PID service.  Returns angular velocity command.
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
    
        heading_pid_.request.current_val = actual_heading;
        heading_pid_.request.target_val = desired_heading;
        heading_pid_.request.dt = dt;
        
        // Call service.  This will block here until service returns.
        heading_pid_client_.call(heading_pid_);
        
        heading_pid_.request.previous_error = heading_pid_.response.current_error;
        heading_pid_.request.previous_integrator_val = heading_pid_.response.current_integrator_val;
        
        heading_pid_last_time_ = current_time;

        return heading_pid_.response.output;
    }
    
    void setPIDValues()
    {  
        heading_pid_.request.kp = 1;
        heading_pid_.request.ki = 0;
        heading_pid_.request.kd = 0;
        heading_pid_.request.current_val = 0;
        heading_pid_.request.target_val = 0;
        heading_pid_.request.previous_error = 0;
        heading_pid_.request.previous_integrator_val = 0;
        heading_pid_.request.integral_term_min = -100;
        heading_pid_.request.integral_term_max = 100;
        heading_pid_.request.dt = 1;
    }
    
    void reset_heading_pid_integral(void) { heading_pid_.request.previous_integrator_val = 0; }
    
private: // fields

    // Can call PID server to calculate angular velocity.
    ros::ServiceClient & heading_pid_client_;
    
    // Service data (request and response).
    htp_auto::PID heading_pid_;
    
    // Last time that PID server was ran.
    ros::Time heading_pid_last_time_;

    // Commanded velocity when travelling to target.
    double travel_velocity_;
    
    // Current state of guidance.
    state_t state_;
    
    // Where the robot is trying to get to.
    double target_[2];
    
    // Set smallest allowed radius to ensure heading controller doesn't get sketchy as we get really close to target.
    // **Do not make this zero.
    double min_acceptance_radius_;
    
    // How slow robot has to travel when approaching target center to be considered at waypoint.
    // **Recommended to not make this zero.
    double minimum_stopping_speed_;
    
    // How close robot must get to target before slowing down or considering it reached.
    double acceptance_radius_;
    
    // If true then robot will stop at target.
    bool stop_at_target_;
    
    // Crow-fly distance to reach target from last updated position.
    double distance_remaining_;
    
    // Where the robot needs to be facing.  Positive CCW.
    double desired_heading_;
    
    // Desired heading when acceptance radius is first reached.  Keeps heading controller from overreacting.
    double locked_desired_heading_;
    
    // The closest the robot has gotten to the target when trying to approach it inside the acceptance radius.
    double smallest_distance_remaining_;

};


/**
 * This class implements the action server that uses the base waypoint guidance logic.
 *
 * It subsribes to the robot's pose topic and synchronously publishes the linear/angular velocity commands.
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
    
        pose_subscriber_ = nh_.subscribe("robot/pose", 20,  &WaypointGuidanceAction::poseMessageReceived, this);

        goal_periodic_timer_ = nh_.createTimer(ros::Duration(.5), &WaypointGuidanceAction::goalPeriodicTimerCallback, this);
        
        // Pretty sure timers don't start automatically, but I couldn't find it in the documentation.
        goal_periodic_timer_.stop();
        
        server_.start();
    }

    void poseMessageReceived(const geometry_msgs::Pose & message)
    {
        if (server_.isActive() && !reached_target_)
        {
            // We have an accepted goal that's still being processed.
            // Run waypoint guidance to get velocity commands, then publish them.
            double linear_velocity = 0;
            double angular_velocity = 0;
            double position[2] = {message.position.x, message.position.y};
            double heading = tf::getYaw(message.orientation);
           
            reached_target_ = guidance_.update(position, heading, linear_velocity, angular_velocity);
            
            geometry_msgs::Twist velocity_message;
            velocity_message.linear.x = linear_velocity;
            velocity_message.angular.z = angular_velocity;

            velocity_publisher_.publish(velocity_message);
        }
    }

    void executeGoalCallback(const htp_auto::WaypointGuidanceGoalConstPtr & goal)
    {
        // TODO validate and do something with goal frame
    	double target[2] = { goal->target_x, goal->target_y };
        guidance_.setTarget(target, goal->stop_at_target);
        guidance_.setAcceptanceRadius(goal->acceptance_radius);
        server_.acceptNewGoal();
            
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
        
        if (reached_target_)
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
    
    // Feedback data to send periodically.
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
    
    // TODO: make this configurable
    double travel_velocity = .5;
    
    ros::ServiceClient heading_pid_client = nh.serviceClient<htp_auto::PID>("pid");
    
    WaypointGuidance2D waypoint_guidance(heading_pid_client, travel_velocity);
    
    WaypointGuidanceAction guidance_action_server(nh, ros::this_node::getName(), waypoint_guidance);
    
    // Wait for callbacks.
    ros::spin();
    
    return 0;

}
