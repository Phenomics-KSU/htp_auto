/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>
*
*  \summary    Merges AVR and NavSatFix messages into single odometry message in ENU coordinate frame.
*/

// Standard Headers
#include <iostream>
#include <iomanip>
#include <math.h>

// ROS Library Headers
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ROS Topic Headers
#include <sensor_msgs/NavSatFix.h>
#include <nmea_navsat_driver/AVR.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <htp_auto/GPSConverterParamsConfig.h>

// Project Headers
#include "coordinate_conversions.h"

// Global so callbacks can use them. Only one thread so don't need to worry about locking.
static ros::Publisher odom_pub;
static sensor_msgs::NavSatFix last_fix;

// Flag representing if there's a valid home position set.
static bool valid_home = false;

// Dynamic reconfiguration variables.
static double yaw_offset = 0;
static double max_pdop = 0;
static double min_sats = 0;
static int8_t min_qual_ind = 0;
static int8_t max_qual_ind = 0;

// Updates ECEF->NED rotation matrix for input argument LLA[3] (rad, m)
void setHomePosition(const double * lla, double Rne[3][3])
{
    Rne_from_lla(lla, Rne);
    valid_home = true;
}

// Saves received message and lets AVR callback do all the checking and conversions.
void NavSatFixMessageReceived(const sensor_msgs::NavSatFix & message)
{
    last_fix = message;
}

// AVR is a proprietary NMEA message containing heading and tilt data.
// This callback combines the last satellite fix message with the heading
// data and publishes an odometry message.
void AVRMessageReceived(const nmea_navsat_driver::AVR & message)
{
    // Timestamp of last satellite fix message that we used with last AVR message.
    static ros::Time last_used_fix_time;

    // Rotation matrix from ECEF to NED. Static because it's associated with home position.
    static double Rne[3][3];

    // If we don't have a new fix then don't use this AVR message.
    if (last_fix.header.stamp == last_used_fix_time)
    {
        return;
    }

    // Save this so we know for next time.
    last_used_fix_time = last_fix.header.stamp;

    // If don't have good enough fix then don't want to publish odom message.
    // For quality indicator:
    // 0: Fix not available or invalid
    // 1: Autonomous GPS fix
    // 2: Differential carrier phase solution RTK (Float)
    // 3: Differential carrier phase solution RTK (Fix)
    // 4: Differential code-based solution, DGPS
    bool pdop_ok = message.pdop <= max_pdop;
    bool num_sats_ok = message.sats_used >= min_sats;
    bool quality_ok = (message.gps_quality >= min_qual_ind) && (message.gps_quality <= max_qual_ind);

    if (!pdop_ok || !num_sats_ok || !quality_ok)
    {
        ROS_WARN_STREAM_THROTTLE(2, std::setprecision(2) << std::fixed << "GPS Checks Failed: "
                << message.pdop << ", " << message.sats_used << ", " << message.gps_quality
                << " Status: " << pdop_ok << num_sats_ok << quality_ok);
        return;
    }

    // Need to add in offset due to antenna configuration.  For example in a 'roll' mount configuration
    // the heading would read 90 degrees when facing north.
    double corrected_yaw = message.yaw - yaw_offset;

    // Need to flip sign since in ENU we should increase CCW but bx982 measures CW
    corrected_yaw *= -1.0;

    // Also need to permanently add 90 degrees since facing 'east' is zero degrees in ENU.
    corrected_yaw += M_PI / 2;

    // Cap corrected yaw between +/- PI
    while (corrected_yaw > +M_PI) { corrected_yaw -= 2 * M_PI; }
    while (corrected_yaw < -M_PI) { corrected_yaw += 2 * M_PI; }

    // To convert to ENU first convert LLA to ECEF.
    double lat_rad = last_fix.latitude * M_PI / 180.0;
    double lon_rad = last_fix.longitude * M_PI / 180.0;
    double lla[] = { lat_rad, lon_rad, last_fix.altitude };
    double ecef[3];
    lla_2_ecef(lla, ecef);

    if (!valid_home)
    {
        ROS_INFO("Setting new home position.");
        setHomePosition(lla, Rne);
    }

    // Now convert from ECEF to NED using rotation matrix (Rne)
    double ned[3];
    rot_mult(Rne, ecef, ned, false);

    // Finally convert from NED to ENU since that's the standard in ROS.
    double enu[3] = { ned[1], ned[0], -ned[2] };

    // Fill in odom data.  Use same time stamp as AVR message since that's when the measurement was made.
    nav_msgs::Odometry odom;
    odom.header.stamp = message.header.stamp;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = enu[0];
    odom.pose.pose.position.y = enu[1];
    odom.pose.pose.position.z = enu[2];
    odom.pose.pose.orientation.z = corrected_yaw;

    // Set covariance depending on GPS solution type.
    double xy_position_cov = -1;
    double z_position_cov = -1;
    double z_orientation_cov = -1;

    switch (message.gps_quality)
    {
    case 1: /* Just have GPS fix */
        xy_position_cov = 5 * 5;
        z_position_cov = 8 * 8;
        z_orientation_cov = 999999.0;
        break;
    case 2: /* RTK float solution */
        xy_position_cov = 0.25 * 0.25;
        z_position_cov = .4 * .4;
        z_orientation_cov = .035 * .035; // .035 = 2 deg
        break;
    case 3: /* RTK fix solution */
        xy_position_cov = .01 * .01;
        z_position_cov = .02 * .02;
        z_orientation_cov = .0017 * .0017; // .0017 = .1 deg
        break;
    case 4: /* DGPS code-based solution */
        xy_position_cov = 2 * 2;
        z_position_cov = 4 * 4;
        z_orientation_cov = 999999.0;
        break;
    default:
        xy_position_cov = 999999.0;
        z_position_cov = 999999.0;
        z_orientation_cov = 999999.0;
        break;
    }

    // the parameters are: (x, y, z, rot about X, rot about Y, rot about Z)
    odom.pose.covariance[0] = xy_position_cov;
    odom.pose.covariance[7] = xy_position_cov;
    odom.pose.covariance[14] = z_position_cov;
    odom.pose.covariance[21] = 99999999.0;
    odom.pose.covariance[28] = 99999999.0;
    odom.pose.covariance[35] = z_orientation_cov;

    // Set covariance on twist to be really high since we don't want to use it.
    odom.twist.covariance[0] = 99999999.0;
    odom.twist.covariance[7] = 99999999.0;
    odom.twist.covariance[14] = 99999999.0;
    odom.twist.covariance[21] = 99999999.0;
    odom.twist.covariance[28] = 99999999.0;
    odom.twist.covariance[35] = 99999999.0;

    odom_pub.publish(odom);

}

// Should be called from reconfiguration server.
void dynamicReconfigureCallback(htp_auto::GPSConverterParamsConfig & config, uint32_t level)
{
    yaw_offset = config.yaw_offset * M_PI / 180.0;
    max_pdop = config.max_pdop;
    min_sats = config.min_sats;
    min_qual_ind = config.min_qual_ind;
    max_qual_ind = config.max_qual_ind;
}

// Invalidates home flag so next position fix will become new home position.
bool resetHomeServiceCallback(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response)
{
    valid_home = false;
}

int main(int argc, char **argv)
{
    // Setup ROS node.
    ros::init(argc, argv, "mission");

    // Establish this program as a node. This is what actually connects to master.
    ros::NodeHandle nh;

    ros::Subscriber avr_sub = nh.subscribe("avr", 1, AVRMessageReceived);
    ros::Subscriber nav_fix_sub = nh.subscribe("fix", 1, NavSatFixMessageReceived);

    odom_pub = nh.advertise<nav_msgs::Odometry>("gps", 5);

    // Setup reconfigure server to allow for parameter updates.
    // Setting the callback will initially call it will all the default values.
    dynamic_reconfigure::Server<htp_auto::GPSConverterParamsConfig> config_server;
    config_server.setCallback(dynamicReconfigureCallback);

    ros::ServiceServer reset_home_service = nh.advertiseService("reset_home", resetHomeServiceCallback);

    // Wait for callbacks.
    ros::spin();

    return 0;
}
