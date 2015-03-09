/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>
*
*  \summary    Merges AVR and NavSatFix messages into single odometry message in ENU coordinate frame.
*              also manages 'home' position that is origin of ENU frame.
*/

// Standard Headers
#include <iostream>
#include <iomanip>
#include <math.h>
#include <string>

// ROS Library Headers
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// ROS Topic Headers
#include <sensor_msgs/NavSatFix.h>
#include <nmea_navsat_driver/AVR.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <htp_auto/SetHome.h>
#include <htp_auto/Home.h>
#include <htp_auto/GPSConverterParamsConfig.h>

// Project Headers
#include "coordinate_conversions.h"
#include "utm_conversions.h"

// Global so callbacks can use them. Only one thread so don't need to worry about locking.
static ros::Publisher odom_pub;
static ros::Publisher home_pub;
static sensor_msgs::NavSatFix last_fix;

// **NOTE** right now home is position of primary antenna

// Flag representing if there's a valid home position set.
static bool valid_home = false;

// Rotation matrix from ECEF to NED associated with home position.
// Only valid if 'valid_home' flag is set.
static double Rne[3][3];

// Home representation for UTM method
// Easting/Northing/Altitude all in meters
static double utm_home[3];
static std::string utm_home_zone;

// Dynamic reconfiguration variables.
static double yaw_offset = 0;
static double max_pdop = 0;
static double min_sats = 0;
static int8_t min_qual_ind = 0;
static int8_t max_qual_ind = 0;

// Conversion constants
const double rad2deg = 180.0 / M_PI;
const double deg2rad = M_PI / 180.0;

// Updates ECEF->NED rotation matrix for input argument LLA[3] (rad, m)
// The 'source' string identifies who set the home position.
void setHomePosition(const double * lla, const std::string & source)
{
    Rne_from_lla(lla, Rne);
    valid_home = true;

    // For second method using UTM
    double lat_deg = lla[0] * rad2deg;
    double lon_deg = lla[1] * rad2deg;
    double altitude = lla[2];
    double northing, easting;
    gps_common::LLtoUTM(lat_deg, lon_deg, northing, easting, utm_home_zone);
    utm_home[0] = easting;
    utm_home[1] = northing;
    utm_home[2] = altitude;

    htp_auto::Home home;
    home.header.stamp = ros::Time::now();
    home.header.frame_id = "home";
    home.latitude = lla[0] * rad2deg;
    home.longitude = lla[1] * rad2deg;
    home.altitude = lla[2];
    home.source = source;

    home_pub.publish(home);
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
    	// Copy into standard types to display properly.
    	double pdop = message.pdop;
    	int sats_used = message.sats_used;
    	int gps_quality = message.gps_quality;
        ROS_WARN_STREAM_THROTTLE(2, "GPS Checks Failed");
        ROS_WARN_STREAM_THROTTLE(2, "PDOP: " << pdop << " SATS: " << sats_used << " QUAL: " << gps_quality);
        ROS_WARN_STREAM_THROTTLE(2, "Status: " << pdop_ok << num_sats_ok << quality_ok);
        return;
    }

    // Need to add in offset due to antenna configuration.  For example in a 'roll' mount configuration
    // the heading would read 90 degrees when facing north.
    double measured_yaw = message.yaw_deg * deg2rad;
    double corrected_yaw = measured_yaw - yaw_offset;

    // Need to flip sign since in ENU we should increase CCW but bx982 measures CW
    corrected_yaw *= -1.0;

    // Also need to permanently add 90 degrees since facing 'east' is zero degrees in ENU.
    corrected_yaw += M_PI / 2;

    // Cap corrected yaw between +/- PI
    while (corrected_yaw > +M_PI) { corrected_yaw -= 2 * M_PI; }
    while (corrected_yaw < -M_PI) { corrected_yaw += 2 * M_PI; }

    // To convert to ENU first convert LLA to ECEF.
    double lat_rad = last_fix.latitude * deg2rad;
    double lon_rad = last_fix.longitude * deg2rad;
    double lla[] = { lat_rad, lon_rad, last_fix.altitude };
    double ecef[3];
    lla_2_ecef(lla, ecef);

    //ROS_INFO_STREAM_THROTTLE(5, "LLA:  " << lla[0] << ", " << lla[1] << ", " << lla[2]);
    //ROS_INFO_STREAM_THROTTLE(5, "ECEF: " << ecef[0] << ", " << ecef[1] << ", " << ecef[2]);

    if (!valid_home)
    {
        ROS_INFO("Setting new home position.");
        setHomePosition(lla, "fix");
    }

    // Now convert from ECEF to NED using rotation matrix (Rne)
    double ned[3];
    rot_mult(Rne, ecef, ned, false);

    //ROS_INFO_STREAM_THROTTLE(5, "NED:  " << ned[0] << ", " << ned[1] << ", " << ned[2]);

    // Finally convert from NED to ENU since that's the standard in ROS.
    double enu[3] = { ned[1], ned[0], -ned[2] };

    // Find ENU using UTM method since above method isn't working.
    double northing, easting;
    std::string zone;
    gps_common::LLtoUTM(last_fix.latitude, last_fix.longitude, northing, easting, zone);

    // Override ENU above with offset from home UTM
    enu[0] = easting - utm_home[0];
    enu[1] = northing - utm_home[1];
    enu[2] = last_fix.altitude - utm_home[2];

    // Take into account the fact the robot is in between the two antennas and the
    // reported lat/lon is only for the primary receiver (which we mount on the right)
    double ant1_offset = message.range / 2.0; // antenna 1 offset in meters
    enu[0] += -sin(corrected_yaw) * ant1_offset; // easting
    enu[1] += cos(corrected_yaw) * ant1_offset;  // northing

    //ROS_INFO_STREAM_THROTTLE(1, "ENU: " << enu[0] << ", " << enu[1] << ", " << enu[2]);

    // TODO: handle this better or stop using UTM
    if (zone != utm_home_zone)
    {
    	ROS_ERROR("CHANGED UTM ZONES!!!");
    }

    // Fill in odom data.  Use same time stamp as AVR message since that's when the measurement was made.
    nav_msgs::Odometry odom;
    odom.header.stamp = message.header.stamp;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = enu[0];
    odom.pose.pose.position.y = enu[1];
    odom.pose.pose.position.z = enu[2];

    // Convert yaw to quaternion before storing in message.
    tf::Quaternion corrected_quat = tf::createQuaternionFromRPY(0, 0, corrected_yaw);
    odom.pose.pose.orientation.x = corrected_quat.getX();
    odom.pose.pose.orientation.y = corrected_quat.getY();
    odom.pose.pose.orientation.z = corrected_quat.getZ();
    odom.pose.pose.orientation.w = corrected_quat.getW();

    // Set covariance depending on GPS solution type.
    double xy_position_cov = -1;
    double z_position_cov = -1;
    double z_orientation_cov = -1;

    switch (message.gps_quality)
    {
    case 1: /* Just have GPS fix */
        xy_position_cov = 5 * 5;
        z_position_cov = 8 * 8;
        z_orientation_cov = 99999999.0;
        break;
    case 2: /* RTK float solution */
        xy_position_cov = 0.25 * 0.25;
        z_position_cov = .4 * .4;
        z_orientation_cov = .035 * .035; // .035 = 2 deg

        // Make really small so filter uses this.
        z_orientation_cov /= 1000000.0;
        break;
    case 3: /* RTK fix solution */
        xy_position_cov = .01 * .01;
        z_position_cov = .02 * .02;
        z_orientation_cov = .0017 * .0017; // .0017 = .1 deg

        // Make really small so filter uses this.
        z_orientation_cov /= 10000.0;
        break;
    case 4: /* DGPS code-based solution */
        xy_position_cov = 2 * 2;
        z_position_cov = 4 * 4;
        z_orientation_cov = 99999999.0;
        break;
    default:
        xy_position_cov = 99999999.0;
        z_position_cov = 99999999.0;
        z_orientation_cov = 99999999.0;
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

// Sets new home position to the LLA specified in request.
bool setHomeServiceCallback(htp_auto::SetHome::Request & request, htp_auto::SetHome::Response & response)
{
	double lat_rad = request.home.latitude * deg2rad;
	double lon_rad = request.home.longitude * deg2rad;
	double lla[] = { lat_rad, lon_rad, request.home.altitude };
	std::string source = (request.home.source == "") ? "service" : request.home.source;

	setHomePosition(lla, source);
	return true;
}

// Should be called from reconfiguration server.
void dynamicReconfigureCallback(htp_auto::GPSConverterParamsConfig & config, uint32_t level)
{
    yaw_offset = config.yaw_offset * deg2rad;
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
    ros::init(argc, argv, "gps_converter");

    // Establish this program as a node. This is what actually connects to master.
    ros::NodeHandle nh;

    ros::Subscriber avr_sub = nh.subscribe("avr", 1, AVRMessageReceived);
    ros::Subscriber nav_fix_sub = nh.subscribe("fix", 1, NavSatFixMessageReceived);

    // Home publisher needs to be latched so log file can store it.
    odom_pub = nh.advertise<nav_msgs::Odometry>("gps", 5);
    home_pub = nh.advertise<htp_auto::Home>("home", 1, /*latched*/true);

    // Setup reconfigure server to allow for parameter updates.
    // Setting the callback will initially call it will all the default values.
    dynamic_reconfigure::Server<htp_auto::GPSConverterParamsConfig> config_server;
    config_server.setCallback(dynamicReconfigureCallback);

    ros::ServiceServer reset_home_service = nh.advertiseService("reset_home", resetHomeServiceCallback);

    // Wait for callbacks.
    ros::spin();

    return 0;
}