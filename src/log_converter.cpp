/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>
*
*  \summary    Use with log_publisher in nmea_navsat_driver package.
*              This will publish to the /gps odom topic and /home topic
*              so that bag can be converted into mission file.
*/

// Standard Headers
#include <iostream>
#include <iomanip>
#include <math.h>
#include <string>

// ROS Topic Headers
//#include <sensor_msgs/NavSatFix.h>
#include <nmea_navsat_driver/GGK.h>
#include <nav_msgs/Odometry.h>
#include <htp_auto/OdometryUTC.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <htp_auto/Home.h>
#include <htp_auto/GPSConverterParamsConfig.h>
#include <htp_auto/ConvertLLA2ENU.h>

// Project Headers
#include "coordinate_conversions.h"
#include "utm_conversions.h"

// Global so callbacks can use them. Only one thread so don't need to worry about locking.
static ros::Publisher odom_pub;
static ros::Publisher odom_utc_pub;
static ros::Publisher home_pub;

// Flag representing if there's a valid home position set.
static bool valid_home = false;

// Rotation matrix from ECEF to NED associated with home position.
// Only valid if 'valid_home' flag is set.
static double Rne[3][3];

// Home representation for UTM method
// Easting/Northing/Altitude all in meters
static double utm_home[3];
static std::string utm_home_zone;

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

    ROS_INFO_STREAM("New home set from source " << source);
}


void GGKMessageReceived(const nmea_navsat_driver::GGK & fix)
{
    // If don't have good enough fix then don't want to publish odom message.
    // For position quality indicator:
    //  0:  Fix not available or invalid
    //  1:  Autonomous GPS fix
    //  2:  RTK float solution
    //  3:  RTK fix solution
    //  4:  Differential, code phase only solution (DGPS)
    //  5:  SBAS solution – WAAS/EGNOS/MSAS
    //  6:  RTK float or RTK location 3D Network solution
    //  7:  RTK fixed 3D Network solution
    //  8:  RTK float or RTK location 2D in a Network solution
    //  9:  RTK fixed 2D Network solution
    //  10: OmniSTAR HP/XP solution
    //  11: OmniSTAR VBS solution
    //  12: Location RTK solution
    //  13: Beacon DGPS
    int fix_quality = fix.gps_quality;
    if (fix_quality != 3) // TODO don't hard code RTK fix.
    {
        ROS_WARN_STREAM("Bad fix: " << fix_quality);
        return;
    }

    // To convert to ENU first convert LLA to ECEF.
    double lat_rad = fix.latitude * deg2rad;
    double lon_rad = fix.longitude * deg2rad;
    double lla[] = { lat_rad, lon_rad, fix.altitude };

    if (!valid_home)
    {
        setHomePosition(lla, "log converter");
    }

    // Find ENU using UTM method since other method isn't working.
    double northing, easting;
    std::string zone;
    gps_common::LLtoUTM(fix.latitude, fix.longitude, northing, easting, zone);

    // Override ENU above with offset from home UTM
    double enu[3];
    enu[0] = easting - utm_home[0];
    enu[1] = northing - utm_home[1];
    enu[2] = fix.altitude - utm_home[2];

    // TODO: handle this better or stop using UTM
    if (zone != utm_home_zone)
    {
        ROS_ERROR("CHANGED UTM ZONES!!!");
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = fix.header.stamp;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = enu[0];
    odom.pose.pose.position.y = enu[1];
    odom.pose.pose.position.z = enu[2];

    odom_pub.publish(odom);

    // Add in UTC time and UTM coordinates.
    htp_auto::OdometryUTC odom_utc;
    odom_utc.odom = odom;
    odom_utc.time = fix.utc_time;
    odom_utc.easting = easting;
    odom_utc.northing = northing;
    odom_utc.altitude = fix.altitude;
    odom_utc.utm_zone = zone;

    odom_utc_pub.publish(odom_utc);

}

int main(int argc, char **argv)
{
    // Setup ROS node.
    ros::init(argc, argv, "log_converter");

    // Establish this program as a node. This is what actually connects to master.
    ros::NodeHandle nh;

    ros::Subscriber ggk_sub = nh.subscribe("ggk", 1000, GGKMessageReceived);

    // Home publisher needs to be latched so log file can store it.
    odom_pub = nh.advertise<nav_msgs::Odometry>("gps", 1000);
    odom_utc_pub = nh.advertise<htp_auto::OdometryUTC>("gps_utc", 1000);
    home_pub = nh.advertise<htp_auto::Home>("home", 1, /*latched*/true);

    // Wait for callbacks.
    ros::spin();

    return 0;
}
