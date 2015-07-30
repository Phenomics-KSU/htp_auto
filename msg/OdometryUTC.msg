# Includes UTC time with odometry message.  
# This should stay in sync with odom message by GPS converter.
float64 time # UTC time
float64 merge_time_diff # Difference between UTC times in merged position/orientation messages.
nav_msgs/Odometry odom # Local coordinates in East-North-Up
# UTM Coordinates
float64 easting
float64 northing
float64 altitude # currently above ellipsoid, but might change
string utm_zone # longitude zone followed by latitude zone i.e. 11G