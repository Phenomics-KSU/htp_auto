/****************************************************************************

Based on: http://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/

Conversion from a quaternion to roll, pitch and yaw.

****************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

ros::Publisher quat_rpy_publisher;
ros::Publisher pose_rpy_publisher;
ros::Publisher pose_wc_rpy_publisher; // wc = with covariance
ros::Publisher pose_wcs_rpy_publisher; // wcs = with covariance stamped
ros::Publisher odom_rpy_publisher;

// Converts quaternion to roll pitch yaw vector
void convert(const geometry_msgs::Quaternion & quat_msg, geometry_msgs::Vector3 & rpy)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quat_msg, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Convert to degrees to make visualization easier
    rpy.x = roll * 180.0 / 3.1415926;
    rpy.y = pitch * 180.0 / 3.1415926;
    rpy.z = yaw * 180.0 / 3.1415926;
}

void QuatMsgCallback(const geometry_msgs::Quaternion msg)
{
    geometry_msgs::Vector3 rpy;
    convert(msg, rpy);
    quat_rpy_publisher.publish(rpy);
}

void PoseMsgCallback(const geometry_msgs::Pose msg)
{
    geometry_msgs::Vector3 rpy;
    convert(msg.orientation, rpy);
    pose_rpy_publisher.publish(rpy);
}

void PoseWCMsgCallback(const geometry_msgs::PoseWithCovariance msg)
{
    geometry_msgs::Vector3 rpy;
    convert(msg.pose.orientation, rpy);
    pose_wc_rpy_publisher.publish(rpy);
}

void PoseWCSMsgCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    geometry_msgs::Vector3 rpy;
    convert(msg.pose.pose.orientation, rpy);
    pose_wcs_rpy_publisher.publish(rpy);
}

void OdomMsgCallback(const nav_msgs::Odometry msg)
{
    geometry_msgs::Vector3 rpy;
    convert(msg.pose.pose.orientation, rpy);
    odom_rpy_publisher.publish(rpy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat_to_euler");

    ros::NodeHandle n;

    quat_rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_quat", 50);
    pose_rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_pose", 50);
    pose_wc_rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_pose_wc", 50);
    pose_wcs_rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_pose_wcs", 50);
    odom_rpy_publisher = n.advertise<geometry_msgs::Vector3>("rpy_odom", 50);

    // Subscribe to topics that contain quaternion fields or is a quaternion message
    ros::Subscriber quat_subscriber = n.subscribe("quat", 50, QuatMsgCallback);
    ros::Subscriber pose_subscriber = n.subscribe("pose", 50, PoseMsgCallback);
    ros::Subscriber pose_wc_subscriber = n.subscribe("pose_wc", 50, PoseWCMsgCallback);
    ros::Subscriber pose_wcs_subscriber = n.subscribe("pose_wcs", 50, PoseWCSMsgCallback);
    ros::Subscriber odom_subscriber = n.subscribe("odom", 50, OdomMsgCallback);

    ros::spin(); // wait for callbacks

    return 0;
}
