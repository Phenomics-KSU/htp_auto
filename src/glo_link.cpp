
// Standard Headers
#include <string>
#include <sstream>
#include <stdexcept>

// ROS Library Headers
#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"

// HTP Package Headers
#include "globs.h"
#include "glo_rx_link.h"
#include "glo_tx_link.h"

// Serial Lib Headers
#include <serial/serial.h>

// Boost Headers
#include <boost/thread.hpp>

// Read in globs from serial port and handle them.
class Receiver
{
public: // methods

    // Constructor
    Receiver(ros::NodeHandle nh, GloRxLink & rx_link) :
        rx_link_(rx_link)
    {
        tilt_pub_ = nh.advertise<geometry_msgs::Vector3>("tilt", 2);
    }

    // Block until new data is ready and then parse it in. If a complete glob
    // is received then handles it. Should be ran on a separate thread.
    void run(void)
    {
        struct glob_meta meta_data;
        while (true)
        {
            if (rx_link_.parse(meta_data))
            {
                handleNewReceivedMessage(meta_data);
            }

            boost::this_thread::interruption_point();
        }
    }

private: // methods

    // Cast raw data to its correct type and then handle it.
    void handleNewReceivedMessage(glob_meta_t & data)
    {
        try
        {
            switch (data.id)
            {
                case GLO_ID_ASSERT_MESSAGE: {
                    verifyMessage(data, sizeof(glo_assert_message_t));
                    glo_assert_message_t * assert = (glo_assert_message_t*)data.ptr;
                    std::string message(assert->text);
                    ROS_WARN_STREAM("Received assert message: " << message);
                    } break;
                case GLO_ID_DEBUG_MESSAGE: {
                    verifyMessage(data, sizeof(glo_debug_message_t));
                    glo_debug_message_t * debug = (glo_debug_message_t*)data.ptr;
                    std::string message(debug->text);
                    ROS_INFO_STREAM("Received debug message: " << message);
                    } break;
                case GLO_ID_STATUS_DATA: {
                    verifyMessage(data, sizeof(glo_status_data_t));
                    glo_status_data_t * status_data = (glo_status_data_t*)data.ptr;
                    ROS_INFO_STREAM("Updated tilt: " << status_data->tilt_angle);
                    geometry_msgs::Vector3 v;
                    v.x = status_data->tilt_angle;
                    tilt_pub_.publish(v);
                    } break;
                default: {
                    ROS_WARN("Unhandled glob with ID %d", (unsigned int)data.id);
                    } break;
            }
        }
        catch (std::invalid_argument const & e)
        {
            ROS_ERROR("Data mismatch when handling new message.");
            ROS_ERROR_STREAM(e.what());
        }
    }

    // Throw invalid_argument exception if data isn't valid.  Expected size is the
    // number of bytes of the struct that goes with 'data'.
    void verifyMessage(glob_meta_t & data, size_t expected_size)
    {
        if (data.ptr == NULL)
        {
            throw std::invalid_argument("Received message with null data pointer.");
        }
        if (data.num_bytes != (uint8_t)expected_size)
        {
            std::ostringstream message;
            message << "Received size " << data.num_bytes << " doesn't match " << expected_size << " for ID " << data.id;
            throw std::invalid_argument(message.str());
        }
    }

private: // fields

    ros::Publisher tilt_pub_;

    GloRxLink & rx_link_;

};

// Send drive command over serial port.
void driveMessageCallback(const geometry_msgs::Vector3ConstPtr & message, GloTxLink & tx_link)
{
    ROS_INFO("Sending driving command");
    glo_driving_command_t driving = { 1, message->x, 3.0 };
    tx_link.send(glob_meta_t(GLO_ID_DRIVING_COMMAND, driving));
}

// Open serial port, start up receiver and subscribe to topics to send over port.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "glo_link");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~"); // Private handle for parameters.

    std::string port;
    if (!nhp.getParam("port", port))
    {
        port = "/dev/ttyUSB0";
    }

    int baud;
    nhp.param("baud", baud, 115200);

    ROS_INFO("Trying to open port %s with baud %d", port.c_str(), baud);

    serial::Serial * serial_port = NULL;
    try
    {
        serial_port = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
    }
    catch (serial::IOException & e)
    {
        ROS_ERROR("Port not found.");
        exit(1);
    }

    if (!serial_port || !serial_port->isOpen())
    {
        ROS_ERROR("Failed to open port");
        exit(1);
    }

    ROS_INFO("Success.");

    GloTxLink tx_link(serial_port);
    GloRxLink rx_link(serial_port);

    // Subscribe to topics to send over serial port.
    ros::Subscriber drive_sub = nh.subscribe<geometry_msgs::Vector3>("drive", 10, boost::bind(driveMessageCallback, _1, tx_link));

    Receiver receiver(nh, rx_link);
    boost::thread receive_thread(&Receiver::run, &receiver);

    // Wait for callbacks until Ctrl+C is pressed.
    ros::spin();

    receive_thread.interrupt();
    receive_thread.join();

    delete serial_port;

    return 0;
}
