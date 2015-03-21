/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>
*
*  \summary    Reads in command file and sequentially adds content to a mission node.
*
*  \depend     Needs a mission node running to upload commands to.
*/

// Standard Headers
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>

// ROS Library Headers
#include <ros/ros.h>

// ROS Topic Headers
#include <std_srvs/Empty.h>
#include <htp_auto/Command.h>
#include <htp_auto/AddMissionItem.h>
#include <htp_auto/LoadMissionFile.h>

using namespace htp_auto;

// Used by service callback to interface with mission.
static ros::ServiceClient add_item_client;
static ros::ServiceClient reset_mission_client;

// Splits a string using the specified delimiter. Stores elements in output vector.
// Returns reference to output vector.
std::vector<std::string> & split(const std::string & s, char delim, std::vector<std::string> & elements)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        elements.push_back(item);
    }
    return elements;
}

// Used by string conversion method to signify a failed conversion.
class BadConversion : public std::runtime_error
{
public:
    BadConversion(std::string const & failed_string)
        : std::runtime_error("Failed string conversion on: " + failed_string)
        { }
};

// Trim whitespace from start
static inline std::string &ltrim(std::string &s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// Trim whitespace from end
static inline std::string &rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// Trim whitespace from both ends
static inline std::string &trim(std::string &s)
{
    return ltrim(rtrim(s));
}

// Converts input string to template type and returns results.
// Throws BadConversion exception on failure.
template <class T>
T parseString(std::string const & s)
{
    std::istringstream i(s);
    T x;
    if (!(i >> x))
    {
        throw BadConversion(s);
    }
    return x;
}

// Parses the comma separated line into a command structure. Returns true if successful.
bool parseLine(std::string const & line, Command & command)
{
    std::vector<std::string> elements;
    split(line, ',', elements);

    unsigned int expected_elements = 9;

    if (elements.size() != expected_elements)
    {
        ROS_WARN("Expected %d elements, parsed %d", expected_elements, elements.size());
        return false; // incorrect number of items.
    }

    try
    {
        command.type = parseString<int32_t>(elements[0]);
        command.param0 = parseString<int32_t>(elements[1]);
        command.param1 = parseString<double>(elements[2]);
        command.param2 = parseString<double>(elements[3]);
        command.param3 = parseString<double>(elements[4]);
        command.param4 = parseString<double>(elements[5]);
        command.param5 = parseString<double>(elements[6]);
        command.param6 = parseString<double>(elements[7]);
        command.param7 = parseString<double>(elements[8]);
    }
    catch (BadConversion & e)
    {
        ROS_WARN("Bad item conversion.");
        return false;
    }

    return true;
}

// Service callback. Uses file name specified in request to upload generic mission commands.
// After successfully uploading all items will reset mission. Returns true if successful.
bool loadFileCallback(LoadMissionFile::Request & request, LoadMissionFile::Response & response)
{
    std::ifstream in_stream;
    in_stream.open(request.file_name.c_str());

    if (!in_stream.is_open())
    {
        ROS_WARN_STREAM("Cannot open file: " << request.file_name);
        return false;
    }

    AddMissionItem add_item; // Service data type.
    Command command; // Command structure wrapped by service data type.
    std::string line; // Used as current line pulled out from file.
    int num_items_loaded = 0; // How many items have been successfully loaded to mission.
    int total_items_in_mission = 0; // How many items are stored in mission.

    while (std::getline(in_stream, line))
    {
        line = trim(line);

    	// Make sure it's not a blank line
        if (line == "")
        {
            continue;
        }

        if (!parseLine(line, command))
        {
            ROS_WARN_STREAM("Invalid command on line " << (num_items_loaded + 1));
            return false;
        }

        add_item.request.item = command;
        if (!add_item_client.call(add_item))
        {
            ROS_WARN("Add command service failed. Is guidance running?");
            return false;
        }

        // It's useful to report total mission items in case user forgot to clear existing mission items.
        total_items_in_mission = add_item.response.item_index + 1;

        ++num_items_loaded;
    }

    // Make sure at least one command was uploaded otherwise consider a failure.
    if (num_items_loaded == 0)
    {
        ROS_WARN("No items uploaded.");
        return false;
    }

    // Reset mission.  If fails don't consider whole callback a failure because
    // the callbacks job was primarily to upload the file which was successful.
    std_srvs::Empty no_args;
    if (!reset_mission_client.call(no_args))
    {
        ROS_WARN("Failed to reset mission after upload.");
    }

    response.num_items_loaded = num_items_loaded;
    response.total_items_in_mission = total_items_in_mission;

    return true;
}

int main(int argc, char **argv)
{
    // Setup ROS node.
    ros::init(argc, argv, "mission_loader");

    // Establish this program as a node. This is what actually connects to master.
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~"); // Private handle for parameters.

    // Connect to mission node and setup clients to consume services.
    add_item_client = nh.serviceClient<AddMissionItem>("add_mission_item");
    reset_mission_client = nh.serviceClient<std_srvs::Empty>("reset_mission");

    ros::ServiceServer load_file_service = nh.advertiseService("load_mission_file", loadFileCallback);

    // Check if caller provided mission file name.
    std::string mission_file_name;
    bool file_provided = nhp.getParam("mission_file", mission_file_name);

    if (file_provided)
    {
        LoadMissionFile::Request request;
        LoadMissionFile::Response response;
        request.file_name = mission_file_name;
        bool load_success = loadFileCallback(request, response);
        if (load_success)
        {
            ROS_INFO("Successfully loaded mission file.");
            ROS_INFO("Loaded %d items from file. Mission now contains %d items.",
                    response.num_items_loaded, response.total_items_in_mission);
        }
        else
        {
            ROS_INFO("Mission file could not be loaded.");
        }
    }
    else
    {
        ROS_INFO("File not provided. Use load service.");
    }

    // Wait for callbacks.
    ros::spin();

    return 0;
}
