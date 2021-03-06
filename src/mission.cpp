/**
*  \author     Kyle McGahee <kmcgahee@ksu.edu>  
*
*  \summary    Implements a list of generic 'commands' that can be automatically executed sequentially.
*              Provides services for managing commands and the state of the mission (ie start, reset). 
*  
*  \depend     Needs a guidance action server running to execute waypoint commands.
*/

// Standard Headers
#include <vector>

// ROS Library Headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// ROS Topic Headers
#include <std_srvs/Empty.h>
#include <htp_auto/Command.h>
#include <htp_auto/WaypointGuidanceAction.h>
#include <htp_auto/AddMissionItem.h>
#include <htp_auto/SetMissionItem.h>
#include <htp_auto/SetHome.h>
#include <htp_auto/ConvertLLA2ENU.h>

using namespace htp_auto;

// Corresponds to 'type' field of Command message.
enum
{
    MISSION_ITEM_WAYPOINT,
    MISSION_ITEM_SET_HOME,
};

// Corresponds to 'frame' field of Command message.
enum
{
    FRAME_LOCAL,
    FRAME_LLA,
};

/**
* Stores generic commands that can be executed sequentially.
* By default the mission is stopped and 'start' must be called after adding at least one command.
*
* Logically commands are split into the following two groups:
*   - Simple Commands: Run instantly. Usually use as a service client or publish to a topic. For example changing travel velocity.
*   - Action Commands: Take non-trivial time to complete. Usually use a non-blocking ROS action server. For example waypoint guidance.  
* 
* The intended use of this class is to use the "executeToNextAction" method in order to process all simple commands
* until either an action command is hit (in which case it's started) or there are no more commands left.
*
* The 'goal complete' callback of the action client is then in charge of continuing mission execution.
*
* The design reason behind this to create a non busy waiting mission that is completely event driven. 
*/
class Mission
{
public: // methods

    // Constructor
    Mission(ros::NodeHandle & nh) :
        current_index_(0),
        nh_(nh),
        guidance_client_("guidance", true /*create separate thread*/),
        started_(false)
    {
        set_home_client_ = nh.serviceClient<SetHome>("set_home");
        convert_lla_2_enu_client_ = nh.serviceClient<ConvertLLA2ENU>("lla_2_enu");
    }
    
    // Executes all simple commands until either an action command is hit (in which case it's started) or there are no more commands.
    // Returns false if a command fails to execute successfully or the end of the mission is reached.
    bool executeToNextAction(void)
    {
        // True if the command that was just executed was an action command.
        bool is_action = false;
        
        // True if the command was executed successfully.
        bool success = true;
        
        while (true)
        {
            success = executeCurrentItem(is_action);

            if (is_action || !success)
            {
                // If command was an action it should have started.
                break;
            }

            advanceCurrentIndex();
        }
        
        return success;
    }
    
    // Executes the current command in the list. 'is_action' output parameter is only valid if there is a valid command to execute.
    // Returns true if successful.  If not successful 'started' flag will become false (if it already wasn't).
    bool executeCurrentItem(bool & is_action)
    {
        if (!started_)
        {
            ROS_INFO("Mission not started.");
            return false;
        }
        
        if (items_.size() == 0)
        {
            ROS_INFO("No mission items stored.");
            started_ = false;
            return false;
        }
        
        if (current_index_ >= items_.size())
        {
            ROS_INFO("No mission items remaining.");
            started_ = false;
            return false;
        }
        
        Command & current_item = items_[current_index_];
        
        bool success = executeItem(current_item, is_action);
        
        if (!success)
        {
            ROS_WARN("Mission command failed. Type ID: %d", current_item.type); 
            started_ = false;
        }

        return success;
    }
    
    // Increments current index by one.  Returns false if failed because current index is already at max.
    bool advanceCurrentIndex(void)
    {
        // Hold invariant that index can never be greater than # of items.
        if ((current_index_ + 1) > items_.size())
        {
            return false;
        }

        ++current_index_;

        return true; // Successfully incremented.
    }

    // Appends (copies) command to end of list.  Returns its index in the list.
    uint32_t addItem(Command const & new_item)
    {
        items_.push_back(new_item);
        
        return items_.size() - 1;
    }
    
    // Deletes all commands and resets current command index.
    void clearAllItems(void)
    {
        ROS_INFO("Clearing all mission items.");
        items_.clear();
        reset();
    }
    
    // Sets current command index to the specified value. Returns true if successful.
    bool setCurrentItemIndex(uint32_t new_index)
    {
        if (new_index > items_.size())
        {
            ROS_WARN("Capping index from %u to %u which signifies end of mission.", new_index, items_.size());
            new_index = items_.size();
        }

        // Make sure that if a 'set home' item came before the item
        // that's being set then the home position is updated because otherwise
        // when the mission is started everything will be off.
        if ((new_index > 0) && (items_.size() > 0))
        {
            bool found_set_home_item = false;
            uint32_t set_home_item_index = 0;
            // At this point new index can never be greater than number of items.
            // Don't search through all items since we don't want to set home after item.
            for (uint32_t i = 0; i < new_index; ++i)
            {
                if (items_[i].type == MISSION_ITEM_SET_HOME)
                {
                    found_set_home_item = true;
                    set_home_item_index = i;
                }
            }
            if (found_set_home_item)
            {
                ROS_INFO("Executing set home item at index %u in case it was skipped.", set_home_item_index);
                bool set_home_successful = executeSetHome(items_[set_home_item_index]);
                if (!set_home_successful)
                {
                    ROS_WARN("Failed to set home.");
                }
            }
        } // end of set home checks

        current_index_ = new_index;
        ROS_INFO("Set mission index to: %u", new_index);
        return true;
    }
    
    // Attempts to start mission at current index.  Returns true if started.
    // Note 1: will run all on-deck simple commands before returning.
    // Note 2: if only simple commands remain then will return false since mission is finished by end of call.
    bool start(void)
    {
        ROS_INFO("Mission start requested.");
        started_ = true;
        
        executeToNextAction();
        
        return started_;
    }
    
    // Starts back at first command.  Does not pause mission.
    void reset(void)
    {
        ROS_INFO("Mission reset.");
        setCurrentItemIndex(0);
    }
    
    // Stops mission at current command until start is called again.  
    // If any action goals are in progress then they will be cancelled.
    void pause(void)
    {
        ROS_INFO("Mission paused.");
        started_ = false;
        
        if (guidance_client_.isServerConnected())
        {
            // This should stop the vehicle.
            guidance_client_.cancelAllGoals();
        }        
    }
    
    // Called when guidance goal is ended (either because it reached waypoint or something stopped it).
    void guidanceGoalCompleteCallback(const actionlib::SimpleClientGoalState & state, const WaypointGuidanceResultConstPtr & result)
    {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            advanceCurrentIndex();
            executeToNextAction();
        }
    }

private: // methods
    
    // Decodes command type.  'is_action' is an output parameter of whether command is 'simple' or 'action' (see class description).
    // Returns true if successful.
    bool executeItem(Command const & item, bool & is_action)
    {
        // default output parameter.
        is_action = false;
        
        bool success = false;
    
        switch (item.type)
        {
            case MISSION_ITEM_WAYPOINT:
                is_action = true;
                success = executeWaypoint(item);               
                break;
            case MISSION_ITEM_SET_HOME:
                is_action = false;
                success = executeSetHome(item);
                break;
            default:
                success = false;
                ROS_WARN("Unhandled mission item with type: %d", item.type);
                break;
        }
        
        return success;
    }
    
    // Converts command to goal for guidance action server.  Returns true if goal is sent successfully.
    bool executeWaypoint(Command const & item)
    {
        if (!guidance_client_.waitForServer(ros::Duration(1.5)))
        {
            ROS_ERROR("Cannot connect to guidance server.");
            return false;
        }

        // Set desired X and Y depending on frame.
        int32_t frame = item.param0;
        double x = 0, y = 0;

        switch (frame)
        {
        case FRAME_LOCAL:
            // This is what guidance client expects so don't need to do any conversions.
            x = item.param1;
            y = item.param2;
            break;
        case FRAME_LLA:
            ConvertLLA2ENU conversion;
            conversion.request.latitude = item.param1;
            conversion.request.longitude = item.param2;
            conversion.request.altitude = item.param3;
            bool success = convert_lla_2_enu_client_.call(conversion);

            if (!success)
            {
                ROS_WARN("Could not convert LLA waypoint to ENU.");
                return false;
            }

            x = conversion.response.east;
            y = conversion.response.north;

            break;
        }

        WaypointGuidanceGoal goal;
        goal.target_index = current_index_;
        goal.target_x = x;
        goal.target_y = y;
        goal.acceptance_radius = item.param4;
        goal.stop_at_target = (item.param5 != 0.0);

        guidance_client_.sendGoal(goal, boost::bind(&Mission::guidanceGoalCompleteCallback, this, _1, _2));

        return true; // successfully sent goal
    }
    
    // Sets home position to LLA specified in item.  Returns true if successful.
    bool executeSetHome(Command const & item)
    {
        SetHome set_home;
        set_home.request.home.latitude = item.param1;
        set_home.request.home.longitude = item.param2;
        set_home.request.home.altitude = item.param3;
        set_home.request.home.source = "mission";

        bool success = set_home_client_.call(set_home);

        return success;
    }

private: // fields

    // Used for subscriptions/publications/etc.
    ros::NodeHandle & nh_;

    // List of command items to execute.
    std::vector<Command> items_;

    // Guidance client used to move vehicle to a waypoint location.
    actionlib::SimpleActionClient<WaypointGuidanceAction> guidance_client_;
    
    // GPS Converter service client used to set home position.
    ros::ServiceClient set_home_client_;

    // Service client used to get ENU of specified LLA waypoints.
    ros::ServiceClient convert_lla_2_enu_client_;

    // Command index (in items) that is currently being executed.
    uint32_t current_index_;
    
    // True if mission has been started and is still running.
    bool started_;
    
};

/**
* Provides services for clients to interact with mission.
*/
class MissionServer
{
public: // methods

    // Constructor
    MissionServer(ros::NodeHandle & nh, Mission & mission) :
        nh_(nh),
        mission_(mission)
    {
        start_service_ = nh_.advertiseService("start_mission", &MissionServer::startMissionCallback, this);
        
        pause_service_ = nh_.advertiseService("pause_mission", &MissionServer::pauseMissionCallback, this);
        
        reset_service_ = nh_.advertiseService("reset_mission", &MissionServer::resetMissionCallback, this);
        
        add_item_service_ = nh_.advertiseService("add_mission_item", &MissionServer::addItemCallback, this);
        
        clear_all_items_service_ = nh_.advertiseService("clear_all_mission_items", &MissionServer::clearAllItemsCallback, this);
        
        set_current_item_service_ = nh_.advertiseService("set_mission_index", &MissionServer::setCurrentItemCallback, this);
    }

    bool startMissionCallback(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response)
    {
        return mission_.start();
    }
    
    bool pauseMissionCallback(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response)
    {
        mission_.pause();
        return true;
    }
    
    bool resetMissionCallback(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response)
    {
        mission_.reset();
        return true;
    }

    bool addItemCallback(AddMissionItem::Request & request, AddMissionItem::Response & response)
    {
        response.item_index = mission_.addItem(request.item);
        return true;
    }
    
    bool clearAllItemsCallback(std_srvs::Empty::Request & request, std_srvs::Empty::Response & response)
    {
        mission_.clearAllItems();
        return true;
    }
    
    bool setCurrentItemCallback(SetMissionItem::Request & request, SetMissionItem::Response & response)
    {
        return mission_.setCurrentItemIndex(request.index);
    }
    
private: // fields

    // Handle for advertising services.
    ros::NodeHandle & nh_;
    
    Mission & mission_;
    
    ros::ServiceServer start_service_;
    
    ros::ServiceServer pause_service_;
    
    ros::ServiceServer reset_service_;
    
    ros::ServiceServer add_item_service_;
    
    ros::ServiceServer clear_all_items_service_;
    
    ros::ServiceServer set_current_item_service_;
    
};

int main(int argc, char **argv)
{
    // Setup ROS node.
    ros::init(argc, argv, "mission");
    
    // Establish this program as a node. This is what actually connects to master.
    ros::NodeHandle nh;
    
    Mission mission(nh);

    MissionServer mission_server(nh, mission);
    
    // Wait for callbacks.
    ros::spin();

    return 0;
}
