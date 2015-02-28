#!/bin/bash

# Function to type a string then hit enter to execute it.
exec_command () 
{
   xdotool type "$1"
   xdotool key Return
}

# Function to type a command without executing it.
type_command () 
{
   xdotool type "$1"
}

# Define names of tabs to open.
tab[0]="ssh1"
tab[1]="bag"
tab[2]="msn"
tab[3]="ld"
tab[4]="cvt"
tab[5]="rqt"
tab[6]="rpy"
tab[7]="cp2h"
tab[8]="cp2gs"
tab[9]="git"
tab[10]="eclipse"

# IP of robot. Default to wireless.
husky_ip="192.168.1.101" # wireless
if [ "$1" = "wired" ]; then
	husky_ip="192.168.1.11" # wired
fi

# Open new terminal to open tabs in.
xdotool key ctrl+alt+t
sleep 2

# Open each command in new tab, but don't execute it.
num_tabs=${#tab[@]}
for ((i = 0; i < num_tabs; i++))
do

    # If this is first tab just use the window we just opened up.
    if [ $i -ne 0 ]; then

        # Open new tab.
        xdotool key ctrl+shift+T
        
        # Wait for new tab to open up before typing.
        sleep 1.3
        
    fi

    # Change tab title
    # NOTE: must comment out PS1 in .bashrc for this to work.
    exec_command "echo -en \"\033]0;${tab[i]}\a\""

    # Execute commands for certain tabs.
    # Have to do everyone with xdotool so it executes in new tab.
    case  "${tab[i]}" in
        ssh1)
            type_command "ssh administrator@$husky_ip"
        ;;
	    bag)
	        inst0="After SSH:"
            inst1="cd ~/bags"
            inst2="rosbag record -O your_file_name /robot_pose_ekf/odom_combined"
		    inst3="press ctrl+C to stop"
		    exec_command "echo -e \"\n\n${inst0}\n${inst1}\n${inst2}\n${inst3}\n\""
		    sleep 0.5
		    type_command "ssh administrator@${husky_ip}"
        ;;
        msn)
        	exec_command "export ROS_MASTER_URI=http://${husky_ip}:11311"
		    type_command "rosservice call /start_mission"
        ;;
        ld)
		    exec_command "export ROS_MASTER_URI=http://${husky_ip}:11311"
            type_command "rosrun htp_auto mission_loader _mission_file:=\"/home/htp/missions/your_file.mission\""
        ;;
        cvt)
		    exec_command "cd ~/ros/src/htp_auto/tools/"
		    sleep .2
            exec_command "python bag2mission.py --help"
            sleep 1
            type_command "python bag2mission.py ~/bags/your_file.bag ~/missions/your_file.mission"
        ;;
	    rqt)
            exec_command "export ROS_MASTER_URI=http://${husky_ip}:11311"
		    type_command "rqt"
        ;;
        rpy)
            exec_command "export ROS_MASTER_URI=http://${husky_ip}:11311"
		    type_command "rosrun htp_auto quat_to_euler odom:=robot_pose_ekf/odom_combined rpy_odom:=odom_euler"
        ;;
        scp2h)
		    exec_command "echo -e \"\nTo move htp package to husky\n\""
            type_command "scp -pr ~/ros/src/htp_auto/. administrator@${husky_ip}:/home/administrator/ros/src/htp_auto"
        ;;
        scp2gs)
		    exec_command "echo -e \"\nTo move log file back from husky\n\""
		    type_command "scp -p administrator@"${husky_ip}":/home/administrator/bags/your_file.bag ~/bags/your_file.bag"
        ;;
        git)
            exec_command "cd ~/ros/src/htp_auto"
            exec_command "git status"
        ;;
        eclipse)
            type_command "bash -i -c eclipse44"
        ;;
        *)              
    esac 

done

exit 0
