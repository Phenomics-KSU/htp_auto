#!/bin/bash

com[0]="Command One"
com[1]="Command Two"

# Open each command in new tab, but don't execute it.
num_commands=${#com[@]}
for ((i = 0; i < num_commands; i++))
do
    # Open new tab.
    xdotool key ctrl+shift+T

    # Wait for new tab to open up before typing.
    sleep 1

    # Doesn't work since script is still running in first shell.
    # not the tab that was just created. 
    #echo -en "\033]0;New terminal title\a"

    xdotool type "${com[i]}"
done

exit 0
