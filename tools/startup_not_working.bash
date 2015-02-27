#!/bin/bash

# Define names of tabs to open.
tab[0]="ssh1"
tab[1]="ssh_bag"
tab[2]="mission"
tab[3]="loader"
tab[4]="rqt1"
tab[5]="rqt2"
tab[6]="rpy"
tab[7]="scp2h"
tab[8]="scp2gs"
tab[9]="sim"
tab[10]="git"
tab[11]="eclipse"

num_tabs="${#tab[@]}"

args=" --window --maximize"

for ((i = 0; i < num_tabs; i++))
do
      # Build command that gets passed with -e flag
      cmd="\"env MYTAB="
      cmd+="${tab[i]}"

      if [ "$1" = "wired" ]; then
           cmd+=" env HUSKY_WIRED=true"
      fi

      cmd+=" bash\""
      
      # Append on to running argument string.
      args+=" --tab -t "
      args+="${tab[i]}" 
      args+=" -e "
      args+="$cmd"
done

command="gnome-terminal"
command+="${args[@]}"

# Echo full command so I can copy and paste it.  This works.
echo "$command"

# Doesn't work.  Just opens a new terminal with nothing in it.
#echo "${args[@]}"
#gnome-terminal "${args[@]}"

exit 0

