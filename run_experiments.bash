#!/bin/bash

for i in $(ls data/pandora/levels); do
	echo START $i
	./bin/visualiser --level=data/pandora/levels/$i &
	sleep 30
	roslaunch planning_system kcl_planning_system.launch
	killall visualiser
	
	echo experiments-single/$i.pddl
	cp /home/bram/projects/catkin_projects/PANDORA/src/kcl_pandora/planning_system/data/pandora_strategic_problem.pddl experiments-single/$i.pddl
done
