#!/bin/bash

for i in $(ls experiments-single/); do
	echo $i
	perl constraint.perl -t 1800 -m 3200000 ~/projects/catkin_projects/ROSPlan/src/rosplan_interface_minesweeper/common/bin/popf3-clp-last -n ~/projects/catkin_projects/PANDORA/src/kcl_pandora/planning_system/data/pandora_domain_strategic.pddl experiments-single/$i > result_$i
done
