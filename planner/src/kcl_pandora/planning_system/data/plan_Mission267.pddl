Number of literals: 8
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_186) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 6.000
b (5.000 | 3.000)b (4.000 | 87.600)b (3.000 | 90.601)b (2.000 | 100.602)b (1.000 | 103.603)
; Plan found with metric 134.459
; States evaluated so far: 8
; Time 0.00
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [84.599]
87.601: (correct_position auv0 strategic_location_1)  [3.000]
90.602: (observe_inspection_point auv0 strategic_location_1 inspection_point_186)  [10.000]
100.603: (correct_position auv0 strategic_location_1)  [3.000]
103.604: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_3)  [30.855]

 * All goal deadlines now no later than 134.459

Resorting to best-first search
b (5.000 | 3.000)b (4.000 | 87.600)b (3.000 | 90.601)b (2.000 | 100.602)b (1.000 | 103.603)