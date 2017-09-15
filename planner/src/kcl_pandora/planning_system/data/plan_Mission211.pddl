Number of literals: 8
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_127) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 6.000
b (5.000 | 3.000)b (4.000 | 41.679)b (3.000 | 44.680)b (2.000 | 54.681)b (1.000 | 57.682)
; Plan found with metric 81.787
; States evaluated so far: 8
; Time 0.00
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [38.678]
41.680: (correct_position auv0 strategic_location_1)  [3.000]
44.681: (observe_inspection_point auv0 strategic_location_1 inspection_point_127)  [10.000]
54.682: (correct_position auv0 strategic_location_1)  [3.000]
57.683: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_2)  [24.104]

 * All goal deadlines now no later than 81.787

Resorting to best-first search
b (5.000 | 3.000)b (4.000 | 41.679)b (3.000 | 44.680)b (2.000 | 54.681)b (1.000 | 57.682)