Number of literals: 8
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_126) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 6.000
b (5.000 | 3.000)b (4.000 | 49.820)b (3.000 | 52.821)b (2.000 | 62.822)b (1.000 | 65.823)
; Plan found with metric 96.106
; States evaluated so far: 8
; Time 0.00
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [46.819]
49.821: (correct_position auv0 strategic_location_1)  [3.000]
52.822: (observe_inspection_point auv0 strategic_location_1 inspection_point_126)  [10.000]
62.823: (correct_position auv0 strategic_location_1)  [3.000]
65.824: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_2)  [30.282]

 * All goal deadlines now no later than 96.106

Resorting to best-first search
b (5.000 | 3.000)b (4.000 | 49.820)b (3.000 | 52.821)b (2.000 | 62.822)b (1.000 | 65.823)