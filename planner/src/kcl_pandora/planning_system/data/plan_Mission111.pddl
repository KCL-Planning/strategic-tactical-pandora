Number of literals: 8
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_66) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 6.000
b (5.000 | 3.000)b (4.000 | 68.773)b (3.000 | 71.774)b (2.000 | 81.775)b (1.000 | 84.776)
; Plan found with metric 136.440
; States evaluated so far: 8
; Time 0.00
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [65.772]
68.774: (correct_position auv0 strategic_location_1)  [3.000]
71.775: (observe_inspection_point auv0 strategic_location_1 inspection_point_66)  [10.000]
81.776: (correct_position auv0 strategic_location_1)  [3.000]
84.777: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_2)  [51.662]

 * All goal deadlines now no later than 136.440

Resorting to best-first search
b (5.000 | 3.000)b (4.000 | 68.773)b (3.000 | 71.774)b (2.000 | 81.775)b (1.000 | 84.776)