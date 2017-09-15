Number of literals: 8
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_179) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 6.000
b (5.000 | 3.000)b (4.000 | 45.155)b (3.000 | 48.156)b (2.000 | 58.157)b (1.000 | 61.158)
; Plan found with metric 129.365
; States evaluated so far: 8
; Time 0.00
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [42.154]
45.156: (correct_position auv0 strategic_location_1)  [3.000]
48.157: (observe_inspection_point auv0 strategic_location_1 inspection_point_179)  [10.000]
58.158: (correct_position auv0 strategic_location_1)  [3.000]
61.159: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_3)  [68.206]

 * All goal deadlines now no later than 129.365

Resorting to best-first search
b (5.000 | 3.000)b (4.000 | 45.155)b (3.000 | 48.156)b (2.000 | 58.157)b (1.000 | 61.158)