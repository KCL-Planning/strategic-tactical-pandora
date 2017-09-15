Number of literals: 8
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_80) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 6.000
b (5.000 | 3.000)b (4.000 | 77.580)b (3.000 | 80.581)b (2.000 | 90.582)b (1.000 | 93.583)
; Plan found with metric 135.833
; States evaluated so far: 8
; Time 0.00
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [74.579]
77.581: (correct_position auv0 strategic_location_1)  [3.000]
80.582: (observe_inspection_point auv0 strategic_location_1 inspection_point_80)  [10.000]
90.583: (correct_position auv0 strategic_location_1)  [3.000]
93.584: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_2)  [42.249]

 * All goal deadlines now no later than 135.833

Resorting to best-first search
b (5.000 | 3.000)b (4.000 | 77.580)b (3.000 | 80.581)b (2.000 | 90.582)b (1.000 | 93.583)