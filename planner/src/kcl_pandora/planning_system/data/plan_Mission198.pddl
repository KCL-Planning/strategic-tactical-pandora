Number of literals: 8
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%] [150%] [160%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%] [140%] [150%] [160%]
Have identified that bigger values of (observed inspection_point_114) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 7.000
b (6.000 | 3.000)b (5.000 | 25.562)b (4.000 | 88.330)b (3.000 | 91.331)b (2.000 | 101.332)b (1.000 | 104.333)
; Plan found with metric 164.100
; States evaluated so far: 9
; Time 0.00
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 mission_site_start_point_2)  [22.561]
25.563: (correct_position auv0 mission_site_start_point_2)  [3.000]
28.564: (do_hover_fast auv0 mission_site_start_point_2 strategic_location_1)  [59.766]
88.331: (correct_position auv0 strategic_location_1)  [3.000]
91.332: (observe_inspection_point auv0 strategic_location_1 inspection_point_114)  [10.000]
101.333: (correct_position auv0 strategic_location_1)  [3.000]
104.334: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_2)  [59.766]

 * All goal deadlines now no later than 164.100

Resorting to best-first search
b (6.000 | 3.000)b (5.000 | 25.562)b (4.000 | 88.330)b (3.000 | 91.331)b (2.000 | 101.332)b (1.000 | 104.333)