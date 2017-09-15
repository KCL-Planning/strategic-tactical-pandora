Number of literals: 11
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that smaller values of (arm_calibration auv0) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
94% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 4.000
b (3.000 | 55.911)b (2.000 | 58.912)b (1.000 | 68.913)
; Plan found with metric 109.063
; States evaluated so far: 6
; Time 0.00
0.000: (do_hover_fast auv0 wp_auv0 strategic_location_1)  [55.911]
55.912: (correct_position auv0 strategic_location_1)  [3.000]
58.913: (examine_panel auv0 strategic_location_1 valve_panel_001)  [10.000]
58.914: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_2)  [50.150]

 * All goal deadlines now no later than 109.063

Resorting to best-first search
b (3.000 | 55.911)b (2.000 | 58.912)b (1.000 | 68.913)