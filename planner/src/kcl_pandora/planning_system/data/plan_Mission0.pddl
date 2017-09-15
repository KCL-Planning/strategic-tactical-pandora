Number of literals: 17
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Have identified that smaller values of (arm_calibration auv0) are preferable
Have identified that bigger values of (valve_goal_completed valve_000) are preferable
Have identified that bigger values of (valve_goal_completed valve_001) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
92% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 11.000
b (10.000 | 22.561)b (9.000 | 75.713)b (8.000 | 78.714)b (7.000 | 88.715)b (6.000 | 198.716)b (5.000 | 201.717)b (4.000 | 291.718)b (3.000 | 294.719)b (2.000 | 414.720)b (1.000 | 417.721)
; Plan found with metric 467.872
; States evaluated so far: 21
; Time 0.00
0.000: (do_hover_fast auv0 wp_auv0 mission_site_start_point_1)  [22.561]
22.562: (correct_position auv0 mission_site_start_point_1)  [3.000]
25.563: (do_hover_fast auv0 mission_site_start_point_1 strategic_location_1)  [50.150]
75.714: (correct_position auv0 strategic_location_1)  [3.000]
78.715: (examine_panel auv0 strategic_location_1 valve_panel_000)  [10.000]
78.716: (turn_valve auv0 strategic_location_1 valve_panel_000 valve_000)  [120.000]
198.717: (correct_position auv0 strategic_location_1)  [3.000]
201.718: (do_hover_fast auv0 strategic_location_1 strategic_location_2)  [90.000]
291.719: (correct_position auv0 strategic_location_2)  [3.000]
294.720: (turn_valve auv0 strategic_location_2 valve_panel_001 valve_001)  [120.000]
414.721: (correct_position auv0 strategic_location_2)  [3.000]
417.722: (do_hover_fast auv0 strategic_location_2 mission_site_start_point_1)  [50.150]

 * All goal deadlines now no later than 467.872

Resorting to best-first search
b (10.000 | 45.122)b (8.000 | 111.821)b (7.000 | 301.823)b (6.000 | 411.824)b (5.000 | 414.825)b (5.000 | 324.824)b (4.000 | 414.824)b (3.000 | 424.825)b (3.000 | 337.826)b (3.000 | 284.917)