Number of literals: 29
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Have identified that bigger values of (observed inspection_point_0) are preferable
Have identified that bigger values of (observed inspection_point_1) are preferable
Have identified that bigger values of (observed inspection_point_2) are preferable
Have identified that bigger values of (observed inspection_point_3) are preferable
Have identified that bigger values of (observed inspection_point_4) are preferable
Have identified that bigger values of (observed inspection_point_5) are preferable
Have identified that bigger values of (observed inspection_point_6) are preferable
Have identified that bigger values of (observed inspection_point_7) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 21.000
b (20.000 | 3.000)b (19.000 | 47.393)b (18.000 | 90.060)b (17.000 | 100.061)b (16.000 | 103.062)b (15.000 | 139.727)b (14.000 | 187.979)b (13.000 | 214.106)b (12.000 | 217.107)b (11.000 | 231.814)b (10.000 | 234.815)b (9.000 | 275.177)b (8.000 | 326.645)b (7.000 | 329.646)b (6.000 | 339.647)b (5.000 | 342.648)b (4.000 | 349.650)b (3.000 | 359.651)b (2.000 | 362.652)b (1.000 | 366.653)
; Plan found with metric 416.704
; States evaluated so far: 37
; Time 0.02
0.000: (correct_position auv0 wp_auv0)  [3.000]
3.001: (do_hover_fast auv0 wp_auv0 mission_site_start_point_1)  [44.392]
47.394: (correct_position auv0 mission_site_start_point_1)  [3.000]
50.395: (do_hover_fast auv0 mission_site_start_point_1 strategic_location_8)  [36.664]
87.060: (correct_position auv0 strategic_location_8)  [3.000]
90.061: (observe_inspection_point auv0 strategic_location_8 inspection_point_7)  [10.000]
100.062: (correct_position auv0 strategic_location_8)  [3.000]
103.063: (do_hover_fast auv0 strategic_location_8 mission_site_start_point_1)  [36.664]
139.728: (correct_position auv0 mission_site_start_point_1)  [3.000]
142.729: (do_hover_fast auv0 mission_site_start_point_1 strategic_location_5)  [42.249]
184.979: (correct_position auv0 strategic_location_5)  [3.000]
187.980: (do_hover_fast auv0 strategic_location_5 strategic_location_7)  [13.124]
201.105: (correct_position auv0 strategic_location_7)  [3.000]
204.106: (observe_inspection_point auv0 strategic_location_7 inspection_point_6)  [10.000]
214.107: (correct_position auv0 strategic_location_7)  [3.000]
217.108: (do_hover_fast auv0 strategic_location_7 strategic_location_6)  [14.705]
231.815: (correct_position auv0 strategic_location_6)  [3.000]
234.816: (do_hover_fast auv0 strategic_location_6 mission_site_start_point_1)  [40.361]
275.178: (correct_position auv0 mission_site_start_point_1)  [3.000]
278.179: (do_hover_fast auv0 mission_site_start_point_1 strategic_location_2)  [48.466]
326.646: (correct_position auv0 strategic_location_2)  [3.000]
329.647: (observe_inspection_point auv0 strategic_location_2 inspection_point_1)  [10.000]
339.648: (correct_position auv0 strategic_location_2)  [3.000]
342.649: (do_hover_fast auv0 strategic_location_2 strategic_location_3)  [4.000]
346.650: (correct_position auv0 strategic_location_3)  [3.000]
349.651: (observe_inspection_point auv0 strategic_location_3 inspection_point_2)  [10.000]
359.652: (correct_position auv0 strategic_location_3)  [3.000]
362.653: (do_hover_controlled auv0 strategic_location_3 strategic_location_1)  [4.000]
366.654: (do_hover_fast auv0 strategic_location_1 mission_site_start_point_1)  [50.050]

 * All goal deadlines now no later than 416.704

Resorting to best-first search
b (20.000 | 3.000)b (19.000 | 47.393)b (19.000 | 7.328)b (18.000 | 11.655)b (17.000 | 51.067)b (16.000 | 54.068)b (15.000 | 68.775)b (14.000 | 80.318)b (13.000 | 122.569)b (12.000 | 261.752)b (12.000 | 238.147)b (11.000 | 248.148)b (10.000 | 251.149)b (9.000 | 287.814)b (9.000 | 278.755)b (8.000 | 292.756)b (7.000 | 295.757)b (6.000 | 299.758)b (5.000 | 349.809)